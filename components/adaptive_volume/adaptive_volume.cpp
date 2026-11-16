#include "adaptive_volume.h"

#ifdef USE_ESP32

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <esp_timer.h>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace adaptive_volume {

static const char *const TAG = "adaptive_volume";
static constexpr uint32_t S_PREF_HASH = 0x5A7E1D0Bu;
static constexpr uint32_t P_PREF_HASH = 0x5A7E1D0Du;  // path gain P, cross-spectral (0.4.1)
static constexpr float P_SAVE_STEP_DB = 1.0f;          // flash P when it moved this much since the last save
static constexpr uint32_t PATH_STEP_BUDGET_US = 4000;  // analysis time per main-loop pass
// Path-mode acceptance of one reply (cross-spectral estimate)
static constexpr int P_MIN_WINDOWS = 10;           // 10 windows of 32 ms (50 % overlap)
static constexpr float P_MIN_COVERAGE = 0.6f;      // share of the weighting excited by the sound
static constexpr float P_MIN_COHERENCE = 0.2f;     // weighted mean coherence
static constexpr uint32_t REPORT_INTERVAL_MS = 10000;

#ifdef USE_SWITCH
void EnableSwitch::write_state(bool state) {
  this->publish_state(state);
  this->parent_->set_enabled(state);
}
#endif

#ifdef USE_NUMBER
void OffsetNumber::restore() {
  this->pref_ = this->make_entity_preference<float>();
  float value = 0.0f;
  if (!this->pref_.load(&value) || !std::isfinite(value))
    value = 0.0f;
  this->publish_state(value);
  this->parent_->set_offset(value);
}
void OffsetNumber::control(float value) {
  this->pref_.save(&value);
  this->publish_state(value);
  this->parent_->set_offset(value);
}
#endif

void AdaptiveVolume::setup() {
  if (this->stack_ != nullptr) {
    this->core_.cfg.sample_rate = static_cast<float>(this->stack_->get_output_sample_rate());
    this->stack_->set_capture_observer(this);
  }
  if (this->media_player_ != nullptr) {
    // Volume changes: show "Current volume" at once
    this->media_player_->add_on_state_callback([this](media_player::MediaPlayerState) {
      if (this->media_player_->volume != this->last_volume_) {
        this->last_volume_ = this->media_player_->volume;
        this->publish_reports_(true);
      }
    });
  }
  if (this->core_.cfg.use_path) {
    RAMAllocator<float> fa;  // PSRAM first
    RAMAllocator<int16_t> ia;
    RAMAllocator<uint8_t> ba;
    this->cap_max_ = CAP_SECONDS * static_cast<uint32_t>(this->core_.cfg.sample_rate);
    this->cap_ = ia.allocate(2 * this->cap_max_);
    this->cap_flag_ = ba.allocate(this->cap_max_ / CHUNK + 1);
    if (!this->sp_.attach(fa.allocate(SpectralPath::MEM_FLOATS)) || this->cap_ == nullptr ||
        this->cap_flag_ == nullptr) {
      ESP_LOGE(TAG, "Could not allocate the path measurement; S stays at the nominal path gain");
      this->cap_max_ = 0;
    }
    // Path mode: P is stored; the S_k median (diagnostic) starts fresh
    this->p_pref_ = global_preferences->make_preference<float>(P_PREF_HASH);
    float stored = NAN;
    if (this->p_pref_.load(&stored) && std::isfinite(stored)) {
      this->core_.restore_p(stored);
      this->saved_p_ = stored;
    }
  } else {
    this->s_pref_ = global_preferences->make_preference<float>(S_PREF_HASH);
    float stored = NAN;
    if (this->s_pref_.load(&stored) && std::isfinite(stored)) {
      this->core_.restore_s(stored);
      this->saved_s_ = stored;
    }
  }
#ifdef USE_SENSOR
  if (this->core_.cfg.use_path) {
    if (this->path_gain_sensor_ != nullptr && !std::isnan(this->core_.p))
      this->path_gain_sensor_->publish_state(this->core_.p);
  } else if (this->speaker_level_sensor_ != nullptr && !std::isnan(this->core_.s)) {
    this->speaker_level_sensor_->publish_state(this->core_.s);
  }
#endif
#ifdef USE_NUMBER
  if (this->offset_number_ != nullptr)
    this->offset_number_->restore();
#endif
#ifdef USE_SWITCH
  if (this->enable_switch_ != nullptr) {
    auto initial = this->enable_switch_->get_initial_state_with_restore_mode();
    const bool on = initial.has_value() ? *initial : false;
    this->enable_switch_->publish_state(on);
    this->core_.enabled = on;
  } else {
    this->core_.enabled = true;
  }
#else
  this->core_.enabled = true;
#endif
  this->apply_(true);
}

void AdaptiveVolume::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Adaptive volume v%s:\n"
                "  Speaker level nominal: %.1f dBFS (stored: %.1f)\n"
                "  Loudness constant: %.1f dB, max attenuation %.1f dB, hysteresis %.1f dB\n"
                "  Master curve min: %.1f dB, software volume min: %.1f dB, media volume %.2f..%.2f\n"
                "  Reply: start delay %" PRIu32 " ms, minimum %.2f s\n"
                "  Sample rate: %.0f Hz\n"
                "  Path mode: %s (analogue gain %.1f dB, P %.1f dB stored)",
                ADAPTIVE_VOLUME_VERSION, this->core_.cfg.speaker_level_nominal, this->core_.s, this->core_.cfg.loudness_constant,
                this->core_.cfg.max_attenuation, this->core_.cfg.hysteresis, this->master_min_db_,
                this->core_.cfg.software_volume_min_db, this->core_.cfg.media_volume_min,
                this->core_.cfg.media_volume_max, this->core_.cfg.reply_start_delay_us / 1000u,
                this->core_.cfg.reply_min_duration_s, this->core_.cfg.sample_rate, YESNO(this->core_.cfg.use_path),
                this->core_.cfg.analog_gain_db, this->core_.p);
}

// ---------------------------------------------------------------- audio task

bool AdaptiveVolume::capture_active(esp_audio_stack::CapturePoint point) const {
  return point == esp_audio_stack::CapturePoint::PROCESSOR_INPUT ||
         point == esp_audio_stack::CapturePoint::PROCESSOR_OUTPUT ||
         point == esp_audio_stack::CapturePoint::REFERENCE;
}

void AdaptiveVolume::push_(const Block &b) {
  const uint32_t head = this->ring_head_.load(std::memory_order_relaxed);
  const uint32_t next = (head + 1) % RING;
  if (next == this->ring_tail_.load(std::memory_order_acquire)) {
    this->ring_dropped_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  this->ring_[head] = b;
  this->ring_head_.store(next, std::memory_order_release);
}

void AdaptiveVolume::on_capture_frame(const esp_audio_stack::CaptureFrame &frame) {
  if (frame.bits != 16 || frame.data == nullptr || frame.frames == 0 || frame.stride == 0)
    return;
  const int16_t *smp = static_cast<const int16_t *>(frame.data);
  const size_t stride = frame.stride;
  Block b{};
  b.n = static_cast<uint32_t>(frame.frames);
  b.ts_us = frame.timestamp_us;
  if (frame.point == esp_audio_stack::CapturePoint::REFERENCE) {
    this->on_reference_frame_(frame);
    return;
  }
  AWeighting *w;
  if (frame.point == esp_audio_stack::CapturePoint::PROCESSOR_INPUT) {
    w = &this->aw_in_;  // mic 1 (first channel), before echo cancellation and BSS
    b.point = 0;
    // Largest sample over all mic channels (clipping check)
    const size_t ch = std::max<size_t>(1, std::min<size_t>(frame.channels, stride));
    int32_t pk = 0;
    for (size_t i = 0; i < frame.frames; i++)
      for (size_t c = 0; c < ch; c++)
        pk = std::max<int32_t>(pk, std::abs(static_cast<int32_t>(smp[i * stride + c])));
    b.peak = static_cast<uint16_t>(std::min<int32_t>(pk, 65535));
  } else if (frame.point == esp_audio_stack::CapturePoint::PROCESSOR_OUTPUT) {
    // AFE output before the stack's Mic Gain trim, so the trim (which S at mic 1
    // does not see) cannot shift N against S, nor the echo reduction.
    w = &this->aw_out_;  // processed output (room noise)
    b.point = 1;
  } else {
    return;
  }
  if (frame.sample_rate != w->rate && frame.sample_rate > 0)
    w->setup(frame.sample_rate);
  // Both points are A-weighted, so the speaker and room levels are measured in
  // the same band as the room noise (also keeps low-frequency hum out).
  double sum = 0.0;
  // mic 1 raw, kept for the path measurement with the reference frame that follows
  const bool keep = b.point == 0 && frame.frames <= MIC_MAX && this->cap_on_.load(std::memory_order_relaxed);
  for (size_t i = 0; i < frame.frames; i++) {
    const float y = w->process(smp[i * stride] * (1.0f / 32768.0f));
    sum += static_cast<double>(y) * y;
    if (keep)
      this->mic_raw_[i] = smp[i * stride];
  }
  if (b.point == 0)
    this->mic_raw_n_ = keep ? static_cast<uint32_t>(frame.frames) : 0;
  b.sum_sq = static_cast<float>(sum);
  this->push_(b);
}

// Reference frame (speaker signal after volume and codec attenuation). While a reply
// is measured: copy it and the mic frame of the same cycle into the buffer, with the
// sound gate (reference 20 dB above its idle floor). Light work only.
void AdaptiveVolume::on_reference_frame_(const esp_audio_stack::CaptureFrame &frame) {
  const int16_t *smp = static_cast<const int16_t *>(frame.data);
  const size_t stride = frame.stride;
  const uint32_t len = static_cast<uint32_t>(frame.frames);
  if (frame.sample_rate != this->aw_ref_.rate && frame.sample_rate > 0)
    this->aw_ref_.setup(frame.sample_rate);
  double sum = 0.0;
  int32_t pk = 0;
  for (uint32_t i = 0; i < len; i++) {
    const int16_t v = smp[i * stride];
    pk = std::max<int32_t>(pk, std::abs(static_cast<int32_t>(v)));
    const float x = this->aw_ref_.process(v * (1.0f / 32768.0f));
    sum += static_cast<double>(x) * x;
  }
  const float level = sum > 0.0 ? static_cast<float>(10.0 * std::log10(sum / len)) : -130.0f;
  const bool on = this->cap_on_.load(std::memory_order_acquire);
  if (!on) {
    // idle floor of the reference: falls at once, rises slowly
    if (level < this->ref_floor_db_)
      this->ref_floor_db_ = level;
    else
      this->ref_floor_db_ += 0.01f * (level - this->ref_floor_db_);
    this->mic_raw_n_ = 0;
    return;
  }
  const uint32_t n = this->cap_n_.load(std::memory_order_relaxed);
  if (this->mic_raw_n_ != len || this->cap_ == nullptr || n + len > this->cap_max_) {
    this->mic_raw_n_ = 0;
    return;
  }
  const uint16_t p16 = static_cast<uint16_t>(std::min<int32_t>(pk, 65535));
  if (p16 > this->ref_peak_.load(std::memory_order_relaxed))
    this->ref_peak_.store(p16, std::memory_order_relaxed);
  int16_t *dst = this->cap_ + 2 * n;
  for (uint32_t i = 0; i < len; i++) {
    dst[2 * i] = smp[i * stride];
    dst[2 * i + 1] = this->mic_raw_[i];
  }
  const uint8_t sound = level >= std::max(this->ref_floor_db_ + 20.0f, -75.0f) ? 1 : 0;
  for (uint32_t c = n / CHUNK; c <= (n + len - 1) / CHUNK; c++)
    this->cap_flag_[c] = sound;
  this->cap_n_.store(n + len, std::memory_order_release);
  this->mic_raw_n_ = 0;
}

// A reply starts: an unfinished analysis of the previous one is dropped.
void AdaptiveVolume::path_start_() {
  if (this->an_busy_) {
    ESP_LOGD(TAG, "Path: previous reply not analysed (a new one started)");
    this->an_busy_ = false;
  }
  this->cap_n_.store(0, std::memory_order_relaxed);
  this->ref_peak_.store(0, std::memory_order_relaxed);
}

// The reply ended (capture already off): start analysing what was captured.
void AdaptiveVolume::path_finish_(const CalResult &r) {
  this->an_end_ = this->cap_n_.load(std::memory_order_acquire);
  this->an_pos_ = 0;
  this->an_xx_ = this->an_yy_ = 0.0;
  this->an_n_ = 0;
  this->an_v0_ = r.v0;
  this->an_m0_ = r.m0;
  this->an_peak_ = this->ref_peak_.load(std::memory_order_relaxed);
  const uint32_t rate = static_cast<uint32_t>(this->core_.cfg.sample_rate);
  this->an_x_.setup(rate);
  this->an_y_.setup(rate);
  this->sp_.reset();
  this->an_busy_ = this->sp_.ready() && this->an_end_ >= CHUNK;
  if (!this->an_busy_)
    ESP_LOGD(TAG, "Path: nothing captured, not counted");
}

// Analyse a part of the capture (time-limited); true when done.
bool AdaptiveVolume::path_step_() {
  const uint32_t t0 = micros();
  float xb[CHUNK], yb[CHUNK];
  while (this->an_pos_ + CHUNK <= this->an_end_) {
    const int16_t *src = this->cap_ + 2 * this->an_pos_;
    const bool sound = this->cap_flag_[this->an_pos_ / CHUNK] != 0;
    for (uint32_t i = 0; i < CHUNK; i++) {
      xb[i] = this->an_x_.process(src[2 * i] * (1.0f / 32768.0f));
      yb[i] = this->an_y_.process(src[2 * i + 1] * (1.0f / 32768.0f));
      if (sound) {
        this->an_xx_ += static_cast<double>(xb[i]) * xb[i];
        this->an_yy_ += static_cast<double>(yb[i]) * yb[i];
      }
    }
    if (sound)
      this->an_n_ += CHUNK;
    this->sp_.push(xb, yb, CHUNK, sound);
    this->an_pos_ += CHUNK;
    if (micros() - t0 > PATH_STEP_BUDGET_US)
      return false;
  }
  return true;
}

// Analysis done: log, check, and update P.
void AdaptiveVolume::path_result_() {
  this->an_busy_ = false;
  const SpectralPath::Result sp = this->sp_.result(this->core_.cfg.sample_rate);
  const float seconds = this->an_n_ / this->core_.cfg.sample_rate;
  const float mic_db = this->an_n_ > 0 ? static_cast<float>(10.0 * std::log10(this->an_yy_ / this->an_n_ + 1e-13)) : NAN;
  const float ref_db = this->an_n_ > 0 ? static_cast<float>(10.0 * std::log10(this->an_xx_ / this->an_n_ + 1e-13)) : NAN;
  const float peak_db = this->an_peak_ > 0 ? 20.0f * std::log10(this->an_peak_ / 32768.0f) : -96.0f;
  // Reply level at the reference (volume and attenuation removed, 0 dB analogue gain)
  const float reply_ref = ref_db - this->core_.v_db(this->an_v0_) + this->an_m0_ - this->core_.cfg.analog_gain_db;
  ESP_LOGD(TAG,
           "Path: %.1f dB (coverage %.2f, coherence %.2f, %.2f s); mic %.1f, ref %.1f, ref peak %.1f dBFS; "
           "reply level %.1f dBFS at the reference",
           sp.p_db, sp.coverage, sp.coherence, seconds, mic_db, ref_db, peak_db, reply_ref);
  const char *why = nullptr;
  if (std::isnan(sp.p_db) || sp.windows < P_MIN_WINDOWS || seconds < this->core_.cfg.active_min_duration_s)
    why = "too little sound";
  else if (sp.coverage < P_MIN_COVERAGE)
    why = "the sound covers too little of the band";
  else if (sp.coherence < P_MIN_COHERENCE)
    why = "coherence too low (talking or noise)";
  if (why != nullptr) {
    ESP_LOGD(TAG, "Path: not counted (%s)", why);
    return;
  }
#ifdef USE_SENSOR
  if (this->path_gain_reply_sensor_ != nullptr)
    this->path_gain_reply_sensor_->publish_state(sp.p_db);
#endif
  const float s_old = this->core_.s_use();
  this->core_.push_p(sp.p_db);
  ESP_LOGD(TAG, "Path: P %.1f dB (median of %d) -> S %.1f (was %.1f)", this->core_.p, this->core_.p_count(),
           this->core_.s_use(), s_old);
  float p = this->core_.p;
  if (std::isnan(this->saved_p_) || std::fabs(p - this->saved_p_) >= P_SAVE_STEP_DB) {
    this->p_pref_.save(&p);
    this->saved_p_ = p;
  }
#ifdef USE_SENSOR
  if (this->path_gain_sensor_ != nullptr)
    this->path_gain_sensor_->publish_state(p);
#endif
  this->apply_(false);
}

// ---------------------------------------------------------------- main loop

void AdaptiveVolume::loop() {
  const bool idle = this->idle_f_ ? this->idle_f_() : false;
  bool noise_updated = false;
  // Echo reduction: one session per stretch of playback
  const bool playing = this->media_player_ != nullptr &&
                       (this->media_player_->state == media_player::MEDIA_PLAYER_STATE_PLAYING ||
                        this->media_player_->state == media_player::MEDIA_PLAYER_STATE_ANNOUNCING);
  if (playing && !this->echo_.active)
    this->echo_.start();
  else if (!playing && this->echo_.active)
    this->echo_finish_();
  uint32_t tail = this->ring_tail_.load(std::memory_order_relaxed);
  const uint32_t head = this->ring_head_.load(std::memory_order_acquire);
  while (tail != head) {
    const Block &b = this->ring_[tail];
    if (b.point == 0) {
      this->core_.on_input(b.sum_sq, b.n, b.ts_us, idle);
      if (this->echo_.active)
        this->echo_.add_input(b.sum_sq, b.n, b.peak);
    } else {
      if (this->core_.on_aweighted(b.sum_sq, b.n, idle))
        noise_updated = true;
      if (this->echo_.active)
        this->echo_.add_output(b.sum_sq, b.n);
    }
    tail = (tail + 1) % RING;
  }
  this->ring_tail_.store(tail, std::memory_order_release);

  const uint32_t dropped = this->ring_dropped_.exchange(0, std::memory_order_relaxed);
  if (dropped > 0) {
    ESP_LOGW(TAG, "%" PRIu32 " audio blocks dropped (main loop busy)", dropped);
  }

  // Reply measurement
  const bool reply = this->reply_f_ ? this->reply_f_() : false;
  const bool blocked = this->blocked_f_ ? this->blocked_f_() : false;
  const float v = this->media_player_ != nullptr ? this->media_player_->volume : 1.0f;
  const CalResult r = this->core_.reply_update(reply, static_cast<uint64_t>(esp_timer_get_time()), v, blocked);
  if (this->core_.cfg.use_path && this->cap_max_ > 0) {
    const bool measuring = this->core_.measuring();
    if (measuring && !this->cap_on_.load(std::memory_order_relaxed))
      this->path_start_();
    this->cap_on_.store(measuring, std::memory_order_release);
  }
  if (r.kind != CalResult::NONE) {
    this->log_result_(r);
    if (this->core_.cfg.use_path) {
#ifdef USE_SENSOR
      if (r.kind == CalResult::DETECTED && this->speaker_level_reply_sensor_ != nullptr)
        this->speaker_level_reply_sensor_->publish_state(r.s_k);
#endif
      if (this->cap_max_ > 0)
        this->path_finish_(r);
    } else if (r.kind == CalResult::DETECTED || r.kind == CalResult::LOWERED) {
      float s = this->core_.s;
      // Flash only when S moved by 1 dB or more since the last save
      if (std::isnan(this->saved_s_) || std::fabs(s - this->saved_s_) >= 1.0f) {
        this->s_pref_.save(&s);
        this->saved_s_ = s;
      }
#ifdef USE_SENSOR
      if (this->speaker_level_sensor_ != nullptr)
        this->speaker_level_sensor_->publish_state(s);
      if (r.kind == CalResult::DETECTED && this->speaker_level_reply_sensor_ != nullptr)
        this->speaker_level_reply_sensor_->publish_state(r.s_k);
#endif
      this->apply_(false);
    }
  }

  if (this->an_busy_ && this->path_step_())
    this->path_result_();

  if (noise_updated)
    this->apply_(false);
  this->publish_reports_(false);
}

void AdaptiveVolume::echo_finish_() {
  const EchoResult e = this->echo_.finish(this->core_.room_pre, this->core_.n_a, this->core_.cfg.sample_rate);
  if (std::isnan(e.peak_dbfs)) {
    ESP_LOGV(TAG, "Echo: playback not measured (%s, %.1f s)", e.reason, e.seconds);
    return;
  }
  if (e.valid) {
    ESP_LOGD(TAG,
             "Echo: %.1f s playback, mic 1 %.1f dBFS (room %.1f), output %.1f (room %.1f) -> echo reduction %s%.1f dB, "
             "mic peak %.1f dBFS",
             e.seconds, e.in_l, e.room_in, e.out_l, e.room_out, e.lower_bound ? ">= " : "", e.erle, e.peak_dbfs);
  } else {
    ESP_LOGD(TAG, "Echo: %.1f s playback, echo reduction not measured (%s), mic peak %.1f dBFS", e.seconds, e.reason,
             e.peak_dbfs);
  }
#ifdef USE_SENSOR
  if (e.valid && this->echo_reduction_sensor_ != nullptr)
    this->echo_reduction_sensor_->publish_state(e.erle);
  if (this->mic_peak_sensor_ != nullptr)
    this->mic_peak_sensor_->publish_state(e.peak_dbfs);
#endif
}

void AdaptiveVolume::log_result_(const CalResult &r) {
  switch (r.kind) {
    case CalResult::SKIPPED:
      ESP_LOGD(TAG, "Speaker calibration: reply not counted (%s; %.2f s, speech %.2f s)", r.reason, r.seconds,
               r.active_s);
      break;
    case CalResult::DETECTED:
      ESP_LOGD(TAG,
               "Speaker calibration: speech %.1f (%.2f of %.2f s), room %.1f, volume %.0f%%, attenuation %.0f dB "
               "-> S_k %.1f%s, S_k median %.1f (of %d, S in use %.1f)",
               r.in_l, r.active_s, r.seconds, r.room_l, r.v0 * 100.0f, r.m0, r.s_k,
               r.marginal ? " (marginal, lowers only)" : "", r.s_new, r.hist_n, r.s_old);
      break;
    case CalResult::LOWERED:
      ESP_LOGD(TAG,
               "Speaker calibration: no speech above room (input %.1f, room %.1f, volume %.0f%%, attenuation %.0f dB): "
               "S %.1f > bound %.1f -> S %.1f",
               r.in_l, r.room_l, r.v0 * 100.0f, r.m0, r.s_old, r.bound, r.s_new);
      break;
    case CalResult::WITHIN_BOUND:
      ESP_LOGD(TAG, "Speaker calibration: no speech above room (input %.1f, room %.1f): S %.1f within bound %.1f", r.in_l,
               r.room_l, r.s_old, r.bound);
      break;
    default:
      break;
  }
}

void AdaptiveVolume::set_enabled(bool enabled) {
  this->core_.enabled = enabled;
  this->apply_(true);
}

void AdaptiveVolume::set_offset(float db) {
  this->core_.offset = db;
  this->apply_(true);
}

void AdaptiveVolume::set_announcement_offset(float db) {
  if (db == this->core_.boost)
    return;
  this->core_.boost = db;
  ESP_LOGD(TAG, "Announcement offset %.0f dB", db);
  this->apply_(false);
}

void AdaptiveVolume::apply_(bool immediate) {
  const float a_before = this->core_.a;
  this->core_.update_attenuation();
  if (this->core_.a != a_before) {
    ESP_LOGD(TAG, "Noise %.1f dBFS, S %.1f, C %.1f -> attenuation %.0f dB", this->core_.n_a, this->core_.s_use(),
             this->core_.c(), this->core_.a);
  }
  this->apply_master_();
  this->publish_reports_(immediate);
}

void AdaptiveVolume::apply_master_() {
  if (this->stack_ == nullptr || this->core_.m == this->last_master_m_)
    return;
  // Codec curve: 1 % = master_min_db, 100 % = 0 dB, linear in dB between.
  const float span = -this->master_min_db_;
  const float m = std::min(this->core_.m, span);
  const float master = m <= 0.0f ? 1.0f : (((span - m) * 99.0f / span) + 1.0f) / 100.0f;
  this->stack_->set_master_volume(master);
  this->last_master_m_ = this->core_.m;
  ESP_LOGD(TAG, "Codec attenuation %.0f dB (master %.3f)", this->core_.m, master);
}

void AdaptiveVolume::publish_reports_(bool force) {
#ifdef USE_SENSOR
  const uint32_t now = millis();
  if (!force && now - this->last_report_ms_ < REPORT_INTERVAL_MS)
    return;
  this->last_report_ms_ = now;
  if (this->attenuation_sensor_ != nullptr && this->core_.a != this->reported_attenuation_) {
    this->attenuation_sensor_->publish_state(this->core_.a);
    this->reported_attenuation_ = this->core_.a;
  }
  if (this->current_volume_sensor_ != nullptr && this->media_player_ != nullptr) {
    const float vol = std::round(this->core_.effective_volume(this->media_player_->volume) * 100.0f);
    if (vol != this->reported_volume_) {
      this->current_volume_sensor_->publish_state(vol);
      this->reported_volume_ = vol;
    }
  }
  if (this->room_noise_sensor_ != nullptr && !std::isnan(this->core_.n_a)) {
    const float noise = std::round(this->core_.n_a);
    if (noise != this->reported_noise_) {
      this->room_noise_sensor_->publish_state(noise);
      this->reported_noise_ = noise;
    }
  }
#endif
}

}  // namespace adaptive_volume
}  // namespace esphome

#endif  // USE_ESP32
