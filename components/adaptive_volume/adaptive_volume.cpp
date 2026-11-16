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
static constexpr uint32_t P_PREF_HASH = 0x5A7E1D0Cu;  // path gain P (0.4.0)
// Path-mode acceptance of one reply
static constexpr float P_MIN_RHO = 0.5f;
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
  // mic 1, A-weighted, kept for the cross-correlation with the reference frame
  const bool keep = b.point == 0 && frame.frames <= XC_MAX;
  for (size_t i = 0; i < frame.frames; i++) {
    const float y = w->process(smp[i * stride] * (1.0f / 32768.0f));
    sum += static_cast<double>(y) * y;
    if (keep)
      this->xc_y_[i] = y;
  }
  if (b.point == 0)
    this->xc_y_n_ = keep ? static_cast<uint32_t>(frame.frames) : 0;
  b.sum_sq = static_cast<float>(sum);
  this->push_(b);
}

// Reference frame (speaker signal after volume and codec attenuation): cross-
// correlated with the mic frame of the same cycle while a reply is measured.
void AdaptiveVolume::on_reference_frame_(const esp_audio_stack::CaptureFrame &frame) {
  const int16_t *smp = static_cast<const int16_t *>(frame.data);
  const size_t stride = frame.stride;
  const uint32_t len = static_cast<uint32_t>(frame.frames);
  if (len > XC_MAX)
    return;
  if (frame.sample_rate != this->aw_ref_.rate && frame.sample_rate > 0)
    this->aw_ref_.setup(frame.sample_rate);
  double sum = 0.0;
  int32_t pk = 0;
  for (uint32_t i = 0; i < len; i++) {
    const int16_t s = smp[i * stride];
    pk = std::max<int32_t>(pk, std::abs(static_cast<int32_t>(s)));
    const float x = this->aw_ref_.process(s * (1.0f / 32768.0f));
    this->xc_x_[i] = x;
    sum += static_cast<double>(x) * x;
  }
  const float level = sum > 0.0 ? static_cast<float>(10.0 * std::log10(sum / len)) : -130.0f;
  const bool on = this->xc_on_.load(std::memory_order_relaxed);
  if (!on) {
    // idle floor of the reference: falls at once, rises slowly
    if (level < this->xc_ref_floor_db_)
      this->xc_ref_floor_db_ = level;
    else
      this->xc_ref_floor_db_ += 0.01f * (level - this->xc_ref_floor_db_);
  }
  const bool matched = this->xc_y_n_ == len;
  // speech in the reference: 20 dB above its idle floor (and not near silence)
  const bool speech = on && matched && level >= std::max(this->xc_ref_floor_db_ + 20.0f, -75.0f);
  LockGuard guard(this->xc_lock_);
  if (on)
    this->xc_ref_peak_ = static_cast<uint16_t>(std::max<int32_t>(this->xc_ref_peak_, std::min<int32_t>(pk, 65535)));
  if (matched)
    this->xc_.add(this->xc_x_, this->xc_y_, len, speech);
  else
    this->xc_.clear_history();
  this->xc_y_n_ = 0;
}

// Coherent speaker level, logged next to the regular measurement (diagnostic only:
// S is not changed by it).
bool AdaptiveVolume::log_coherent_(const CalResult &r, float *path_db) {
  XCorr::Result x;
  uint16_t peak;
  char profile[(XCorr::LAGS / 4 + 1) * 7 + 1];
  {
    LockGuard guard(this->xc_lock_);
    x = this->xc_.result();
    // Coarse lag profile (normalised correlation every 4th lag), to locate the peak
    size_t len = 0;
    profile[0] = '\0';
    const double norm = std::sqrt(this->xc_.rxx * this->xc_.ryy);
    for (int t = 0; t < XCorr::LAGS && norm > 0.0 && len + 7 < sizeof(profile); t += 4)
      len += snprintf(profile + len, sizeof(profile) - len, " %.2f", this->xc_.rxy[t] / norm);
    const float seconds = this->xc_.n / this->core_.cfg.sample_rate;
    peak = this->xc_ref_peak_;
    this->xc_.reset();
    this->xc_ref_peak_ = 0;
    if (std::isnan(x.gain_db) || seconds < this->core_.cfg.active_min_duration_s) {
      ESP_LOGD(TAG, "Path: too little reference speech (%.2f s), not counted", seconds);
      return false;
    }
  }
  const float peak_db = peak > 0 ? 20.0f * std::log10(peak / 32768.0f) : -96.0f;
  // Reply level at the reference, normalised like S_k (volume and attenuation removed,
  // at 0 dB analogue gain): shows how loud each voice plays
  const float reply_ref = x.ref_db - this->core_.v_db(r.v0) + r.m0 - this->core_.cfg.analog_gain_db;
  ESP_LOGD(TAG,
           "Path: gain %.1f dB at lag %d, rho %.2f (mic %.1f, coherent %.1f, ref %.1f dBFS, ref peak %.1f dBFS); "
           "reply level %.1f dBFS at the reference",
           x.gain_db, x.lag, x.rho, x.mic_db, x.coherent_db, x.ref_db, peak_db, reply_ref);
  ESP_LOGD(TAG, "Path: correlation at lags 0, 4 .. %d:%s", XCorr::LAGS - 1, profile);
  if (x.rho < P_MIN_RHO) {
    ESP_LOGD(TAG, "Path: rho below %.2f, not counted", P_MIN_RHO);
    return false;
  }
  if (x.lag <= 0 || x.lag >= XCorr::LAGS - 1) {
    ESP_LOGD(TAG, "Path: peak at the edge of the lag window, not counted");
    return false;
  }
  *path_db = x.gain_db;
#ifdef USE_SENSOR
  if (this->path_gain_reply_sensor_ != nullptr)
    this->path_gain_reply_sensor_->publish_state(x.gain_db);
#endif
  return true;
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
  const bool xc_measuring = this->core_.measuring();
  if (xc_measuring && !this->xc_on_.load(std::memory_order_relaxed)) {
    LockGuard guard(this->xc_lock_);  // a new reply: start from zero
    this->xc_.reset();
    this->xc_ref_peak_ = 0;
  }
  this->xc_on_.store(xc_measuring, std::memory_order_relaxed);
  if (r.kind != CalResult::NONE) {
    this->log_result_(r);
    float path_db = NAN;
    const bool path_ok = this->log_coherent_(r, &path_db);
    if (this->core_.cfg.use_path) {
#ifdef USE_SENSOR
      if (r.kind == CalResult::DETECTED && this->speaker_level_reply_sensor_ != nullptr)
        this->speaker_level_reply_sensor_->publish_state(r.s_k);
#endif
      if (path_ok) {
        const float s_old = this->core_.s_use();
        this->core_.push_p(path_db);
        ESP_LOGD(TAG, "Path: P %.1f dB (median of %d) -> S %.1f (was %.1f)", this->core_.p, this->core_.p_count(),
                 this->core_.s_use(), s_old);
        float p = this->core_.p;
        // Flash only when P moved by 0.5 dB or more since the last save
        if (std::isnan(this->saved_p_) || std::fabs(p - this->saved_p_) >= 0.5f) {
          this->p_pref_.save(&p);
          this->saved_p_ = p;
        }
#ifdef USE_SENSOR
        if (this->path_gain_sensor_ != nullptr)
          this->path_gain_sensor_->publish_state(this->core_.p);
#endif
        this->apply_(false);
      }
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
