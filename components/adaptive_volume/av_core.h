#pragma once
// Adaptive volume core: platform-independent logic (no ESPHome includes), so it
// can be unit-tested on a PC. The ESPHome component feeds it audio block powers
// and state, and applies its attenuation output to the codec.
//
// Symbols follow the design document (flowchart rev 3):
//   P_in       input power (processor input, mic 1, before echo cancellation)
//   P_room,pre robust estimate of P_in over idle time (room before the reply)
//   N_A        robust estimate of the A-weighted output level (room noise)
//   S          speaker level at the mics, referred to V = 0 dB, M = 0 dB
//   C          loudness constant + user offset
//   A          applied attenuation, M = max(0, A - B) to the codec

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>

namespace esphome {
namespace adaptive_volume {

// Robust noise estimator on 250 ms levels (dBFS):
// 0.5 s block peaks -> min(3rd largest of the last 16, max of the last 9)
// -> exponential smoothing (6 s rising, ~1.4 s falling).
struct NoiseEstimator {
  static constexpr int BLOCK_SAMPLES = 2;
  static constexpr int N_BLOCKS = 16;
  static constexpr int RANK = 3;
  static constexpr int HOLD_BLOCKS = 9;
  float blocks[N_BLOCKS] = {};
  int n_blocks = 0, head = 0;
  float block_peak = -200.0f;
  int block_count = 0;
  float est = NAN;

  void discard_partial() {
    block_peak = -200.0f;
    block_count = 0;
  }
  // Feed one 250 ms level; returns true when the estimate was updated (every 0.5 s).
  bool feed(float level) {
    const float RISE = 0.0800f;  // 1 - exp(-0.5 s / 6 s)
    const float FALL = 0.3f;     // ~1.4 s time constant
    block_peak = std::max(block_peak, level);
    if (++block_count < BLOCK_SAMPLES)
      return false;
    blocks[head] = block_peak;
    head = (head + 1) % N_BLOCKS;
    if (n_blocks < N_BLOCKS)
      n_blocks++;
    discard_partial();
    float sorted[N_BLOCKS] = {};
    for (int i = 0; i < n_blocks; i++)
      sorted[i] = blocks[(head - 1 - i + N_BLOCKS) % N_BLOCKS];
    // partial selection sort, descending, up to RANK
    const int k = std::min(RANK, n_blocks);
    for (int i = 0; i < k; i++) {
      int m = i;
      for (int j = i + 1; j < n_blocks; j++)
        if (sorted[j] > sorted[m])
          m = j;
      std::swap(sorted[i], sorted[m]);
    }
    const float a = sorted[k - 1];
    float b = -200.0f;
    for (int i = 0; i < std::min(HOLD_BLOCKS, n_blocks); i++)
      b = std::max(b, blocks[(head - 1 - i + N_BLOCKS) % N_BLOCKS]);
    const float target = std::min(a, b);
    if (std::isnan(est))
      est = target;
    else
      est += (target > est ? RISE : FALL) * (target - est);
    return true;
  }
};

// Approximate A-weighting: 2nd-order Butterworth high-pass at 200 Hz plus a
// 1st-order high-pass at 500 Hz (within ~1-2 dB of A-weighting from 200 Hz up,
// stronger below, which also removes DC, hum and rumble).
struct AWeighting {
  float b0{0}, b1{0}, b2{0}, a1{0}, a2{0}, rc_a{0};
  float x1{0}, x2{0}, y1{0}, y2{0}, hp_x1{0}, hp_y1{0};
  uint32_t rate{0};
  void setup(uint32_t sample_rate) {
    const float fs = static_cast<float>(sample_rate);
    const float pi = 3.14159265f;
    const float w0 = 2.0f * pi * 200.0f / fs;
    const float c = std::cos(w0), al = std::sin(w0) / (2.0f * 0.7071f);
    const float a0 = 1.0f + al;
    b0 = (1.0f + c) / 2.0f / a0;
    b1 = -(1.0f + c) / a0;
    b2 = b0;
    a1 = -2.0f * c / a0;
    a2 = (1.0f - al) / a0;
    const float rc = 1.0f / (2.0f * pi * 500.0f), dt = 1.0f / fs;
    rc_a = rc / (rc + dt);
    x1 = x2 = y1 = y2 = hp_x1 = hp_y1 = 0.0f;
    rate = sample_rate;
  }
  float process(float in) {
    const float y = b0 * in + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1;
    x1 = in;
    y2 = y1;
    y1 = y;
    const float hp = rc_a * (hp_y1 + y - hp_x1);
    hp_x1 = y;
    hp_y1 = hp;
    return hp;
  }
};

struct CoreConfig {
  float sample_rate{16000.0f};      // rate of the blocks fed (processor input / output)
  float speaker_level_nominal{-30.0f};
  float loudness_constant{20.0f};
  float max_attenuation{30.0f};
  float hysteresis{2.0f};
  float software_volume_min_db{-49.0f};
  float media_volume_min{0.1f};
  float media_volume_max{1.0f};
  uint32_t reply_start_delay_us{0};
  float reply_min_duration_s{0.4f};  // shortest reply (all frames)
  float active_min_duration_s{0.3f}; // speech (active frames) needed for a measurement
};

struct CalResult {
  enum Kind { NONE, SKIPPED, DETECTED, LOWERED, WITHIN_BOUND } kind{NONE};
  const char *reason{""};
  float in_l{NAN}, room_l{NAN}, s_k{NAN}, s_old{NAN}, s_new{NAN}, bound{NAN}, v0{0}, m0{0}, seconds{0}, active_s{0};
  int hist_n{0};
  bool marginal{false};
};

class Core {
 public:
  static constexpr float ACTIVE = 2.0f;          // a frame is speech when P_in >= 2 P_room (3 dB above)
  static constexpr float SHORT_CLEAR = 3.0f;     // speaker power 3x room: input 6 dB above room
  static constexpr float CLEAR_DB = 9.0f;        // expected speech below this above the room: a measurement reads high
  static constexpr float BOUND_MARGIN_DB = 3.0f;  // S set this far below the bound
  static constexpr float FLOOR_BELOW_NOMINAL = 30.0f;
  static constexpr int READY_TICKS = 40;          // 10 s of idle 250 ms ticks
  static constexpr int HISTORY = 3;

  CoreConfig cfg;

  // ---- state ----
  float s{NAN};         // measured S (NAN = never measured)
  float n_a{NAN};       // room noise, A-weighted (dBFS)
  float room_pre{NAN};  // P_room,pre level (dBFS)
  bool ready{false};
  float a{0.0f};          // applied attenuation A (dB)
  float m{0.0f};          // codec attenuation M = max(0, A - B)
  bool enabled{false};
  float offset{0.0f};     // user offset (dB, + = louder)
  float boost{0.0f};      // announcement offset B (dB)

  // ---- helpers ----
  float v_db(float v) const {
    const float x = cfg.media_volume_min + (cfg.media_volume_max - cfg.media_volume_min) * v;
    return cfg.software_volume_min_db * (1.0f - x);
  }
  float s_use() const { return std::isnan(s) ? cfg.speaker_level_nominal : s; }
  float c() const { return cfg.loudness_constant + offset; }
  bool measuring() const { return measuring_; }

  // Restore a stored S (after boot): it counts as one entry of the average.
  void restore_s(float value) {
    s = value;
    if (!std::isnan(value)) {
      hist_[0] = value;
      hist_n_ = 1;
      hist_head_ = 1;
    }
  }

  // Input block (processor input, mic 1): sum of squared normalized samples.
  // During a reply every frame counts towards the total; frames at least 3 dB
  // above the room (speech) also count towards the active sum, room subtracted.
  void on_input(double sum_sq, uint32_t n, uint64_t ts_us, bool idle) {
    if (measuring_ && n > 0 && ts_us >= start_us_ + cfg.reply_start_delay_us) {
      r_sum_ += sum_sq;
      r_n_ += n;
      if (p_room0_ > 0.0 && sum_sq / n >= ACTIVE * p_room0_) {
        a_sum_ += sum_sq - p_room0_ * n;
        a_n_ += n;
      }
    }
    feed_tick_(in_tick_, in_est_, sum_sq, n, idle, [this](float est) {
      room_pre = est;
    }, [this]() {
      if (!ready && ++idle_ticks_ >= READY_TICKS)
        ready = true;
    });
  }

  // A-weighted output block; returns true when N_A was updated.
  bool on_aweighted(double sum_sq, uint32_t n, bool idle) {
    bool updated = false;
    feed_tick_(aw_tick_, aw_est_, sum_sq, n, idle, [this, &updated](float est) {
      n_a = est;
      updated = true;
    }, []() {});
    return updated;
  }

  // Reply state, called regularly. q = a wake-word reply is playing.
  // v = media volume (0..1), blocked = muted or amplifier off.
  // Returns a result when a reply has just ended.
  CalResult reply_update(bool q, uint64_t now_us, float v, bool blocked) {
    CalResult r;
    if (q) {
      if (!measuring_) {
        measuring_ = true;
        changed_ = false;
        start_us_ = now_us;
        r_sum_ = 0;
        r_n_ = 0;
        a_sum_ = 0;
        a_n_ = 0;
        p_room0_ = std::isnan(room_pre) ? 0.0 : std::pow(10.0, room_pre / 10.0);
        v0_ = v;
        m0_ = m;
      }
      if (std::fabs(v - v0_) > 0.001f || m != m0_ || blocked)
        changed_ = true;
      return r;
    }
    if (!measuring_)
      return r;
    measuring_ = false;
    return evaluate_();
  }

  // Attenuation from S, N_A, C. Returns true when M changed.
  bool update_attenuation() {
    float target = 0.0f;
    if (enabled && !std::isnan(n_a)) {
      target = s_use() - n_a - c();
      target = std::min(std::max(target, 0.0f), cfg.max_attenuation);
    }
    if (target < 0.5f)
      a = 0.0f;
    else if (std::fabs(target - a) >= cfg.hysteresis)
      a = std::round(target);
    const float new_m = std::max(0.0f, a - boost);
    if (new_m == m)
      return false;
    m = new_m;
    return true;
  }

  // Effective volume as a media-volume equivalent (0..1), for display.
  float effective_volume(float v) const {
    const float total_db = v_db(v) - a;
    const float x = 1.0f - total_db / cfg.software_volume_min_db;
    const float eff = (x - cfg.media_volume_min) / (cfg.media_volume_max - cfg.media_volume_min);
    return std::min(1.0f, std::max(0.0f, eff));
  }

 protected:
  struct Tick {
    double sum{0};
    uint32_t n{0};
  };

  template<typename OnEst, typename OnTick>
  void feed_tick_(Tick &t, NoiseEstimator &est, double sum_sq, uint32_t n, bool idle, OnEst on_est, OnTick on_tick) {
    if (!idle) {
      t = Tick{};
      est.discard_partial();
      return;
    }
    t.sum += sum_sq;
    t.n += n;
    const uint32_t tick_n = static_cast<uint32_t>(cfg.sample_rate / 4.0f);
    if (t.n < tick_n)
      return;
    const double ms = t.sum / t.n;
    t = Tick{};
    const float level = ms > 1e-13 ? static_cast<float>(10.0 * std::log10(ms)) : -130.0f;
    on_tick();
    if (est.feed(level))
      on_est(est.est);
  }

  void push_hist_(float v) {
    hist_[hist_head_] = v;
    hist_head_ = (hist_head_ + 1) % HISTORY;
    if (hist_n_ < HISTORY)
      hist_n_++;
  }
  float hist_mean_() const {
    double sum = 0;
    for (int i = 0; i < hist_n_; i++)
      sum += std::pow(10.0, hist_[i] / 10.0);
    return static_cast<float>(10.0 * std::log10(sum / hist_n_));
  }

  CalResult evaluate_() {
    CalResult r;
    r.v0 = v0_;
    r.m0 = m0_;
    r.s_old = s_use();
    r.seconds = r_n_ / cfg.sample_rate;
    r.kind = CalResult::SKIPPED;
    if (changed_) {
      r.reason = "volume changed, muted or amplifier off";
      return r;
    }
    if (r.seconds < cfg.reply_min_duration_s) {
      r.reason = "too short";
      return r;
    }
    if (!ready || p_room0_ <= 0.0) {
      r.reason = "room reference not ready";
      return r;
    }
    if (v0_ <= 0.0f) {
      r.reason = "volume 0";
      return r;
    }
    r.active_s = a_n_ / cfg.sample_rate;
    r.room_l = room_pre;
    const float vdb = v_db(v0_);
    if (r.active_s >= cfg.active_min_duration_s) {
      // Speech frames only: mean speaker power over them
      const double p_spk = a_sum_ / a_n_;
      r.in_l = static_cast<float>(10.0 * std::log10(p_spk + p_room0_));
      if (r.in_l > -6.0f) {
        r.reason = "input near clipping";
        return r;
      }
      r.s_k = static_cast<float>(10.0 * std::log10(p_spk)) - vdb + m0_;
      // Expected speech level above the room, from the S in use. Below 9 dB
      // only the louder syllables pass the gate, so S_k reads high: trust it
      // only to lower S. (A too-high S predicts too low a level, so it can
      // still be corrected; a too-low S predicts a clear level and is raised.)
      const float expected_db = r.s_old + vdb - m0_ - room_pre;
      if (expected_db < CLEAR_DB) {
        r.marginal = true;
        if (r.s_k >= r.s_old) {
          r.reason = "marginal level, would raise S";
          return r;
        }
        hist_n_ = 0;
        hist_head_ = 0;
      }
      r.kind = CalResult::DETECTED;
      push_hist_(r.s_k);
      r.hist_n = hist_n_;
      s = hist_mean_();
      r.s_new = s;
      return r;
    }
    // Short but clearly audible speech (mean of its frames >= 6 dB above the
    // room): too little to measure, and no reason to doubt S.
    if (a_n_ > 0 && a_sum_ / a_n_ >= SHORT_CLEAR * p_room0_) {
      r.reason = "too little speech";
      return r;
    }
    // Too little speech above the room: the speaker was at most at room level.
    r.in_l = static_cast<float>(10.0 * std::log10(r_sum_ / r_n_));
    r.bound = static_cast<float>(10.0 * std::log10(p_room0_)) - vdb + m0_;
    if (r.s_old > r.bound) {
      r.kind = CalResult::LOWERED;
      s = std::max(r.bound - BOUND_MARGIN_DB, cfg.speaker_level_nominal - FLOOR_BELOW_NOMINAL);
      hist_n_ = 0;
      hist_head_ = 0;
      r.s_new = s;
    } else {
      r.kind = CalResult::WITHIN_BOUND;
      r.s_new = r.s_old;
    }
    return r;
  }

  NoiseEstimator in_est_, aw_est_;
  Tick in_tick_, aw_tick_;
  int idle_ticks_{0};
  bool measuring_{false}, changed_{false};
  uint64_t start_us_{0};
  double r_sum_{0}, a_sum_{0}, p_room0_{0};
  uint32_t r_n_{0}, a_n_{0};
  float v0_{0}, m0_{0};
  float hist_[HISTORY] = {};
  int hist_n_{0}, hist_head_{0};
};

}  // namespace adaptive_volume
}  // namespace esphome
