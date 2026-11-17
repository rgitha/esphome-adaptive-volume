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
// 0.5 s block averages (power) -> median of the last 16 blocks (8 s)
// -> exponential smoothing (6 s rising, ~1.4 s falling).
// The median counts sound that fills most of the last 8 s (steady noise, continuous
// talk or TV) and ignores sound that does not (intermittent talk, clatter), with no
// separate hold or release. The first WARMUP_BLOCKS blocks after boot are skipped:
// the start-up transient (AFE, codec) made the first block read ~15 dB high, and the
// estimate then took ~5 s to fall from it.
struct NoiseEstimator {
  static constexpr int BLOCK_SAMPLES = 2;
  static constexpr int N_BLOCKS = 16;
  static constexpr int WARMUP_BLOCKS = 2;  // 1 s
  int warmup = WARMUP_BLOCKS;
  float blocks[N_BLOCKS] = {};
  int n_blocks = 0, head = 0;
  double block_power = 0.0;
  int block_count = 0;
  float est = NAN;

  void discard_partial() {
    block_power = 0.0;
    block_count = 0;
  }
  // Feed one 250 ms level; returns true when the estimate was updated (every 0.5 s).
  bool feed(float level) {
    const float RISE = 0.0800f;  // 1 - exp(-0.5 s / 6 s)
    const float FALL = 0.3f;     // ~1.4 s time constant
    block_power += std::pow(10.0, level / 10.0);
    if (++block_count < BLOCK_SAMPLES)
      return false;
    if (warmup > 0) {
      warmup--;
      discard_partial();
      return false;
    }
    blocks[head] = static_cast<float>(10.0 * std::log10(block_power / BLOCK_SAMPLES + 1e-30));
    head = (head + 1) % N_BLOCKS;
    if (n_blocks < N_BLOCKS)
      n_blocks++;
    discard_partial();
    // median of the blocks held (insertion sort, at most 16)
    float v[N_BLOCKS];
    for (int i = 0; i < n_blocks; i++) {
      const float x = blocks[i];
      int k = i - 1;
      while (k >= 0 && v[k] > x) {
        v[k + 1] = v[k];
        k--;
      }
      v[k + 1] = x;
    }
    const float target = (n_blocks % 2) ? v[n_blocks / 2] : 0.5f * (v[n_blocks / 2 - 1] + v[n_blocks / 2]);
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
// Path gain from the cross-spectrum (0.4.1): H(f) = Sxy(f) / Sxx(f) per FFT bin over a
// reply (Welch: 512-point Hann windows, 50 % overlap, windows with reference speech
// only), then the mean of |H|^2 over 200 Hz - 3 kHz with a fixed nominal weighting W(f)
// (long-term speech spectrum shape), renormalised over the bins the sound excited
// (within 40 dB of the strongest). Each bin is estimated on its own, so what a sound
// does not excite is left out instead of guessed, and the fixed weighting makes the
// result independent of the sound's spectrum. Talking and room noise do not correlate
// with the reference: they add scatter, no bias. Memory is supplied by the caller
// (MEM_FLOATS floats, PSRAM on the device).
struct SpectralPath {
  static constexpr int N = 512, HOP = N / 2, BINS = N / 2 + 1, LOG2N = 9;
  static constexpr size_t MEM_FLOATS = 2 * N /*ring x,y*/ + N /*speech flags*/ + N /*window*/ + N /*twiddles*/ +
                                       2 * N /*fft re,im*/ + 4 * BINS /*sxx syy sxy_re sxy_im*/;
  float *rx{nullptr}, *ry{nullptr}, *flag{nullptr}, *win{nullptr}, *tw{nullptr}, *fre{nullptr}, *fim{nullptr};
  float *sxx{nullptr}, *syy{nullptr}, *sre{nullptr}, *sim{nullptr};
  int pos{0}, since{0}, windows{0};
  float speech_n{0};

  bool attach(float *mem) {
    if (mem == nullptr)
      return false;
    float *p = mem;
    rx = p; p += N;
    ry = p; p += N;
    flag = p; p += N;
    win = p; p += N;
    tw = p; p += N;  // cos (first half), sin (second half) for k < N/2
    fre = p; p += N;
    fim = p; p += N;
    sxx = p; p += BINS;
    syy = p; p += BINS;
    sre = p; p += BINS;
    sim = p; p += BINS;
    const float pi = 3.14159265358979f;
    for (int i = 0; i < N; i++) {
      win[i] = 0.5f - 0.5f * std::cos(2.0f * pi * i / N);
      rx[i] = ry[i] = flag[i] = 0.0f;
    }
    for (int k = 0; k < N / 2; k++) {
      tw[k] = std::cos(2.0f * pi * k / N);
      tw[N / 2 + k] = -std::sin(2.0f * pi * k / N);
    }
    pos = since = 0;
    speech_n = 0;
    reset();
    return true;
  }
  bool ready() const { return rx != nullptr; }
  // a new reply: clear the spectra and the window history
  void reset() {
    if (!ready())
      return;
    for (int k = 0; k < BINS; k++)
      sxx[k] = syy[k] = sre[k] = sim[k] = 0.0f;
    for (int i = 0; i < N; i++)
      flag[i] = 0.0f;
    speech_n = 0;
    since = 0;
    windows = 0;
  }
  // x: reference, y: mic (A-weighted), same samples. speech: the reference carries
  // sound in this block. A window counts when at least 3/4 of it is speech.
  void push(const float *x, const float *y, uint32_t len, bool speech) {
    if (!ready())
      return;
    const float f = speech ? 1.0f : 0.0f;
    for (uint32_t i = 0; i < len; i++) {
      speech_n += f - flag[pos];
      rx[pos] = x[i];
      ry[pos] = y[i];
      flag[pos] = f;
      pos = (pos + 1) % N;
      if (++since >= HOP) {
        since = 0;
        if (speech_n >= 0.75f * N)
          accumulate_();
      }
    }
  }
  struct Result {
    float p_db{NAN};      // weighted path gain (dB)
    float coverage{0};    // share of the weighting in excited bins
    float coherence{0};   // weighted mean coherence (0..1)
    int windows{0};
  };
  Result result(float sample_rate) const {
    Result r;
    r.windows = windows;
    if (!ready() || windows == 0)
      return r;
    float smax = 0.0f;
    for (int k = 0; k < BINS; k++) {
      const float fk = k * sample_rate / N;
      if (fk >= 200.0f && fk <= 3000.0f)
        smax = std::max(smax, sxx[k]);
    }
    if (smax <= 0.0f)
      return r;
    double wall = 0, wok = 0, acc = 0, coh = 0;
    for (int k = 0; k < BINS; k++) {
      const float fk = k * sample_rate / N;
      if (fk < 200.0f || fk > 3000.0f)
        continue;
      const double w = weight(fk);
      wall += w;
      if (sxx[k] < smax * 1e-4f || syy[k] <= 0.0f)
        continue;
      const double cross = static_cast<double>(sre[k]) * sre[k] + static_cast<double>(sim[k]) * sim[k];
      wok += w;
      acc += w * cross / (static_cast<double>(sxx[k]) * sxx[k]);
      coh += w * cross / (static_cast<double>(sxx[k]) * syy[k]);
    }
    if (wok <= 0.0)
      return r;
    r.coverage = static_cast<float>(wok / wall);
    r.coherence = static_cast<float>(coh / wok);
    r.p_db = static_cast<float>(10.0 * std::log10(acc / wok + 1e-30));
    return r;
  }
  // Nominal weighting: long-term speech spectrum shape (power), band 200 Hz - 3 kHz
  static double weight(double f) {
    const double lp = 1.0 / (1.0 + (f / 500.0) * (f / 500.0));
    const double hp = (f / 200.0) * (f / 200.0) / (1.0 + (f / 200.0) * (f / 200.0));
    return lp * hp;
  }

 private:
  void accumulate_() {
    // pack x (real) and y (imaginary) into one complex FFT, oldest sample first
    for (int i = 0; i < N; i++) {
      const int j = (pos + i) % N;
      int r = 0;  // bit-reversed index
      for (int b = 0; b < LOG2N; b++)
        r |= ((i >> b) & 1) << (LOG2N - 1 - b);
      fre[r] = win[i] * rx[j];
      fim[r] = win[i] * ry[j];
    }
    for (int len = 2; len <= N; len <<= 1) {
      const int half = len / 2, step = N / len;
      for (int s = 0; s < N; s += len)
        for (int k = 0; k < half; k++) {
          const float wr = tw[k * step], wi = tw[N / 2 + k * step];
          const int a = s + k, b = a + half;
          const float tr = fre[b] * wr - fim[b] * wi, ti = fre[b] * wi + fim[b] * wr;
          fre[b] = fre[a] - tr;
          fim[b] = fim[a] - ti;
          fre[a] += tr;
          fim[a] += ti;
        }
    }
    for (int k = 0; k < BINS; k++) {
      const int m = (N - k) % N;
      // X = (Z[k] + conj Z[N-k]) / 2, Y = (Z[k] - conj Z[N-k]) / 2j
      const float xr = 0.5f * (fre[k] + fre[m]), xi = 0.5f * (fim[k] - fim[m]);
      const float yr = 0.5f * (fim[k] + fim[m]), yi = -0.5f * (fre[k] - fre[m]);
      sxx[k] += xr * xr + xi * xi;
      syy[k] += yr * yr + yi * yi;
      // conj(X) * Y
      sre[k] += xr * yr + xi * yi;
      sim[k] += xr * yi - xi * yr;
    }
    windows++;
  }
};

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

// Echo reduction while the device plays (diagnostic). Over one playback
// session (media player playing or announcing): the A-weighted level of mic 1
// before echo cancellation and of the processed output, each with its room
// level subtracted. Echo reduction = input echo - output residual (dB). Valid
// when the input is at least ECHO_DB above the room (the echo dominates);
// when the output is within 1 dB of its room level the result is a lower bound.
// Talking during playback adds to the output and lowers the result.
struct EchoResult {
  bool valid{false};       // echo reduction measured
  bool lower_bound{false}; // output residual at the room level: at least this much
  const char *reason{""};
  float seconds{0}, in_l{NAN}, out_l{NAN}, room_in{NAN}, room_out{NAN}, erle{NAN};
  float peak_dbfs{NAN};    // largest mic sample (all mics, before processing)
};

struct EchoMeter {
  static constexpr float MIN_SECONDS = 1.5f;
  static constexpr float ECHO_DB = 15.0f;
  bool active{false};
  double in_sum{0}, out_sum{0};
  uint32_t in_n{0}, out_n{0};
  uint16_t peak{0};

  void start() {
    active = true;
    in_sum = out_sum = 0;
    in_n = out_n = 0;
    peak = 0;
  }
  void add_input(double sum_sq, uint32_t n, uint16_t pk) {
    in_sum += sum_sq;
    in_n += n;
    peak = std::max(peak, pk);
  }
  void add_output(double sum_sq, uint32_t n) {
    out_sum += sum_sq;
    out_n += n;
  }
  EchoResult finish(float room_in_db, float room_out_db, float rate) {
    active = false;
    EchoResult r;
    r.seconds = in_n / rate;
    if (r.seconds < MIN_SECONDS || out_n == 0) {
      r.reason = "too short";
      return r;
    }
    r.peak_dbfs = peak > 0 ? 20.0f * std::log10(peak / 32768.0f) : -96.0f;
    r.in_l = static_cast<float>(10.0 * std::log10(in_sum / in_n + 1e-12));
    r.out_l = static_cast<float>(10.0 * std::log10(out_sum / out_n + 1e-12));
    r.room_in = room_in_db;
    r.room_out = room_out_db;
    if (std::isnan(room_in_db) || std::isnan(room_out_db)) {
      r.reason = "room reference not ready";
      return r;
    }
    const double p_room_in = std::pow(10.0, room_in_db / 10.0);
    const double p_room_out = std::pow(10.0, room_out_db / 10.0);
    const double p_in = in_sum / in_n, p_out = out_sum / out_n;
    if (p_in < std::pow(10.0, ECHO_DB / 10.0) * p_room_in) {
      r.reason = "playback too quiet at the mic";
      return r;
    }
    // Output residual above its room level; at most 1 dB above the room the
    // residual is not measurable: use 1 dB above as a floor (lower bound).
    double resid = p_out - p_room_out;
    const double floor = (std::pow(10.0, 0.1) - 1.0) * p_room_out;
    if (resid < floor) {
      resid = floor;
      r.lower_bound = true;
    }
    r.erle = static_cast<float>(10.0 * std::log10(p_in - p_room_in) - 10.0 * std::log10(resid));
    r.valid = true;
    return r;
  }
};

struct CoreConfig {
  float sample_rate{16000.0f};      // rate of the blocks fed (processor input / output)
  // Path mode (0.4.0): S = P + analog_gain_db, P = reference -> mic path gain from
  // the cross-correlation. How loud the played sound is, is not measured: C (loudness
  // above the room) assumes the speaker output OUTPUT_LEVEL_DB below full scale, so S
  // does not follow individual sounds; C is set by ear as a whole. Off: S from S_k
  // (as before).
  bool use_path{false};
  float analog_gain_db{0.0f};       // mic/reference analogue gain (N follows it, P does not)
  float path_gain_nominal{-9.0f};   // P before the first measurement
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
  float s_use() const {
    if (cfg.use_path)
      return s_from_p(std::isnan(p) ? cfg.path_gain_nominal : p);
    return std::isnan(s) ? cfg.speaker_level_nominal : s;
  }
  float s_from_p(float path) const { return path + cfg.analog_gain_db; }

  // ---- path mode: P history (median of the last P_HISTORY accepted replies) ----
  static constexpr int P_HISTORY = 5;
  float p{NAN};  // path gain reference -> mic (dB), NAN = never measured
  // A stored P counts as 3 entries, so one odd reply after boot cannot move it.
  static constexpr int P_RESTORE_WEIGHT = 3;
  void restore_p(float value) {
    p = value;
    if (!std::isnan(value)) {
      for (int i = 0; i < P_RESTORE_WEIGHT; i++)
        p_hist_[i] = value;
      p_n_ = P_RESTORE_WEIGHT;
      p_head_ = P_RESTORE_WEIGHT % P_HISTORY;
    }
  }
  void push_p(float pk) {
    p_hist_[p_head_] = pk;
    p_head_ = (p_head_ + 1) % P_HISTORY;
    if (p_n_ < P_HISTORY)
      p_n_++;
    float v[P_HISTORY];
    for (int i = 0; i < p_n_; i++) {  // insertion sort (tiny)
      float x = p_hist_[i];
      int j = i - 1;
      while (j >= 0 && v[j] > x) {
        v[j + 1] = v[j];
        j--;
      }
      v[j + 1] = x;
    }
    p = (p_n_ % 2) ? v[p_n_ / 2] : 0.5f * (v[p_n_ / 2 - 1] + v[p_n_ / 2]);
  }
  int p_count() const { return p_n_; }
  // Path mode: speaker output level assumed by C, in dB below full scale at the
  // speaker reference (0 dB analogue gain, full volume, no attenuation). It applies to
  // whatever is played (speech, music, chimes): typical material at full volume sits
  // about 46 dB below the reference's full scale (measured: a typical HA voice -44.7 to
  // -46.7). S = P + gain is a path gain, not a level, so this output level is added to
  // C here; C stays loudness above the room.
  static constexpr float OUTPUT_LEVEL_DB = 46.0f;
  float c() const { return cfg.loudness_constant + offset + (cfg.use_path ? OUTPUT_LEVEL_DB : 0.0f); }
  bool measuring() const { return measuring_; }

  // Restore a stored S (after boot): it counts as one entry of the history.
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
  // Median of the history (dB): one bad reply, too high (sound over it) or too
  // low (room quieter than before the wake word), does not move S. With fewer
  // than 3 entries: the one entry, or the mean of two (median of two).
  float hist_median_() const {
    if (hist_n_ == 1)
      return hist_[0];
    if (hist_n_ == 2)
      return 0.5f * (hist_[0] + hist_[1]);
    const float a = hist_[0], b = hist_[1], c = hist_[2];
    return std::max(std::min(a, b), std::min(std::max(a, b), c));
  }

  CalResult evaluate_() {
    CalResult r;
    r.v0 = v0_;
    r.m0 = m0_;
    // S_k's own estimate (in path mode S is not a level, and S_k is a diagnostic)
    r.s_old = cfg.use_path ? (std::isnan(s) ? cfg.speaker_level_nominal : s) : s_use();
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
      s = hist_median_();
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
  float p_hist_[P_HISTORY] = {};
  int p_n_{0}, p_head_{0};
};

}  // namespace adaptive_volume
}  // namespace esphome
