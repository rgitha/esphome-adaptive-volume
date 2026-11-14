// Host test for av_core.h: g++ -std=c++17 -O2 -I../components/adaptive_volume test_core.cpp && ./a.out
#include <cassert>
#include <cstdio>
#include <random>

#include "av_core.h"

using namespace esphome::adaptive_volume;

static int failures = 0;
#define CHECK(cond, ...)                 \
  do {                                   \
    if (!(cond)) {                       \
      printf("FAIL %s:%d: ", __FILE__, __LINE__); \
      printf(__VA_ARGS__);               \
      printf("\n");                      \
      failures++;                        \
    }                                    \
  } while (0)

struct Sim {
  Core core;
  std::mt19937 rng{1};
  double s_true = -35.0;   // speaker at mics, referred to V = 0, M = 0
  double room_in = -80.0;  // room at the input (A-weighted, like N_A)
  double room_aw = -80.0;  // room A-weighted
  float volume = 1.0f;
  bool amp = true;
  uint64_t t_us = 0;
  static constexpr uint32_t FRAME = 512;  // 32 ms at 16 kHz

  Sim() {
    core.cfg.media_volume_min = 0.1f;
    core.cfg.media_volume_max = 1.0f;
    core.cfg.speaker_level_nominal = -35.0f;
    core.enabled = true;
  }
  static double pw(double db) { return std::pow(10.0, db / 10.0); }

  // one 32 ms frame of both points
  void frame(bool idle, bool sound, double speech_db_jitter) {
    std::normal_distribution<double> j(0.0, 1.0);
    double p_in = pw(room_in + 0.5 * j(rng));
    if (sound && amp) {
      const double g = core.v_db(volume) - core.m;
      // lognormal speech fluctuation with the same mean power as s_true
      const double bias = speech_db_jitter * speech_db_jitter * std::log(10.0) / 20.0;
      p_in += pw(s_true + g - bias + speech_db_jitter * j(rng));
    }
    t_us += 32000;
    core.on_input(p_in * FRAME, FRAME, t_us, idle);
    if (core.on_aweighted(pw(room_aw + 0.5 * j(rng)) * FRAME, FRAME, idle))
      core.update_attenuation();
  }
  void idle(double seconds) {
    for (int i = 0; i < seconds / 0.032; i++) {
      frame(true, false, 0);
      core.reply_update(false, t_us, volume, !amp);
    }
  }
  // Reply: ANNOUNCING at t0, sound reaches the mics 300 ms later
  CalResult reply(double seconds, double jitter = 6.0) {
    const uint64_t t0 = t_us;
    core.reply_update(true, t_us, volume, !amp);
    const int frames = static_cast<int>((seconds + 0.3) / 0.032);
    for (int i = 0; i < frames; i++) {
      const bool sound = (t_us - t0) >= 300000;
      frame(false, sound, jitter);
      core.reply_update(true, t_us, volume, !amp);
    }
    CalResult r = core.reply_update(false, t_us, volume, !amp);
    core.update_attenuation();
    return r;
  }
};

int main() {
  // 1. Fresh device, slider 100 %, quiet room: first reply plays at nominal S,
  //    gets measured and S lands near the truth.
  {
    Sim sim;
    sim.idle(15);
    CHECK(sim.core.ready, "idle reference ready after 15 s");
    CHECK(std::fabs(sim.core.n_a - (-80.0f)) < 1.5f, "N_A %.1f", sim.core.n_a);
    CHECK(std::fabs(sim.core.a - 25.0f) < 1.0f, "initial attenuation %.1f (nominal -35, N -80, C 20)", sim.core.a);
    auto r = sim.reply(1.0);
    CHECK(r.kind == CalResult::DETECTED, "1 s reply detected (kind %d, %s)", r.kind, r.reason);
    CHECK(std::fabs(sim.core.s - (-35.0f)) < 2.0f, "S %.1f after 1 s reply", sim.core.s);
    printf("1: S after one 1 s reply = %.2f (true -35)\n", sim.core.s);
  }
  // 2. Stuck case: speaker 30 dB weaker than S -> recovers within a few replies.
  {
    Sim sim;
    sim.idle(15);
    sim.s_true = -65.0;
    int replies = 0;
    while (replies < 6) {
      sim.idle(1);
      auto r = sim.reply(1.5);
      replies++;
      if (r.kind == CalResult::DETECTED)
        break;
    }
    CHECK(std::fabs(sim.core.s - (-65.0f)) < 2.5f, "S %.1f after recovery", sim.core.s);
    printf("2: recovered to S = %.2f in %d replies (true -65)\n", sim.core.s, replies);
  }
  // 3. Low slider: undetected reply, S unchanged.
  {
    Sim sim;
    sim.idle(15);
    sim.reply(2.0);
    const float s_before = sim.core.s;
    sim.volume = 0.0f;
    sim.idle(1);
    auto r = sim.reply(2.0);
    CHECK(r.kind == CalResult::WITHIN_BOUND || r.kind == CalResult::SKIPPED, "low slider kind %d", r.kind);
    CHECK(sim.core.s == s_before, "S unchanged at low slider");
  }
  // 4. Amplifier off: not counted.
  {
    Sim sim;
    sim.idle(15);
    sim.amp = false;
    auto r = sim.reply(2.0);
    CHECK(r.kind == CalResult::SKIPPED, "amp off skipped");
  }
  // 5. Short reply: 0.2 s of clear speech (< 0.3 s active): not counted, S kept.
  {
    Sim sim;
    sim.idle(15);
    auto r = sim.reply(0.2);
    CHECK(r.kind == CalResult::SKIPPED, "0.2 s of speech skipped (kind %d)", r.kind);
  }
  // 6. Offset +6 -> 6 dB less attenuation; announcement offset reduces M only.
  {
    Sim sim;
    sim.idle(15);
    const float a0 = sim.core.a;
    sim.core.offset = 6.0f;
    sim.core.update_attenuation();
    CHECK(std::fabs((a0 - sim.core.a) - 6.0f) < 0.5f, "offset +6: %.1f -> %.1f", a0, sim.core.a);
    sim.core.boost = 6.0f;
    sim.core.update_attenuation();
    CHECK(std::fabs(sim.core.m - (sim.core.a - 6.0f)) < 0.01f, "M = A - B");
  }
  // 7. Scatter: 20 short replies, 6 dB independent jitter per 32 ms frame (harsh); S within 3 dB.
  {
    Sim sim;
    sim.idle(15);
    float worst = 0;
    for (int i = 0; i < 20; i++) {
      sim.idle(1);
      sim.reply(1.2);
      worst = std::max(worst, std::fabs(sim.core.s - (-35.0f)));
    }
    CHECK(worst < 3.0f, "worst S error %.2f dB", worst);
    printf("7: worst S error over 20 replies of 1.2 s = %.2f dB\n", worst);
  }
  // 8. Disabled: attenuation 0.
  {
    Sim sim;
    sim.idle(15);
    sim.core.enabled = false;
    sim.core.update_attenuation();
    CHECK(sim.core.a == 0.0f && sim.core.m == 0.0f, "disabled -> 0");
  }
  // 9. Median of 3: one reply 10 dB too high, or 6 dB too low, does not move S;
  //    a real change is followed after two replies.
  {
    Sim sim;
    sim.idle(15);
    for (int i = 0; i < 3; i++) {
      sim.idle(1);
      sim.reply(2.0, 1.0);
    }
    const float s0 = sim.core.s;
    sim.s_true = -25.0;  // someone talking over this reply
    sim.idle(1);
    sim.reply(2.0, 1.0);
    CHECK(std::fabs(sim.core.s - s0) < 1.0f, "high outlier moved S %.1f -> %.1f", s0, sim.core.s);
    sim.s_true = -41.0;  // room quieter during this reply
    sim.idle(1);
    sim.reply(2.0, 1.0);
    CHECK(std::fabs(sim.core.s - s0) < 1.0f, "low outlier moved S %.1f -> %.1f", s0, sim.core.s);
    sim.s_true = -30.0;  // real change (louder speaker)
    for (int i = 0; i < 2; i++) {
      sim.idle(1);
      sim.reply(2.0, 1.0);
    }
    CHECK(std::fabs(sim.core.s - (-30.0f)) < 1.5f, "real change followed: S %.1f", sim.core.s);
    printf("9: outliers rejected (S %.2f), real change followed after 2 replies (S %.2f)\n", s0, sim.core.s);
  }
  // 10. Boot transient: the first 0.5 s reads 15 dB high; it is skipped, so the
  //     first estimate is the room (-77), not the transient.
  {
    NoiseEstimator e;
    float first = NAN;
    for (int i = 0; i < 2; i++)
      e.feed(-62.0f);  // first block: start-up transient
    for (int i = 0; i < 4; i++)
      if (e.feed(-77.0f) && std::isnan(first))
        first = e.est;
    CHECK(std::fabs(first - (-77.0f)) < 0.5f, "first estimate %.1f (room -77)", first);
    printf("10: first estimate after a loud boot block = %.1f (room -77)\n", first);
  }
  printf(failures == 0 ? "ALL PASSED\n" : "%d FAILURES\n", failures);
  return failures == 0 ? 0 : 1;
}
