// Host test for the room-noise estimator: g++ -std=c++17 -O2 -I../components/adaptive_volume test_noise.cpp && ./a.out
// 250 ms levels; room -70 dBFS, talk -45 dBFS (syllables vary +-4 dB).
#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>
#include "av_core.h"
using namespace esphome::adaptive_volume;

static std::mt19937 rng(3);
static float talk() { return -45.0f + std::uniform_real_distribution<float>(-4, 4)(rng); }
static float room() { return -70.0f + std::uniform_real_distribution<float>(-1, 1)(rng); }

// feeds `seconds` of levels from f(); returns the estimate at the end
template <typename F> static float run(NoiseEstimator &e, float seconds, F f) {
  for (int i = 0; i < int(seconds * 4); i++) e.feed(f(i));
  return e.est;
}

int main() {
  setvbuf(stdout, nullptr, _IONBF, 0);
  // 1. Steady room: N follows it
  {
    NoiseEstimator e;
    float n = run(e, 20, [](int) { return room(); });
    printf("1 steady room: N %.1f (room -70)\n", n);
    assert(std::fabs(n + 70) < 1.5);
  }
  // 2. Intermittent chat (1 s talk, 2 s pause, repeating for 30 s), then the query
  //    2 s after the last sentence: N must stay near the room
  {
    NoiseEstimator e;
    run(e, 10, [](int) { return room(); });
    float peak = -200;
    for (int k = 0; k < 10; k++) {
      peak = std::max(peak, run(e, 1, [](int) { return talk(); }));
      peak = std::max(peak, run(e, 2, [](int) { return room(); }));
    }
    float at_query = e.est;
    printf("2 intermittent chat (1/3 of the time): highest N %.1f, N at the query %.1f (room -70)\n", peak, at_query);
    assert(peak < -63 && at_query < -66);
  }
  // 3. Continuous talk / TV (talk with 0.5 s gaps every 3 s): N rises towards the talk
  {
    NoiseEstimator e;
    run(e, 10, [](int) { return room(); });
    float n = 0;
    for (int k = 0; k < 8; k++) {
      run(e, 2.5, [](int) { return talk(); });
      n = run(e, 0.5, [](int) { return room(); });
    }
    printf("3 continuous talk/TV for 24 s: N %.1f (talk about -45)\n", n);
    assert(n > -50);
    // after it stops: N falls within a few seconds
    float t = 0;
    while (e.est > -66 && t < 20) { run(e, 0.5, [](int) { return room(); }); t += 0.5f; }
    printf("  after it stops: N below -66 after %.1f s\n", t);
    assert(t <= 7.0);
  }
  // 4. Rising noise (fan switched on, -70 -> -55): N follows within ~10 s
  {
    NoiseEstimator e;
    run(e, 10, [](int) { return room(); });
    float t = 0;
    while (e.est < -57 && t < 30) { run(e, 0.5, [](int) { return -55.0f + std::uniform_real_distribution<float>(-1, 1)(rng); }); t += 0.5f; }
    printf("4 rising noise -70 -> -55: within 2 dB after %.1f s\n", t);
    assert(t <= 20.0);  // median needs ~4.5 s, then the 6 s rise smoothing
  }
  // 5. Clatter: a 0.5 s bang at -35 every 4 s: ignored
  {
    NoiseEstimator e;
    run(e, 10, [](int) { return room(); });
    float peak = -200;
    for (int k = 0; k < 6; k++) {
      peak = std::max(peak, run(e, 0.5, [](int) { return -35.0f; }));
      peak = std::max(peak, run(e, 3.5, [](int) { return room(); }));
    }
    printf("5 clatter (0.5 s every 4 s): highest N %.1f (room -70)\n", peak);
    assert(peak < -66);
  }
  printf("OK\n");
}
