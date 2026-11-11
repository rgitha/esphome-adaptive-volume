// Model of speech gating: g++ -std=c++17 -O2 -I../components/adaptive_volume test_gating.cpp && ./a.out
// TTS file: 4 s, of which 1.5 s speech (syllables, 6 dB lognormal per 32 ms frame),
// silence otherwise. Replies run with the full attenuation feedback, at several
// offsets / room levels / slider settings; S must settle and not oscillate.
#include <cstdio>
#include <random>
#include <vector>

#include "av_core.h"

using namespace esphome::adaptive_volume;

static double pw(double db) { return std::pow(10.0, db / 10.0); }

struct Run {
  Core core;
  std::mt19937 rng;
  double s_true;          // mean speech power at the mics, referred to V = 0, M = 0
  double room_in, room_aw;
  float volume;
  uint64_t t = 0;
  Run(double s_true, double room_in, double room_aw, float volume, float offset, float nominal, unsigned seed)
      : rng(seed), s_true(s_true), room_in(room_in), room_aw(room_aw), volume(volume) {
    core.cfg.media_volume_min = 0.1f;
    core.cfg.speaker_level_nominal = nominal;
    core.enabled = true;
    core.offset = offset;
  }
  void frame(bool idle, double speech_db) {
    std::normal_distribution<double> j(0, 1);
    double p = pw(room_in + 0.5 * j(rng));
    if (!std::isnan(speech_db))
      p += pw(speech_db);
    t += 32000;
    core.on_input(p * 512, 512, t, idle);
    if (core.on_aweighted(pw(room_aw + 0.5 * j(rng)) * 512, 512, idle))
      core.update_attenuation();
  }
  void idle(double sec) {
    for (int i = 0; i < sec / 0.032; i++) {
      frame(true, NAN);
      core.reply_update(false, t, volume, false);
    }
  }
  CalResult reply(double total = 4.0, double speech = 1.5, double lead = 0.3) {
    std::normal_distribution<double> j(0, 1);
    const double sigma = 6.0, bias = sigma * sigma * std::log(10.0) / 20.0;
    core.reply_update(true, t, volume, false);
    const int n = total / 0.032, s0 = lead / 0.032, s1 = (lead + speech) / 0.032;
    for (int i = 0; i < n; i++) {
      double sp = NAN;
      if (i >= s0 && i < s1)
        sp = s_true + core.v_db(volume) - core.m - bias + sigma * j(rng);
      frame(false, sp);
      core.reply_update(true, t, volume, false);
    }
    auto r = core.reply_update(false, t, volume, false);
    core.update_attenuation();
    return r;
  }
};

static const char *kind(int k) {
  static const char *n[] = {"none", "skip", "meas", "lower", "within"};
  return n[k];
}

int main() {
  int bad = 0;
  struct Case { const char *name; double room_in, room_aw; float volume, offset, nominal; };
  // Input and room are A-weighted (same weighting as N_A), so they sit at about the same level.
  const Case cases[] = {
      {"room -71/-71, slider 95%, offset 0", -71, -71, 0.95f, 0, -29},
      {"room -71/-71, slider 95%, offset -3", -71, -71, 0.95f, -3, -29},
      {"room -71/-71, slider 95%, offset -6", -71, -71, 0.95f, -6, -29},
      {"room -71/-71, slider 95%, offset +6", -71, -71, 0.95f, 6, -29},
      {"room -71/-71, slider 70%, offset 0", -71, -71, 0.70f, 0, -29},
      {"room -71/-71, slider 50%, offset -6", -71, -71, 0.50f, -6, -29},
      {"room -71/-71, nominal 10 dB too high", -71, -71, 0.95f, 0, -17},
      {"room -71/-71, nominal 20 dB too high, offset -6", -71, -71, 0.95f, -6, -7},
      {"room -71/-71, nominal 8 dB too low", -71, -71, 0.95f, 0, -35},
      {"loud room -58/-58, slider 95%", -58, -58, 0.95f, 0, -29},
      {"very quiet -80/-80, slider 95%", -80, -80, 0.95f, 0, -29},
  };
  for (const auto &c : cases) {
    Run run(-29.0, c.room_in, c.room_aw, c.volume, c.offset, c.nominal, 11);
    run.idle(15);
    printf("%s\n  ", c.name);
    std::vector<float> s_hist;
    for (int i = 0; i < 12; i++) {
      run.idle(2);
      auto r = run.reply();
      printf("%s %.1f/A%.0f  ", kind(r.kind), run.core.s_use(), run.core.a);
      s_hist.push_back(run.core.s_use());
    }
    // settled: last 6 within 3 dB of the truth, or consistently below their bound (low slider)
    float worst = 0;
    for (size_t i = 6; i < s_hist.size(); i++)
      worst = std::max(worst, std::fabs(s_hist[i] - (-29.0f)));
    float mean = 0; for (size_t i = 6; i < s_hist.size(); i++) mean += s_hist[i]; mean /= (s_hist.size() - 6);
    printf("\n  last 6: mean S %.1f (true -29.0), worst error %.1f dB\n", mean, worst);
    if (worst > 3.0f)
      bad++;
  }
  printf(bad ? "%d cases not settled\n" : "all settled\n", bad);
  return bad ? 1 : 0;
}
