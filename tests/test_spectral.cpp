// Host test for SpectralPath: g++ -std=c++17 -O2 -I../components/adaptive_volume test_spectral.cpp && ./a.out
// A frequency-dependent path (resonance + low-pass + delay) driven by sounds with very
// different spectra must give the same path gain, close to the weighted true gain, and
// talking at the mic must not bias it.
#include <cassert>
#include <cmath>
#include <complex>
#include <cstdio>
#include <random>
#include <vector>
#include "av_core.h"
using namespace esphome::adaptive_volume;

static const float FS = 16000.0f;
struct Biquad {  // RBJ peaking / low-pass
  float b0, b1, b2, a1, a2, x1 = 0, x2 = 0, y1 = 0, y2 = 0;
  float run(float x) {
    float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
    x2 = x1; x1 = x; y2 = y1; y1 = y;
    return y;
  }
  std::complex<double> resp(double f) const {
    std::complex<double> z = std::polar(1.0, -2 * M_PI * f / FS);
    return (double(b0) + double(b1) * z + double(b2) * z * z) / (1.0 + double(a1) * z + double(a2) * z * z);
  }
};
static Biquad peaking(float f0, float q, float gain_db) {
  float A = std::pow(10.0f, gain_db / 40), w = 2 * M_PI * f0 / FS, al = std::sin(w) / (2 * q), a0 = 1 + al / A;
  return {(1 + al * A) / a0, -2 * std::cos(w) / a0, (1 - al * A) / a0, -2 * std::cos(w) / a0, (1 - al / A) / a0};
}
static Biquad lowpass(float f0, float q) {
  float w = 2 * M_PI * f0 / FS, al = std::sin(w) / (2 * q), c = std::cos(w), a0 = 1 + al;
  return {(1 - c) / 2 / a0, (1 - c) / a0, (1 - c) / 2 / a0, -2 * c / a0, (1 - al) / a0};
}

// the path: 0.3 x peaking(650 Hz, Q2, +8 dB) x low-pass(2.5 kHz) x 11-sample delay
static double true_weighted_db() {
  Biquad p = peaking(650, 2, 8), l = lowpass(2500, 0.707);
  double acc = 0, w = 0;
  for (int k = 0; k <= 256; k++) {
    double f = k * FS / 512;
    if (f < 200 || f > 3000) continue;
    double g = 0.3 * std::abs(p.resp(f) * l.resp(f));
    acc += SpectralPath::weight(f) * g * g;
    w += SpectralPath::weight(f);
  }
  return 10 * std::log10(acc / w);
}

static SpectralPath::Result run(float tilt_pole, float talk, unsigned seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<float> nd(0, 1);
  std::vector<float> mem(SpectralPath::MEM_FLOATS);
  SpectralPath sp;
  sp.attach(mem.data());
  Biquad p = peaking(650, 2, 8), l = lowpass(2500, 0.707);
  std::vector<float> hist(16, 0.0f);
  float lp = 0, tl = 0;
  const int F = 512;
  std::vector<float> x(F), y(F);
  for (int fr = 0; fr < 150; fr++) {
    for (int i = 0; i < F; i++) {
      lp = tilt_pole * lp + (1 - tilt_pole) * nd(rng);  // spectral tilt of the "sound"
      const float xi = lp;
      for (int k = 15; k > 0; k--) hist[k] = hist[k - 1];
      hist[0] = xi;
      tl = 0.6f * tl + 0.4f * nd(rng);
      x[i] = xi;
      y[i] = 0.3f * l.run(p.run(hist[11])) + talk * tl;
    }
    sp.push(x.data(), y.data(), F, true);
  }
  return sp.result(FS);
}

int main() {
  const double truth = true_weighted_db();
  printf("true weighted path gain %.2f dB\n", truth);
  double lo = 1e9, hi = -1e9;
  for (float pole : {0.0f, 0.5f, 0.8f, 0.93f}) {
    auto r = run(pole, 0.0f, 1);
    printf("sound tilt pole %.2f: P %.2f dB, coverage %.2f, coherence %.2f\n", pole, r.p_db, r.coverage, r.coherence);
    assert(std::fabs(r.p_db - truth) < 0.5);
    lo = std::min(lo, double(r.p_db));
    hi = std::max(hi, double(r.p_db));
  }
  printf("spread over sounds %.2f dB\n", hi - lo);
  assert(hi - lo < 0.5);
  auto t = run(0.5f, 0.2f, 2);
  printf("talking at the mic: P %.2f dB, coherence %.2f\n", t.p_db, t.coherence);
  assert(std::fabs(t.p_db - truth) < 0.5 && t.coherence < 0.9);
  printf("OK\n");
}
