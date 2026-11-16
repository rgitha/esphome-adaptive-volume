// Host test for XCorr (coherent speaker level): g++ -std=c++17 -O2 -I../components/adaptive_volume test_xcorr.cpp && ./a.out
// Mic = gain x delayed reference (+ one weak reflection) + uncorrelated "talking".
// The coherent level must follow the reply path, not the talking.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>
#include "av_core.h"
using namespace esphome::adaptive_volume;

static XCorr::Result run(float gain, int delay, float talk_db, unsigned seed) {
  std::mt19937 rng(seed);
  std::normal_distribution<float> nd(0.0f, 1.0f);
  const int N = 16000 * 3, F = 512;
  std::vector<float> x(N), y(N);
  float lp = 0, lp2 = 0;
  for (int i = 0; i < N; i++) {  // speech-like: low-passed noise with a syllable envelope
    lp = 0.7f * lp + 0.3f * nd(rng);
    const float env = 0.5f + 0.5f * std::sin(2 * 3.14159f * 4.0f * i / 16000.0f);
    x[i] = 0.05f * env * lp;
  }
  const float talk = std::pow(10.0f, talk_db / 20.0f);
  for (int i = 0; i < N; i++) {
    lp2 = 0.7f * lp2 + 0.3f * nd(rng);
    const float d = i >= delay ? x[i - delay] : 0.0f;
    const float r = i >= delay + 40 ? x[i - delay - 40] : 0.0f;  // reflection 2.5 ms later
    y[i] = gain * (d + 0.2f * r) + talk * 0.05f * lp2;
  }
  XCorr xc;
  AWeighting ax, ay;
  ax.setup(16000);
  ay.setup(16000);
  std::vector<float> fx(F), fy(F);
  for (int s = 0; s + F <= N; s += F) {
    for (int i = 0; i < F; i++) {
      fx[i] = ax.process(x[s + i]);
      fy[i] = ay.process(y[s + i]);
    }
    xc.add(fx.data(), fy.data(), F, true);
  }
  return xc.result();
}

int main() {
  const XCorr::Result clean = run(0.1f, 3, -60.0f, 1);
  printf("clean:        coherent %.2f mic %.2f ref %.2f rho %.2f lag %d\n", clean.coherent_db, clean.mic_db,
         clean.ref_db, clean.rho, clean.lag);
  assert(clean.lag == 3);
  for (float talk : {-20.0f, -10.0f, 0.0f}) {
    const XCorr::Result r = run(0.1f, 3, talk, 2);
    printf("talk %+5.0f dB: coherent %.2f mic %.2f rho %.2f lag %d (shift %+.2f dB)\n", talk, r.coherent_db, r.mic_db,
           r.rho, r.lag, r.coherent_db - clean.coherent_db);
    assert(std::fabs(r.coherent_db - clean.coherent_db) < 1.0f);
  }
  for (int d : {0, 5, 8, 20, 32}) {
    const XCorr::Result r = run(0.1f, d, -60.0f, 3);
    printf("delay %d:      coherent %.2f lag %d (shift %+.2f dB)\n", d, r.coherent_db, r.lag,
           r.coherent_db - clean.coherent_db);
    assert(r.lag == d);
    assert(std::fabs(r.coherent_db - clean.coherent_db) < 1.0f);
  }
  const XCorr::Result g2 = run(0.2f, 3, -60.0f, 1);
  printf("gain x2:      shift %+.2f dB (expect +6.02)\n", g2.coherent_db - clean.coherent_db);
  assert(std::fabs(g2.coherent_db - clean.coherent_db - 6.02f) < 0.2f);
  printf("OK\n");
}
