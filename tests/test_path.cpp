// Host test for path mode: g++ -std=c++17 -O2 -I../components/adaptive_volume test_path.cpp && ./a.out
#include <cassert>
#include <cstdio>
#include "av_core.h"
using namespace esphome::adaptive_volume;
int main(){ Core c; c.cfg.use_path=true; c.cfg.analog_gain_db=24;
  printf("nominal S %.1f\n", c.s_use()); assert(c.s_use()==15.0f);  // P -9 + analogue gain 24
  for(float v: {-8.7f,-9.1f,-30.0f,-8.9f,-9.0f}) c.push_p(v);
  printf("P %.2f S %.2f\n", c.p, c.s_use()); assert(c.p==-9.0f);
  c.push_p(5.0f); printf("after outlier P %.2f\n", c.p); assert(c.p > -9.2f && c.p < -8.6f);
  Core d; d.cfg.use_path=true; d.restore_p(-8.5f); d.push_p(-2.5f); printf("restored, then one outlier: %.2f\n", d.p); assert(d.p==-8.5f);
  d.push_p(-2.5f); d.push_p(-2.5f); printf("after three: %.2f\n", d.p); assert(d.p==-2.5f);
  puts("PATH OK"); }
