#pragma once

#ifdef USE_ESP32

#include <atomic>
#include <functional>

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#include "esphome/components/esp_audio_stack/esp_audio_stack.h"
#include "esphome/components/media_player/media_player.h"
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif

#include "av_core.h"

namespace esphome {
namespace adaptive_volume {

static constexpr const char *ADAPTIVE_VOLUME_VERSION = "0.3.1";

class AdaptiveVolume;

#ifdef USE_SWITCH
class EnableSwitch : public switch_::Switch, public Parented<AdaptiveVolume> {
 protected:
  void write_state(bool state) override;
};
#endif

#ifdef USE_NUMBER
class OffsetNumber : public number::Number, public Parented<AdaptiveVolume> {
 public:
  void restore();
 protected:
  void control(float value) override;
  ESPPreferenceObject pref_;
};
#endif

// Lock-free single-producer (audio task) / single-consumer (main loop) ring of
// per-frame block powers.
struct Block {
  uint8_t point;  // 0 = input (mic 1, before AEC), 1 = A-weighted output
  float sum_sq;   // sum of squared normalized samples
  uint32_t n;     // samples
  uint64_t ts_us;
  uint16_t peak;  // input only: largest absolute sample over all mics
};

class AdaptiveVolume : public Component, public esp_audio_stack::AudioCaptureObserver {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

  // AudioCaptureObserver (audio task)
  bool capture_active(esp_audio_stack::CapturePoint point) const override;
  void on_capture_frame(const esp_audio_stack::CaptureFrame &frame) override;

  // configuration
  void set_stack(esp_audio_stack::ESPAudioStack *stack) { this->stack_ = stack; }
  void set_media_player(media_player::MediaPlayer *mp) { this->media_player_ = mp; }
  void set_idle_lambda(std::function<bool()> &&f) { this->idle_f_ = std::move(f); }
  void set_reply_lambda(std::function<bool()> &&f) { this->reply_f_ = std::move(f); }
  void set_blocked_lambda(std::function<bool()> &&f) { this->blocked_f_ = std::move(f); }
  void set_speaker_level_nominal(float v) { this->core_.cfg.speaker_level_nominal = v; }
  void set_loudness_constant(float v) { this->core_.cfg.loudness_constant = v; }
  void set_max_attenuation(float v) { this->core_.cfg.max_attenuation = v; }
  void set_hysteresis(float v) { this->core_.cfg.hysteresis = v; }
  void set_master_volume_min_db(float v) { this->master_min_db_ = v; }
  void set_software_volume_min_db(float v) { this->core_.cfg.software_volume_min_db = v; }
  void set_media_volume_range(float lo, float hi) {
    this->core_.cfg.media_volume_min = lo;
    this->core_.cfg.media_volume_max = hi;
  }
  void set_reply_start_delay(uint32_t ms) { this->core_.cfg.reply_start_delay_us = ms * 1000u; }
  void set_reply_min_duration(uint32_t ms) { this->core_.cfg.reply_min_duration_s = ms / 1000.0f; }
#ifdef USE_SENSOR
  void set_room_noise_sensor(sensor::Sensor *s) { this->room_noise_sensor_ = s; }
  void set_speaker_level_sensor(sensor::Sensor *s) { this->speaker_level_sensor_ = s; }
  void set_speaker_level_reply_sensor(sensor::Sensor *s) { this->speaker_level_reply_sensor_ = s; }
  void set_echo_reduction_sensor(sensor::Sensor *s) { this->echo_reduction_sensor_ = s; }
  void set_mic_peak_sensor(sensor::Sensor *s) { this->mic_peak_sensor_ = s; }
  void set_attenuation_sensor(sensor::Sensor *s) { this->attenuation_sensor_ = s; }
  void set_current_volume_sensor(sensor::Sensor *s) { this->current_volume_sensor_ = s; }
#endif
#ifdef USE_SWITCH
  void set_enable_switch(EnableSwitch *s) { this->enable_switch_ = s; }
#endif
#ifdef USE_NUMBER
  void set_offset_number(OffsetNumber *n) { this->offset_number_ = n; }
#endif

  // runtime API (from YAML lambdas)
  void set_enabled(bool enabled);
  void set_offset(float db);
  /// Announcement offset B (dB): attenuation is reduced by this while it is set.
  void set_announcement_offset(float db);
  float get_attenuation() const { return this->core_.a; }
  float get_speaker_level() const { return this->core_.s_use(); }
  float get_room_noise() const { return this->core_.n_a; }

 protected:
  void apply_(bool immediate);
  void apply_master_();
  void publish_reports_(bool force);
  void log_result_(const CalResult &r);
  void echo_finish_();

  esp_audio_stack::ESPAudioStack *stack_{nullptr};
  media_player::MediaPlayer *media_player_{nullptr};
  std::function<bool()> idle_f_, reply_f_, blocked_f_;
  float master_min_db_{-40.0f};
  Core core_;
  EchoMeter echo_;
  ESPPreferenceObject s_pref_;

  // audio-task state (A-weighting filters)
  AWeighting aw_in_, aw_out_;
  float saved_s_{NAN};

  static constexpr uint32_t RING = 128;
  Block ring_[RING];
  std::atomic<uint32_t> ring_head_{0}, ring_tail_{0};
  std::atomic<uint32_t> ring_dropped_{0};
  void push_(const Block &b);

  // reporting
  uint32_t last_report_ms_{0};
  float reported_attenuation_{NAN}, reported_volume_{NAN}, reported_noise_{NAN};
  float last_master_m_{-1.0f};
  float last_volume_{NAN};

#ifdef USE_SENSOR
  sensor::Sensor *room_noise_sensor_{nullptr};
  sensor::Sensor *speaker_level_sensor_{nullptr};
  sensor::Sensor *speaker_level_reply_sensor_{nullptr};
  sensor::Sensor *echo_reduction_sensor_{nullptr};
  sensor::Sensor *mic_peak_sensor_{nullptr};
  sensor::Sensor *attenuation_sensor_{nullptr};
  sensor::Sensor *current_volume_sensor_{nullptr};
#endif
#ifdef USE_SWITCH
  EnableSwitch *enable_switch_{nullptr};
#endif
#ifdef USE_NUMBER
  OffsetNumber *offset_number_{nullptr};
#endif
};

}  // namespace adaptive_volume
}  // namespace esphome

#endif  // USE_ESP32
