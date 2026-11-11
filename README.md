# esphome-adaptive-volume

> Version 0.1.0: in development and testing, not released yet.

ESPHome external component that turns all sound down in a quiet room and back
up when the room gets louder. It works on voice devices built on
[esphome-audio-stack](https://github.com/n-IA-hane/esphome-audio-stack).

- The **volume slider** in Home Assistant sets the level for a loud room. It
  never moves; the component attenuates through the codec's master volume, a
  separate gain stage.
- The device **calibrates itself** on its own spoken replies. It measures how
  loud its speech is at its microphones, so it needs no manual calibration.
- Every audio sample is measured through the audio stack's capture interface,
  so short replies (about 1 s) are measured as well as long ones.

## Usage

```yaml
external_components:
  - source: github://n-IA-hane/esphome-audio-stack@v2026.11.0
    components: [esp_audio_stack, esp_afe]
  - source: github://rgitha/esphome-adaptive-volume@main
    components: [adaptive_volume]

adaptive_volume:
  id: adaptive
  esp_audio_stack_id: audio_stack
  media_player: external_media_player
  # The device makes no sound: the room may be measured
  idle: |-
    return id(voice_assistant_phase) == 1 &&
           id(external_media_player).state == media_player::MEDIA_PLAYER_STATE_IDLE;
  # A spoken reply to a voice command is playing
  reply: |-
    return id(va_run_active) && id(voice_assistant_phase) == 5 &&
           id(external_media_player).state == media_player::MEDIA_PLAYER_STATE_ANNOUNCING;
  # Output muted or amplifier off: a reply measured now is not counted
  blocked: return id(external_media_player).is_muted();
  enable:
    name: Adaptive volume
  offset:
    name: Adaptive volume offset
  room_noise:
    name: Room noise
  speaker_level:
    name: Speaker level
  speaker_level_reply:
    name: Speaker level (last reply)
  attenuation:
    name: Adaptive volume reduction
  current_volume:
    name: Current volume
```

The three conditions are lambdas so the component fits any voice assistant
YAML. `voice_assistant_phase` and `va_run_active` above are globals of the
example device; use whatever your configuration tracks.

To give an announcement more level, call
`id(adaptive).set_announcement_offset(6)` when it starts and
`id(adaptive).set_announcement_offset(0)` when it ends. The offset reduces the
attenuation, so it has no effect when the attenuation is already 0.

## Options

| Option | Default | Meaning |
|---|---|---|
| `esp_audio_stack_id` | required | The audio stack (its capture interface and master volume) |
| `media_player` | required | Media player whose volume is the slider |
| `idle`, `reply`, `blocked` | required, required, optional | Conditions, see above |
| `speaker_level_nominal` | -35 | Speaker level S (dBFS) before the first measured reply. Set it to what this hardware measures |
| `loudness_constant` | 20 | C: speech level above room noise at the mics, at full slider (dB) |
| `max_attenuation` | 30 | Largest attenuation (dB) |
| `hysteresis` | 2 | Attenuation changes only when the target moves this far (dB) |
| `master_volume_min_db` | -40 | Must equal `master_volume_min_db` of `esp_audio_stack` |
| `software_volume_min_db` | -49 | The stack's software volume curve floor |
| `media_volume_min`, `media_volume_max` | 0, 1 | Must equal `volume_min`, `volume_max` of the media player |
| `reply_start_delay` | 300ms | Time from ANNOUNCING to the reply's sound at the mics |
| `reply_min_duration` | 400ms | Shortest measured reply |

Entities, all optional: `enable` (switch, off by default), `offset` (number,
-6 to +6 dB, + = louder), and the sensors `room_noise`, `speaker_level`,
`speaker_level_reply`, `attenuation`, `current_volume`. The noise, attenuation
and volume sensors report at most every 10 s, and only on change; user actions
show at once.

The component uses the audio stack's capture interface, so it can't be used
together with `esp_audio_capture`.

## How it works

Symbols: P = power, L = 10·log10(P) in dBFS.

**Room noise N_A.** The processed (AFE) output is A-weighted (approximately:
2nd-order high-pass at 200 Hz plus 1st-order at 500 Hz) and its RMS is taken
over 250 ms. A robust estimator then follows noise you would have to talk over
and ignores isolated sounds:

1. Peak per 0.5 s block.
2. 3rd largest of the last 16 block peaks (8 s), capped by the largest of the
   last 9 (4.5 s hold).
3. Exponential smoothing, 6 s rising, about 1.4 s falling.

Only time when the device is idle (the `idle` condition) is used; otherwise the
estimate holds.

**Room level before the reply P_room,pre.** The same estimator runs on the
unweighted processor input (mic 1, before echo cancellation), idle time only.
So it holds the room level from before the wake word.

**Speaker level S**, measured at the end of each reply (`reply` condition):

- P_in,reply = mean input power, from `reply_start_delay` after the reply
  starts until it ends.
- P_spk = P_in,reply − P_room,pre.
- Detected when the input is at least 2 dB above the room
  (P_spk ≥ 0.585·P_room,pre). Then
  S_k = 10·log10(P_spk) − V(v₀) + M₀, where V(v) is the software volume gain
  and M₀ the codec attenuation during the reply. S is the power mean of the
  last up to 3 S_k.
- Not detected: S_bound = 10·log10(0.585·P_room,pre) − V(v₀) + M₀. If S is
  above it, S drops to S_bound − 3 dB (not below nominal − 30 dB), so the next
  reply plays loud enough to be measured.
- Not counted: volume or attenuation changed, `blocked`, shorter than
  `reply_min_duration`, room reference not ready (10 s of idle after boot),
  volume 0, input above −6 dBFS.

S is saved to flash.

**Attenuation.** A_t = clamp(S − N_A − C, 0, max_attenuation), with
C = loudness_constant + offset. A follows A_t in whole dB when it moves by
`hysteresis` or more (0 below 0.5 dB). The codec gets M = max(0, A − B), B
being the announcement offset.

The logic is in `av_core.h`, which has no ESPHome dependencies; `tests/`
holds a host test (`g++ -std=c++17 -I components/adaptive_volume tests/test_core.cpp`).

## License

Same model as ESPHome: the C++ code is GPLv3, the Python code and everything
else is MIT. See [LICENSE](LICENSE).
