# esphome-adaptive-volume

> Version 0.2.0: in development and testing, not released yet.

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
| `speaker_level_nominal` | -30 | Speaker level S (dBFS) before the first measured reply. Set it to what this hardware measures |
| `loudness_constant` | 20 | C: speech level above room noise at the mics, at full slider (dB) |
| `max_attenuation` | 30 | Largest attenuation (dB) |
| `hysteresis` | 2 | Attenuation changes only when the target moves this far (dB) |
| `master_volume_min_db` | -40 | Must equal `master_volume_min_db` of `esp_audio_stack` |
| `software_volume_min_db` | -49 | The stack's software volume curve floor |
| `media_volume_min`, `media_volume_max` | 0, 1 | Must equal `volume_min`, `volume_max` of the media player |
| `reply_start_delay` | 0ms | Ignore the start of each reply for this long (normally not needed: silence is excluded anyway) |
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
processor input (mic 1, before echo cancellation), A-weighted like the room
noise, idle time only. So it holds the room level from before the wake word.

**Speaker level S**, measured at the end of each reply (`reply` condition),
on the A-weighted processor input, frame by frame (about 32 ms):

- A frame counts as speech when its power is at least twice the room
  (3 dB above). Silence, pauses and the time before the TTS audio arrives
  stay at room level and are left out.
- With at least 0.3 s of speech frames:
  S_k = 10·log10(mean over speech frames of (P_in − P_room,pre)) − V(v₀) + M₀,
  where V(v) is the software volume gain and M₀ the codec attenuation during
  the reply. S is the power mean of the last up to 3 S_k.
- Close to the room, only the louder syllables pass the gate and S_k reads
  high. So when the expected speech level (from the S in use) is less than
  9 dB above the room, S_k may only lower S, never raise it.
- Less than 0.3 s of speech that is clearly audible (at least 6 dB above the
  room): not counted.
- No speech above the room: the speaker was at most at room level, so
  S_bound = L_room,pre − V(v₀) + M₀. If S is above it, S drops to
  S_bound − 3 dB (not below nominal − 30 dB), so the next reply plays loud
  enough to be measured.
- Not counted: volume or attenuation changed, `blocked`, shorter than
  `reply_min_duration`, room reference not ready (10 s of idle after boot),
  volume 0, speech above −6 dBFS.

S is saved to flash when it has moved by 1 dB or more since the last save.

**Attenuation.** A_t = clamp(S − N_A − C, 0, max_attenuation), with
C = loudness_constant + offset. A follows A_t in whole dB when it moves by
`hysteresis` or more (0 below 0.5 dB). The codec gets M = max(0, A − B), B
being the announcement offset.

The logic is in `av_core.h`, which has no ESPHome dependencies; `tests/`
holds host tests (`g++ -std=c++17 -I components/adaptive_volume tests/test_core.cpp`, and
`tests/test_gating.cpp`, a model of TTS replies with silence at several volume settings).

## License

Same model as ESPHome: the C++ code is GPLv3, the Python code and everything
else is MIT. See [LICENSE](LICENSE).
