"""Adaptive volume for esp_audio_stack voice devices.

Follows the room noise and calibrates itself on the device's own spoken
replies; applies the result as codec master attenuation so the media volume
(the user's slider) never moves.
"""

import esphome.codegen as cg
from esphome.components import media_player, number, sensor, switch
from esphome.components.esp_audio_stack import ESPAudioStack
import esphome.config_validation as cv
import esphome.final_validate as fv
from esphome.const import (
    CONF_ID,
    CONF_MEDIA_PLAYER,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
)

__version__ = "0.3.0"

CODEOWNERS = ["@rgitha"]
DEPENDENCIES = ["esp_audio_stack", "media_player"]
AUTO_LOAD = ["sensor", "switch", "number"]

CONF_ESP_AUDIO_STACK_ID = "esp_audio_stack_id"
CONF_IDLE = "idle"
CONF_REPLY = "reply"
CONF_BLOCKED = "blocked"
CONF_SPEAKER_LEVEL_NOMINAL = "speaker_level_nominal"
CONF_LOUDNESS_CONSTANT = "loudness_constant"
CONF_MAX_ATTENUATION = "max_attenuation"
CONF_HYSTERESIS = "hysteresis"
CONF_MASTER_VOLUME_MIN_DB = "master_volume_min_db"
CONF_SOFTWARE_VOLUME_MIN_DB = "software_volume_min_db"
CONF_MEDIA_VOLUME_MIN = "media_volume_min"
CONF_MEDIA_VOLUME_MAX = "media_volume_max"
CONF_REPLY_START_DELAY = "reply_start_delay"
CONF_REPLY_MIN_DURATION = "reply_min_duration"
CONF_ENABLE = "enable"
CONF_OFFSET = "offset"
CONF_ROOM_NOISE = "room_noise"
CONF_SPEAKER_LEVEL = "speaker_level"
CONF_SPEAKER_LEVEL_REPLY = "speaker_level_reply"
CONF_ATTENUATION = "attenuation"
CONF_CURRENT_VOLUME = "current_volume"

UNIT_DB = "dB"
UNIT_DBFS = "dBFS"

adaptive_volume_ns = cg.esphome_ns.namespace("adaptive_volume")
AdaptiveVolume = adaptive_volume_ns.class_("AdaptiveVolume", cg.Component)
EnableSwitch = adaptive_volume_ns.class_(
    "EnableSwitch", switch.Switch, cg.Parented.template(AdaptiveVolume)
)
OffsetNumber = adaptive_volume_ns.class_(
    "OffsetNumber", number.Number, cg.Parented.template(AdaptiveVolume)
)


def _db(value):
    return cv.float_(value)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(AdaptiveVolume),
            cv.Required(CONF_ESP_AUDIO_STACK_ID): cv.use_id(ESPAudioStack),
            cv.Required(CONF_MEDIA_PLAYER): cv.use_id(media_player.MediaPlayer),
            # When the room may be measured (the device makes no sound).
            cv.Required(CONF_IDLE): cv.returning_lambda,
            # When a spoken reply to a voice command is playing.
            cv.Required(CONF_REPLY): cv.returning_lambda,
            # Output muted or amplifier off (invalidates a reply measurement).
            cv.Optional(CONF_BLOCKED): cv.returning_lambda,
            cv.Optional(CONF_SPEAKER_LEVEL_NOMINAL, default=-30.0): cv.float_range(
                min=-90.0, max=0.0
            ),
            cv.Optional(CONF_LOUDNESS_CONSTANT, default=20.0): cv.float_range(
                min=-20.0, max=60.0
            ),
            cv.Optional(CONF_MAX_ATTENUATION, default=30.0): cv.float_range(
                min=0.0, max=60.0
            ),
            cv.Optional(CONF_HYSTERESIS, default=2.0): cv.float_range(
                min=0.0, max=10.0
            ),
            cv.Optional(CONF_MASTER_VOLUME_MIN_DB, default=-40.0): cv.float_range(
                min=-96.0, max=-1.0
            ),
            cv.Optional(CONF_SOFTWARE_VOLUME_MIN_DB, default=-49.0): cv.float_range(
                min=-96.0, max=-1.0
            ),
            cv.Optional(CONF_MEDIA_VOLUME_MIN, default=0.0): cv.percentage,
            cv.Optional(CONF_MEDIA_VOLUME_MAX, default=1.0): cv.percentage,
            cv.Optional(
                CONF_REPLY_START_DELAY, default="0ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_REPLY_MIN_DURATION, default="400ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_ENABLE): switch.switch_schema(
                EnableSwitch,
                default_restore_mode="RESTORE_DEFAULT_OFF",
                icon="mdi:volume-vibrate",
            ),
            cv.Optional(CONF_OFFSET): number.number_schema(
                OffsetNumber,
                icon="mdi:tune-vertical",
                entity_category=ENTITY_CATEGORY_CONFIG,
                unit_of_measurement=UNIT_DB,
            ),
            cv.Optional(CONF_ROOM_NOISE): sensor.sensor_schema(
                unit_of_measurement=UNIT_DBFS,
                icon="mdi:waveform",
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_SPEAKER_LEVEL): sensor.sensor_schema(
                unit_of_measurement=UNIT_DBFS,
                icon="mdi:speaker-wireless",
                accuracy_decimals=1,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_SPEAKER_LEVEL_REPLY): sensor.sensor_schema(
                unit_of_measurement=UNIT_DBFS,
                icon="mdi:speaker-message",
                accuracy_decimals=1,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_ATTENUATION): sensor.sensor_schema(
                unit_of_measurement=UNIT_DB,
                icon="mdi:volume-minus",
                accuracy_decimals=0,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_CURRENT_VOLUME): sensor.sensor_schema(
                unit_of_measurement="%",
                icon="mdi:volume-medium",
                accuracy_decimals=0,
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
)


def _final_validate(config):
    full = fv.full_config.get()
    if "esp_audio_capture" in full:
        raise cv.Invalid(
            "adaptive_volume uses the audio stack's capture interface, which "
            "esp_audio_capture also needs; only one of them can be used"
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config):
    cg.add_define("USE_ESP_AUDIO_STACK_CAPTURE")
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_stack(await cg.get_variable(config[CONF_ESP_AUDIO_STACK_ID])))
    cg.add(var.set_media_player(await cg.get_variable(config[CONF_MEDIA_PLAYER])))

    for key, setter in (
        (CONF_IDLE, var.set_idle_lambda),
        (CONF_REPLY, var.set_reply_lambda),
        (CONF_BLOCKED, var.set_blocked_lambda),
    ):
        if key in config:
            lam = await cg.process_lambda(config[key], [], return_type=cg.bool_)
            cg.add(setter(lam))

    cg.add(var.set_speaker_level_nominal(config[CONF_SPEAKER_LEVEL_NOMINAL]))
    cg.add(var.set_loudness_constant(config[CONF_LOUDNESS_CONSTANT]))
    cg.add(var.set_max_attenuation(config[CONF_MAX_ATTENUATION]))
    cg.add(var.set_hysteresis(config[CONF_HYSTERESIS]))
    cg.add(var.set_master_volume_min_db(config[CONF_MASTER_VOLUME_MIN_DB]))
    cg.add(var.set_software_volume_min_db(config[CONF_SOFTWARE_VOLUME_MIN_DB]))
    cg.add(
        var.set_media_volume_range(
            config[CONF_MEDIA_VOLUME_MIN], config[CONF_MEDIA_VOLUME_MAX]
        )
    )
    cg.add(
        var.set_reply_start_delay(config[CONF_REPLY_START_DELAY].total_milliseconds)
    )
    cg.add(
        var.set_reply_min_duration(
            config[CONF_REPLY_MIN_DURATION].total_milliseconds
        )
    )

    if CONF_ENABLE in config:
        sw = await switch.new_switch(config[CONF_ENABLE])
        await cg.register_parented(sw, var)
        cg.add(var.set_enable_switch(sw))
    if CONF_OFFSET in config:
        num = await number.new_number(
            config[CONF_OFFSET], min_value=-6.0, max_value=6.0, step=1.0
        )
        await cg.register_parented(num, var)
        cg.add(var.set_offset_number(num))
    for key, setter in (
        (CONF_ROOM_NOISE, var.set_room_noise_sensor),
        (CONF_SPEAKER_LEVEL, var.set_speaker_level_sensor),
        (CONF_SPEAKER_LEVEL_REPLY, var.set_speaker_level_reply_sensor),
        (CONF_ATTENUATION, var.set_attenuation_sensor),
        (CONF_CURRENT_VOLUME, var.set_current_volume_sensor),
    ):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(setter(sens))
