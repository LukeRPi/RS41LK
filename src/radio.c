#include <stdio.h>

#include "config.h"
#include "log.h"
#include "template.h"
#include "hal/system.h"
#include "hal/delay.h"
#include "hal/usart_gps.h"
#include "codecs/morse/morse.h"
#include "codecs/bell/bell.h"
#include "codecs/mfsk/mfsk.h"
#include "drivers/ubxg6010/ubxg6010.h"
#include "radio_internal.h"
#include "radio_si4032.h"
#include "radio_payload_cw.h"
#include "radio_payload_aprs_weather.h"


radio_transmit_entry radio_transmit_schedule[] = {
#if RADIO_SI4032_TX_HORUS_V1_CONTINUOUS
        {
                .enabled = RADIO_SI4032_TX_HORUS_V1,
                .radio_type = RADIO_TYPE_SI4032,
                .data_mode = RADIO_DATA_MODE_HORUS_V1,
                .time_sync_seconds = HORUS_V1_TIME_SYNC_SECONDS,
                .time_sync_seconds_offset = HORUS_V1_TIME_SYNC_OFFSET_SECONDS,
                .frequency = RADIO_SI4032_TX_FREQUENCY_HORUS_V1,
                .tx_power = RADIO_SI4032_TX_POWER,
                .symbol_rate = HORUS_V1_BAUD_RATE_SI4032,
                .payload_encoder = &radio_horus_v1_payload_encoder,
                .fsk_encoder_api = &mfsk_fsk_encoder_api,
        },
        {
                .enabled = RADIO_SI4032_TX_HORUS_V1,
                .radio_type = RADIO_TYPE_SI4032,
                .data_mode = RADIO_DATA_MODE_HORUS_V1,
                .time_sync_seconds = HORUS_V1_TIME_SYNC_SECONDS,
                .time_sync_seconds_offset = HORUS_V1_TIME_SYNC_OFFSET_SECONDS,
                .frequency = RADIO_SI4032_TX_FREQUENCY_HORUS_V1,
                .tx_power = RADIO_SI4032_TX_POWER,
                .symbol_rate = HORUS_V1_BAUD_RATE_SI4032,
                .payload_encoder = &radio_horus_v1_idle_encoder,
                .fsk_encoder_api = &mfsk_fsk_encoder_api,
        },
#elif RADIO_SI4032_TX_HORUS_V2_CONTINUOUS
        {
                .enabled = RADIO_SI4032_TX_HORUS_V2,
                .radio_type = RADIO_TYPE_SI4032,
                .data_mode = RADIO_DATA_MODE_HORUS_V2,
                .time_sync_seconds = HORUS_V2_TIME_SYNC_SECONDS,
                .time_sync_seconds_offset = HORUS_V2_TIME_SYNC_OFFSET_SECONDS,
                .frequency = RADIO_SI4032_TX_FREQUENCY_HORUS_V2,
                .tx_power = RADIO_SI4032_TX_POWER,
                .symbol_rate = HORUS_V2_BAUD_RATE_SI4032,
                .payload_encoder = &radio_horus_v2_payload_encoder,
                .fsk_encoder_api = &mfsk_fsk_encoder_api,
        },
        {
                .enabled = RADIO_SI4032_TX_HORUS_V2,
                .radio_type = RADIO_TYPE_SI4032,
                .data_mode = RADIO_DATA_MODE_HORUS_V2,
                .time_sync_seconds = HORUS_V2_TIME_SYNC_SECONDS,
                .time_sync_seconds_offset = HORUS_V2_TIME_SYNC_OFFSET_SECONDS,
                .frequency = RADIO_SI4032_TX_FREQUENCY_HORUS_V2,
                .tx_power = RADIO_SI4032_TX_POWER,
                .symbol_rate = HORUS_V2_BAUD_RATE_SI4032,
                .payload_encoder = &radio_horus_v2_idle_encoder,
                .fsk_encoder_api = &mfsk_fsk_encoder_api,
        },
#else
        {
                .enabled = RADIO_SI4032_TX_CW,
                .radio_type = RADIO_TYPE_SI4032,
                .data_mode = RADIO_DATA_MODE_CW,
                .transmit_count = RADIO_SI4032_TX_CW_COUNT,
                .time_sync_seconds = CW_TIME_SYNC_SECONDS,
                .time_sync_seconds_offset = CW_TIME_SYNC_OFFSET_SECONDS,
                .frequency = RADIO_SI4032_TX_FREQUENCY_CW,
                .tx_power = RADIO_SI4032_TX_POWER,
                .symbol_rate = MORSE_WPM_TO_SYMBOL_RATE(CW_SPEED_WPM),
                .payload_encoder = &radio_cw_payload_encoder,
                .fsk_encoder_api = &morse_fsk_encoder_api,
        },
        {
                .enabled = RADIO_SI4032_TX_APRS,
                .radio_type = RADIO_TYPE_SI4032,
                .data_mode = RADIO_DATA_MODE_APRS_1200,
                .transmit_count = RADIO_SI4032_TX_APRS_COUNT,
                .time_sync_seconds = APRS_TIME_SYNC_SECONDS,
                .time_sync_seconds_offset = APRS_TIME_SYNC_OFFSET_SECONDS,
                .frequency = RADIO_SI4032_TX_FREQUENCY_APRS_1200,
                .tx_power = RADIO_SI4032_TX_POWER,
                .symbol_rate = 1200,
#if APRS_WEATHER_REPORT_ENABLE
                .payload_encoder = &radio_aprs_weather_report_payload_encoder,
#endif
                .fsk_encoder_api = &bell_fsk_encoder_api,
        },
        
#endif
        {
                .end = true,
        }
};

static volatile bool radio_module_initialized = false;


radio_transmit_entry *radio_current_transmit_entry = NULL;
static uint8_t radio_current_transmit_entry_index = 0;
static uint8_t radio_transmit_entry_count = 0;

static volatile uint32_t radio_post_transmit_delay_counter = 0;
static volatile uint32_t radio_next_symbol_counter = 0;

static radio_transmit_entry *radio_start_transmit_entry = NULL;

static uint32_t radio_previous_time_sync_scheduled = 0;

char radio_current_payload_message[RADIO_PAYLOAD_MESSAGE_MAX_LENGTH];

uint8_t radio_current_payload[RADIO_PAYLOAD_MAX_LENGTH];
uint16_t radio_current_payload_length = 0;

uint8_t radio_current_symbol_data[RADIO_SYMBOL_DATA_MAX_LENGTH];

static volatile uint32_t start_tick = 0, end_tick = 0;

telemetry_data current_telemetry_data;

radio_module_state radio_shared_state = {
        .radio_transmission_active = false,
        .radio_transmission_finished = false,
        .radio_transmit_next_symbol_flag = false,
        .radio_symbol_count_interrupt = 0,
        .radio_symbol_count_loop = 0,
        .radio_dma_transfer_active = false,
        .radio_manual_transmit_active = false,
        .radio_interrupt_transmit_active = false,

        .radio_current_fsk_tones = NULL,
        .radio_current_fsk_tone_count = 0,
        .radio_current_tone_spacing_hz_100 = 0,

        .radio_current_symbol_rate = 0,
        .radio_current_symbol_delay_ms_100 = 0
};


static inline void radio_reset_next_symbol_counter()
{
    if (radio_shared_state.radio_current_symbol_rate > 0) {
        radio_next_symbol_counter = (uint32_t) (((float) SYSTEM_SCHEDULER_TIMER_TICKS_PER_SECOND) /
                                                (float) radio_shared_state.radio_current_symbol_rate);
    } else {
        radio_next_symbol_counter = (uint32_t) (((float) radio_shared_state.radio_current_symbol_delay_ms_100) *
                                                (float) SYSTEM_SCHEDULER_TIMER_TICKS_PER_SECOND / 100000.0f);
    }
}

static inline bool radio_should_transmit_next_symbol()
{
    return radio_next_symbol_counter == 0;
}

static bool radio_start_transmit(radio_transmit_entry *entry)
{
    bool success;

    radio_shared_state.radio_symbol_count_interrupt = 0;
    radio_shared_state.radio_symbol_count_loop = 0;

    telemetry_collect(&current_telemetry_data);

    if (entry->messages != NULL && entry->message_count > 0) {
        template_replace(radio_current_payload_message, sizeof(radio_current_payload_message),
                entry->messages[entry->current_message_index], &current_telemetry_data);
    } else {
        radio_current_payload_message[0] = '\0';
    }

    radio_current_payload_length = entry->payload_encoder->encode(
            radio_current_payload, sizeof(radio_current_payload),
            &current_telemetry_data, radio_current_payload_message);

    log_info("Full payload length: %d\n", radio_current_payload_length);

#ifdef SEMIHOSTING_ENABLE
    log_info("Payload: ");
    log_bytes_hex(radio_current_payload_length, (char *) radio_current_payload);
    log_info("\n    ");
    log_bytes(radio_current_payload_length, (char *) radio_current_payload);
    log_info("\n");
#endif

    // USART interrupts may interfere with transmission timing
    bool enable_gps_during_transmit = false;

    switch (entry->data_mode) {
        case RADIO_DATA_MODE_CW:
            // CW timing is not as critical
            enable_gps_during_transmit = true;

            morse_encoder_new(&entry->fsk_encoder, entry->symbol_rate);
            radio_shared_state.radio_current_symbol_rate = entry->fsk_encoder_api->get_symbol_rate(&entry->fsk_encoder);
            entry->fsk_encoder_api->get_tones(&entry->fsk_encoder, &radio_shared_state.radio_current_fsk_tone_count,
                    &radio_shared_state.radio_current_fsk_tones);
            entry->fsk_encoder_api->set_data(&entry->fsk_encoder, radio_current_payload_length, radio_current_payload);
            break;
        case RADIO_DATA_MODE_RTTY:
            break;
        case RADIO_DATA_MODE_APRS_1200:
            // TODO: make bell tones and flag field count configurable
            bell_encoder_new(&entry->fsk_encoder, entry->symbol_rate, BELL_FLAG_FIELD_COUNT_1200, bell202_tones);
            radio_shared_state.radio_current_symbol_rate = entry->fsk_encoder_api->get_symbol_rate(&entry->fsk_encoder);
            entry->fsk_encoder_api->get_tones(&entry->fsk_encoder, &radio_shared_state.radio_current_fsk_tone_count,
                    &radio_shared_state.radio_current_fsk_tones);
            entry->fsk_encoder_api->set_data(&entry->fsk_encoder, radio_current_payload_length, radio_current_payload);
            break;
        default:
            return false;
    }

    usart_gps_enable(enable_gps_during_transmit);
    if (!enable_gps_during_transmit) {
        ubxg6010_reset_parser();
    }

    switch (entry->radio_type) {
        case RADIO_TYPE_SI4032:
            success = radio_start_transmit_si4032(entry, &radio_shared_state);
            break;
        default:
            return false;
    }

    if (!success) {
        usart_gps_enable(true);
        // TODO: stop transmit here
        return false;
    }

    log_info("TX start\n");

    if (leds_enabled) {
        system_set_red_led(true);
    }

    radio_shared_state.radio_transmission_active = true;

    return true;
}

static inline void radio_reset_transmit_state()
{
    radio_shared_state.radio_transmission_active = false;

    radio_next_symbol_counter = 0;
    radio_current_payload_length = 0;

    radio_shared_state.radio_current_fsk_tones = NULL;
    radio_shared_state.radio_current_fsk_tone_count = 0;
    radio_shared_state.radio_current_tone_spacing_hz_100 = 0;

    radio_shared_state.radio_current_symbol_rate = 0;
    radio_shared_state.radio_current_symbol_delay_ms_100 = 0;
}

static bool radio_stop_transmit(radio_transmit_entry *entry)
{
    bool success;

    switch (entry->radio_type) {
        case RADIO_TYPE_SI4032:
            success = radio_stop_transmit_si4032(entry, &radio_shared_state);
            break;
        default:
            return false;
    }

    radio_reset_transmit_state();

    radio_shared_state.radio_manual_transmit_active = false;
    radio_shared_state.radio_interrupt_transmit_active = false;
    radio_shared_state.radio_dma_transfer_active = false;

    switch (entry->data_mode) {
        case RADIO_DATA_MODE_CW:
            morse_encoder_destroy(&entry->fsk_encoder);
            break;
        case RADIO_DATA_MODE_RTTY:
            break;
        case RADIO_DATA_MODE_APRS_1200:
            bell_encoder_destroy(&entry->fsk_encoder);
            break;
        default:
            return false;
    }

    usart_gps_enable(true);
    if (leds_enabled) {
        system_set_red_led(false);
    }

    return success;
}

static bool radio_transmit_symbol(radio_transmit_entry *entry)
{
    bool success;

    switch (entry->radio_type) {
        case RADIO_TYPE_SI4032:
            success = radio_transmit_symbol_si4032(entry, &radio_shared_state);
            break;
        default:
            return false;
    }

    if (success) {
        radio_shared_state.radio_symbol_count_interrupt++;
    }

    return success;
}

static void radio_reset_transmit_delay_counter()
{
    radio_post_transmit_delay_counter = RADIO_POST_TRANSMIT_DELAY_MS * SYSTEM_SCHEDULER_TIMER_TICKS_PER_SECOND / 1000;
}

static void radio_next_transmit_entry()
{
    radio_current_transmit_entry->current_message_index =
            (radio_current_transmit_entry->current_message_index + 1) % radio_current_transmit_entry->message_count;

    radio_current_transmit_entry->current_transmit_index =
            (radio_current_transmit_entry->current_transmit_index + 1) % radio_current_transmit_entry->transmit_count;

    // Proceed to next transmit entry when transmit counter wraps
    if (radio_current_transmit_entry->current_transmit_index == 0) {
        do {
            radio_current_transmit_entry_index = (radio_current_transmit_entry_index + 1) % radio_transmit_entry_count;
            radio_current_transmit_entry = &radio_transmit_schedule[radio_current_transmit_entry_index];
        } while (!radio_current_transmit_entry->enabled);
    }

    radio_reset_transmit_delay_counter();
}

void radio_handle_timer_tick()
{
    if (!radio_module_initialized || radio_shared_state.radio_dma_transfer_active
        || radio_shared_state.radio_manual_transmit_active || radio_shared_state.radio_interrupt_transmit_active) {
        return;
    }

    if (radio_shared_state.radio_transmission_active) {
        if (radio_next_symbol_counter > 0) {
            radio_next_symbol_counter--;
        }

        if (radio_should_transmit_next_symbol()) {
            radio_shared_state.radio_transmit_next_symbol_flag = true;
            radio_reset_next_symbol_counter();
        }
    } else {
        if (radio_post_transmit_delay_counter > 0) {
            // TODO: use power saving
            radio_post_transmit_delay_counter--;
        }
    }
}

void radio_handle_data_timer_tick()
{
    radio_handle_data_timer_si4032();
}

bool radio_handle_time_sync()
{
    // TODO: How to poll GPS time?
    // TODO: Get time at millisecond precision!
    // ubxg6010_request_gpstime();

    gps_data gps;
    ubxg6010_get_current_gps_data(&gps);

    if (!gps.updated) {
        // The GPS data has not been updated yet
        return false;
    }

    if (false && gps.fix < 3) {
        log_info("Skipping transmit entry that requires time sync, no GPS fix");
        radio_next_transmit_entry();
        return false;
    }

    uint32_t time_millis = gps.time_of_week_millis;

    if (time_millis == radio_previous_time_sync_scheduled) {
        // The GPS chip has not provided an updated time yet for some reason
        return false;
    }

    uint32_t time_sync_offset_millis = radio_current_transmit_entry->time_sync_seconds_offset * 1000;

    if (time_millis < time_sync_offset_millis) {
        return false;
    }

    uint32_t time_sync_millis = radio_current_transmit_entry->time_sync_seconds * 1000;

    uint32_t time_with_offset_millis = time_millis - time_sync_offset_millis;
    uint32_t time_sync_period_millis = time_with_offset_millis % time_sync_millis;

    bool is_scheduled_time = time_sync_period_millis < RADIO_TIME_SYNC_THRESHOLD_MS;

    log_debug("Time with offset: %lu, sync period: %lu, scheduled: %d\n", time_with_offset_millis, time_sync_period_millis, is_scheduled_time);

    if (!is_scheduled_time) {
        log_info("Time: %lu, GPS fix: %d - Waiting for time sync at every %d seconds with offset of %d\n", time_millis,
                gps.fix,
                radio_current_transmit_entry->time_sync_seconds,
                radio_current_transmit_entry->time_sync_seconds_offset);
        return false;
    }

    log_info("Time: %lu, GPS fix: %d, sync period: %lu - Scheduling transmit at %d seconds with offset of %d\n",
            time_millis, gps.fix,
            time_sync_period_millis, radio_current_transmit_entry->time_sync_seconds,
            radio_current_transmit_entry->time_sync_seconds_offset);

    radio_previous_time_sync_scheduled = time_millis;

    return true;
}

void radio_handle_main_loop()
{
    bool time_sync_required = radio_current_transmit_entry->time_sync_seconds > 0;

    if (time_sync_required && !radio_shared_state.radio_transmission_active &&
        !radio_shared_state.radio_transmission_finished) {
        bool schedule_transmit = radio_handle_time_sync();
        if (!schedule_transmit) {
            // TODO: use power saving
            delay_ms(100);
            return;
        }

        radio_reset_transmit_delay_counter();
        radio_start_transmit_entry = radio_current_transmit_entry;
    } else if (!radio_shared_state.radio_transmission_active && radio_post_transmit_delay_counter == 0) {
        #if defined(SEMIHOSTING_ENABLE) && defined(LOGGING_ENABLE)
        telemetry_collect(&current_telemetry_data);
        log_info("Battery: %d mV\n", current_telemetry_data.battery_voltage_millivolts);
        log_info("Internal temperature: %ld C\n", current_telemetry_data.internal_temperature_celsius_100 / 100);
        log_info("Time: %02d:%02d:%02d\n",
                current_telemetry_data.gps.hours, current_telemetry_data.gps.minutes,
                current_telemetry_data.gps.seconds);
        log_info("Fix: %d, Sats: %d, OK packets: %d, Bad packets: %d\n",
                current_telemetry_data.gps.fix, current_telemetry_data.gps.satellites_visible,
                current_telemetry_data.gps.ok_packets, current_telemetry_data.gps.bad_packets);
        log_info("Lat: %ld *1M, Lon: %ld *1M, Alt: %ld m\n",
                current_telemetry_data.gps.latitude_degrees_1000000 / 10,
                current_telemetry_data.gps.longitude_degrees_1000000 / 10,
                (current_telemetry_data.gps.altitude_mm / 1000) * 3280 / 1000);
        #endif

        radio_reset_transmit_delay_counter();
        radio_start_transmit_entry = radio_current_transmit_entry;
    }

    radio_handle_main_loop_si4032(radio_current_transmit_entry, &radio_shared_state);

    bool first_symbol = false;
    if (radio_start_transmit_entry != NULL) {
        log_info("Start transmit\n");
        bool success = radio_start_transmit(radio_start_transmit_entry);
        start_tick = system_get_tick();

        radio_start_transmit_entry = NULL;
        if (!success) {
            radio_next_transmit_entry();
            return;
        }

        if (!radio_shared_state.radio_dma_transfer_active) {
            first_symbol = true;
            radio_shared_state.radio_transmit_next_symbol_flag = true;
        }
    }

    if (radio_shared_state.radio_transmission_active && radio_shared_state.radio_transmit_next_symbol_flag) {
        radio_shared_state.radio_transmit_next_symbol_flag = false;

        if (!radio_shared_state.radio_manual_transmit_active && !radio_shared_state.radio_interrupt_transmit_active) {
            bool success = radio_transmit_symbol(radio_current_transmit_entry);
            if (success) {
                if (first_symbol) {
                    radio_reset_next_symbol_counter();
                }
            } else {
                radio_shared_state.radio_transmission_finished = true;
            }
        }
    }

    if (radio_shared_state.radio_transmission_finished) {
        end_tick = system_get_tick();
        radio_stop_transmit(radio_current_transmit_entry);
        radio_shared_state.radio_transmission_finished = false;

        radio_next_transmit_entry();

        log_info("TX stop\n");
        log_info("Symbol count (interrupt): %ld\n", radio_shared_state.radio_symbol_count_interrupt);
        log_info("Symbol count (loop): %ld\n", radio_shared_state.radio_symbol_count_loop);
        log_info("Total ticks: %ld\n", end_tick - start_tick);
        log_info("Next symbol counter: %ld\n", radio_next_symbol_counter);
        log_info("Symbol rate: %ld\n", radio_shared_state.radio_current_symbol_rate);
        log_info("Symbol delay: %ld\n", radio_shared_state.radio_current_symbol_delay_ms_100);
        log_info("Tone spacing: %ld\n", radio_shared_state.radio_current_tone_spacing_hz_100);
    }
}

void radio_init()
{
    uint8_t count;
    for (count = 0; !radio_transmit_schedule[count].end; count++);
    radio_transmit_entry_count = count;

    memset(&current_telemetry_data, 0, sizeof(current_telemetry_data));

    for (uint8_t i = 0; i < radio_transmit_entry_count; i++) {
        radio_transmit_entry *entry = &radio_transmit_schedule[i];
        switch (entry->data_mode) {
            case RADIO_DATA_MODE_CW:
                entry->messages = cw_message_templates;
                break;
            case RADIO_DATA_MODE_APRS_1200:
                entry->messages = aprs_comment_templates;
                break;
            default:
                break;
        }
        if (entry->messages != NULL) {
            for (entry->message_count = 0; entry->messages[entry->message_count] != NULL; entry->message_count++);
        }
        if (entry->transmit_count == 0) {
            entry->transmit_count = 1;
        }
    }

    radio_current_transmit_entry = &radio_transmit_schedule[radio_current_transmit_entry_index];

    while (!radio_current_transmit_entry->enabled) {
        radio_current_transmit_entry_index = (radio_current_transmit_entry_index + 1) % radio_transmit_entry_count;
        radio_current_transmit_entry = &radio_transmit_schedule[radio_current_transmit_entry_index];
    }

    radio_init_si4032();

    radio_module_initialized = true;
}
