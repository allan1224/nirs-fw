#include <cstdio>
#include "freertos/FreeRTOS.h"
#include "treatment-timer.h"
#include "esp_event.h"
#include "esp_log.h"

ESP_EVENT_DEFINE_BASE(TREATMENT_TIMER_EVENT);


bool TreatmentTimer::treatment_done_isr_callback(void *args) {
    ESP_DRAM_LOGI(T_TIMER_TAG, "Treatment done!");
    BaseType_t high_task_awoken = pdFALSE;

    auto timer_info = (timer_info_t *) args;

    timer_group_set_counter_enable_in_isr(timer_info->group, timer_info->timer, TIMER_PAUSE);
    DeviceControls::disable_treatment_lock();
    DeviceControls::set_led_enabled(false);
    esp_event_post(TREATMENT_SEQUENCE_EVENT, TREATMENT_END, args, sizeof(timer_info_t), 0);
    return high_task_awoken == pdTRUE;
}

TreatmentTimer::TreatmentTimer(timer_group_t group, timer_idx_t timer, uint16_t duration_sec) {
    _info = static_cast<timer_info_t *>(calloc(1, sizeof(timer_info_t)));
    _info->group = group;
    _info->timer = timer;
    _info->duration = duration_sec;

    timer_config_t config = {
            .alarm_en = TIMER_ALARM_EN,
            .counter_en = TIMER_PAUSE,
            .counter_dir = TIMER_COUNT_UP,
            .auto_reload = TIMER_AUTORELOAD_DIS,
            .divider = TIMER_DIVIDER
    };
    timer_init(group, timer, &config);

    timer_set_counter_value(group, timer, 0);

    timer_set_alarm_value(group, timer, duration_sec * TIMER_SCALE);
    timer_enable_intr(group, timer);

    timer_isr_callback_add(group, timer, treatment_done_isr_callback, _info, 0);

    esp_event_handler_register(TREATMENT_TIMER_EVENT, UPDATE_TIMER_STATE,
                               TreatmentTimer::bt_event_handler, _info);
}

void TreatmentTimer::bt_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    ESP_LOGI(T_TIMER_TAG, "Handling bluetooth write event");

    auto info = (timer_info_t *) event_handler_arg;
    auto config = static_cast<timer_config_t *>(calloc(1, sizeof(timer_config_t)));
    timer_get_config(info->group, info->timer, config);
    ESP_LOGD(T_TIMER_TAG, "Current State: %s, duration = %d", config->counter_en == TIMER_START ? "running" : "stopped", info->duration);

    if (event_id == UPDATE_TIMER_STATE) {
        auto new_state = (uint8_t *) event_data;
        bool new_timer_en = new_state[0] == 0x80;
        uint16_t new_duration_sec = (new_state[2] << 8) | new_state[3];
        ESP_LOGD(T_TIMER_TAG, "Target State: %s, duration = %d", new_timer_en ? "running" : "stopped", new_duration_sec);

        if (new_timer_en && new_timer_en != DeviceControls::treatment_is_running()) {
            ESP_ERROR_CHECK(esp_event_post(TREATMENT_SEQUENCE_EVENT, SEQUENCE_START, nullptr, 0, portMAX_DELAY));
        }

        if (new_duration_sec != info->duration) {
            timer_set_counter_value(info->group, info->timer, 0);
            timer_set_alarm_value(info->group, info->timer, new_duration_sec * TIMER_SCALE);
        }

        timer_get_config(info->group, info->timer, config);
        ESP_LOGI(T_TIMER_TAG, "Timer updated!");
        ESP_LOGD(T_TIMER_TAG, "Updated State: %s, duration = %d", config->counter_en == TIMER_START ? "running" : "stopped", info->duration);
    }
}
