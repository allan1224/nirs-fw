#include "esp_log.h"
#include "esp_event.h"
#include "device-controls.h"
#include "../ambient-sensors/include/ambientLight.h"

bool DeviceControls::led_enabled = false;
bool DeviceControls::zones[ZONES_IDX_NB] = {
        [LEFT_ZONE_IDX] = true,
        [RIGHT_ZONE_IDX] = true,
};

gpio_num_t DeviceControls::right_zone_pin = GPIO_NUM_21;
gpio_num_t DeviceControls::left_zone_pin = GPIO_NUM_27;
bool DeviceControls::treatment_ongoing = false;

ESP_EVENT_DEFINE_BASE(DEVCTRL_EVENT);

DeviceControls::DeviceControls() {
    ESP_LOGI(DEVCTRL_TAG, "Initialize Device Controls...");

    // LED setup
    gpio_config_t led_conf;
    led_conf.mode = GPIO_MODE_OUTPUT;
    led_conf.intr_type = GPIO_INTR_DISABLE;
    led_conf.pin_bit_mask = ((1ULL<<left_zone_pin) | (1ULL<<right_zone_pin));
    led_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    led_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&led_conf);

    update_led_state();

    esp_event_handler_register(DEVCTRL_EVENT, UPDATE_LED_STATE,
                                        DeviceControls::bt_event_handler, nullptr);
}

void DeviceControls::set_zone_state(zones_t zone_idx, bool state) {
    if (!treatment_ongoing) {
        zones[zone_idx] = state;
        update_led_state();
    }
}

void DeviceControls::set_led_enabled(bool state) {
    if (!treatment_ongoing) {
        led_enabled = state;
        update_led_state();
    }
}

void DeviceControls::update_led_state() {
    if (!led_enabled) {
        gpio_set_level(left_zone_pin, 0);
        gpio_set_level(right_zone_pin, 0);
    } else {
        for (int zone_idx = 0; zone_idx < ZONES_IDX_NB; zone_idx++){
            if (zone_idx == LEFT_ZONE_IDX) {
                gpio_set_level(left_zone_pin, zones[zone_idx]);
            } else if (zone_idx == RIGHT_ZONE_IDX) {
                gpio_set_level(right_zone_pin, zones[zone_idx]);
            }
        }
    }
}

void DeviceControls::bt_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    ESP_LOGI(DEVCTRL_TAG, "Handling bluetooth event");

    if (event_id == UPDATE_LED_STATE) {
        auto data = (uint8_t *) event_data;
        uint8_t led_state_value = data[0];

        ESP_LOGD(DEVCTRL_TAG, "New value : %d", led_state_value);

        switch(led_state_value) {
            case 1:
                set_led_enabled(true);
                set_zone_state(LEFT_ZONE_IDX, false);
                set_zone_state(RIGHT_ZONE_IDX, false);
                break;
            case 2:
                set_led_enabled(false);
                set_zone_state(LEFT_ZONE_IDX, true);
                set_zone_state(RIGHT_ZONE_IDX, false);
                break;
            case 3:
                set_led_enabled(true);
                set_zone_state(LEFT_ZONE_IDX, true);
                set_zone_state(RIGHT_ZONE_IDX, false);
                break;
            case 4:
                set_led_enabled(false);
                set_zone_state(LEFT_ZONE_IDX, false);
                set_zone_state(RIGHT_ZONE_IDX, true);
                break;
            case 5:
                set_led_enabled(true);
                set_zone_state(LEFT_ZONE_IDX, false);
                set_zone_state(RIGHT_ZONE_IDX, true);
                break;
            case 6:
                set_led_enabled(false);
                set_zone_state(LEFT_ZONE_IDX, true);
                set_zone_state(RIGHT_ZONE_IDX, true);
                break;
            case 7:
                set_led_enabled(true);
                set_zone_state(LEFT_ZONE_IDX, true);
                set_zone_state(RIGHT_ZONE_IDX, true);
                break;
            default:
                set_led_enabled(false);
                set_zone_state(LEFT_ZONE_IDX, false);
                set_zone_state(RIGHT_ZONE_IDX, false);
                break;
        }
        ESP_LOGI(DEVCTRL_TAG, "LED On [%s], Zones [Left: %s, Right: %s]", led_enabled ? "true" : "false",
                 zones[LEFT_ZONE_IDX] ? "On" : "Off", zones[RIGHT_ZONE_IDX] ? "On" : "Off");
        update_led_state();
    }
}
