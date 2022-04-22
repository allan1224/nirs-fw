#ifdef __cplusplus
extern "C" {
#endif

#ifndef COMPONENTS_DEVICE_CTRL_H
#define COMPONENTS_DEVICE_CTRL_H

#include <cstdio>
#include "driver/gpio.h"
#include "esp_event_base.h"
#include <driver/adc.h>
#include "../../events/include/events.h"

#define DEVCTRL_TAG "DEVCTRL_MODULE"

enum zones_t {
    LEFT_ZONE_IDX,
    RIGHT_ZONE_IDX,

    ZONES_IDX_NB,
};

class DeviceControls {
    public:
        DeviceControls();
        static void set_zone_state(zones_t, bool);
        static void set_led_enabled(bool);
        static void bt_event_handler(void*, esp_event_base_t, int32_t, void*);
        static void inline enable_treatment_lock() { treatment_ongoing = true; }
        static void inline disable_treatment_lock() { treatment_ongoing = false; }
        static bool inline treatment_is_running() { return treatment_ongoing; }

private:
        static bool led_enabled;
        static bool zones[ZONES_IDX_NB];
        static gpio_num_t left_zone_pin;
        static gpio_num_t right_zone_pin;
        static void update_led_state();
        static bool treatment_ongoing;
};

#endif /* COMPONENTS_DEVICE_CTRL_H */

#ifdef __cplusplus
}
#endif