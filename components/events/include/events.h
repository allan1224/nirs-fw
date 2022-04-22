#ifdef __cplusplus
extern "C" {
#endif

#ifndef GLOBAL_EVENTS_H
#define GLOBAL_EVENTS_H

#include "esp_event.h"

ESP_EVENT_DECLARE_BASE(DEVCTRL_EVENT);
enum devctrl_event_id_t {
    UPDATE_LED_STATE
};

ESP_EVENT_DECLARE_BASE(TREATMENT_SEQUENCE_EVENT);
enum treatment_sequence_event_id_t {
    SEQUENCE_START, // START
    TREATMENT_START, // STIMULATE
    TREATMENT_END, // END_READ
    SEQUENCE_COMPLETE // COMPLETE
};

ESP_EVENT_DECLARE_BASE(SENSOR_EVENT);
enum sensor_event_id_t {
    SPO2_READ,
    AMBIENT_READ,
    TEMPERATURE_READ
};

ESP_EVENT_DECLARE_BASE(TREATMENT_TIMER_EVENT);
enum treatment_timer_event_t {
    UPDATE_TIMER_STATE,
};

#endif //GLOBAL_EVENTS_H

#ifdef __cplusplus
}
#endif