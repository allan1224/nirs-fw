#ifdef __cplusplus
extern "C" {
#endif

#ifndef COMPONENTS_T_TIMER_H
#define COMPONENTS_T_TIMER_H

#include <cstdio>
#include "driver/timer.h"
#include "esp_event_base.h"
#include "../../device-controls/include/device-controls.h"
#include "../../events/include/events.h"

#define T_TIMER_TAG "T_TIMER_MODULE"

// MARK: - HW_TIMER definitions
#define TIMER_DIVIDER (16)
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)
#define DEFAULT_DURATION_SEC 15

typedef struct {
    timer_group_t group;
    timer_idx_t timer;
    uint16_t duration;
} timer_info_t;

// MARK: - Class definition
class TreatmentTimer {
    public:
        TreatmentTimer(timer_group_t, timer_idx_t, uint16_t);
        static bool IRAM_ATTR treatment_done_isr_callback(void *args);
        static void bt_event_handler(void*, esp_event_base_t, int32_t, void*);
    private:
        timer_info_t* _info;
};
#endif /* COMPONENTS_T_TIMER_H */

#ifdef __cplusplus
}
#endif