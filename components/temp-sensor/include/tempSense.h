//
// Created by Allan Frederick on 10/3/21.
//

#ifdef __cplusplus
extern "C" {
#endif

#ifndef NIRS_FW_TEMPSENSE_H
#define NIRS_FW_TEMPSENSE_H

#include <esp_timer.h>
#include "esp_event.h"
#include "../../oximeter-sensors/include/oximeter-sensors.h"

static void timer_callback_tempCheck(void* arg);
static esp_timer_handle_t TIMER_tempCheck;

class tempSense {
public:
        tempSense();
        static int voltageToCelsius(uint32_t);
};

#endif //NIRS_FW_TEMPSENSE_H

#ifdef __cplusplus
}
#endif
