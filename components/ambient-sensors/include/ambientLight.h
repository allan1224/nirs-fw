//
// Created by Allan Frederick on 10/3/21.
//

#ifdef __cplusplus
extern "C" {
#endif

#ifndef NIRS_FW_AMBIENTLIGHT_H
#define NIRS_FW_AMBIENTLIGHT_H
    
#include <esp_timer.h>
#include "esp_event.h"
#include "../../oximeter-sensors/include/oximeter-sensors.h"

static void timer_callback_lightCheck(void* arg);
static esp_timer_handle_t TIMER_lightCheck;

class ambientLight{
    public:
        ambientLight();
        static bool detectAmbientLight();
        static void setBaseAmbientValue();
        static void startTimer();
        static void stopTimer();

};

static int ambient_left_base;
static int ambient_left_current;
static int ambient_right_base;
static int ambient_right_current;



#define AMBIENT_THRESHOLD 1000

#endif //NIRS_FW_AMBIENTLIGHT_H

#ifdef __cplusplus
}
#endif