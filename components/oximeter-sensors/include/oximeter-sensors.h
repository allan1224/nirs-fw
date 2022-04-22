#ifdef __cplusplus
extern "C" {
#endif

#ifndef COMPONENTS_SENSORS_H
#define COMPONENTS_SENSORS_H

#include <cstdio>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "../../ambient-sensors/include/ambientLight.h"
#include "../../events/include/events.h"

#define SPO2_SENSOR_TAG "SPO2_MODULE"

// Start treat sequence event -> move to event source file
static esp_event_loop_handle_t treatmentSequenceStart;
static esp_event_loop_handle_t treatmentSequenceStimulate;

static void timer_callback_spo2(void* arg);
static esp_timer_handle_t TIMER_spo2Check;

static void timer_callback_setReadStatus(void* arg);
static esp_timer_handle_t TIMER_Reading;

// SpO2 constants
#define MAX_SAMPLES_PER_PD  80
#define STORED_PERIODS      10
#define SAMPLE_SIZE         4
#define RISING_PD_THRESHOLD 3 // number of rising STORED_PERIODS to determine a peak

class OximeterSensors {
    public:
        OximeterSensors();
        double getSpLevels();
        static void calculate(gpio_num_t gpio_num);
        static uint32_t getTimeSinceStart();
        void blink(); // for testing
        static void startReads();
        static void stopReads();
        static void getReadHandler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
};

static bool oximterTimerStatus = true;
static int T; // slot milliseconds to read a value from the sensor
static float readsIR[SAMPLE_SIZE], sumIR,lastIR, reader, start;
static float readsRED[SAMPLE_SIZE], sumRED,lastRED;
static int period, samples;
static int samplesCounter;
static float readsIRMM[MAX_SAMPLES_PER_PD],readsREDMM[MAX_SAMPLES_PER_PD];
static int ptrMM;
static float IRmax;
static float IRmin;
static float REDmax;
static float REDmin;
static double R;
static float measuresR[STORED_PERIODS];
static int measuresPeriods[STORED_PERIODS];
static int m;
static int ptr;
static float beforeIR;
static bool rising;
static int rise_count;
static int n;
static long int last_beat;
static bool finger_status;
static double SpO2;
static bool rCalculated;
static gpio_num_t REDLed;
static gpio_num_t IRLed;
static gpio_num_t sensorPin;


#endif /* COMPONENTS_SENSORS_H */

#ifdef __cplusplus
}
#endif