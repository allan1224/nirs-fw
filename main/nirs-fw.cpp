#include <cstdlib>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "bluetooth.h"
#include "device-controls.h"
#include "oximeter-sensors.h"
#include "treatment-timer.h"
#include "ambientLight.h"
#include "tempSense.h"
#include "events.h"

#define MAIN_TAG "NIRS_FW"


void Stimulate(void* handler_args, esp_event_base_t base, int32_t id, void* event_data);

void Complete(void* handler_args, esp_event_base_t base, int32_t id, void* event_data);

extern "C" {
    void app_main();
}

void app_main(void) {
    ESP_LOGI(MAIN_TAG, "Starting device!");

    //////////////// Shared Initialization /////////////////
    esp_err_t ret_val;
    ret_val = nvs_flash_init();
    if (ret_val == ESP_ERR_NVS_NO_FREE_PAGES || ret_val == ESP_ERR_NVS_NEW_VERSION_FOUND){
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret_val = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret_val);

    esp_event_loop_create_default();

    /////////////// Component Initialization ///////////////
    DeviceControls dev_ctrl;

    BluetoothController bluetooth_component;

    // Initialization does NOT start the periodic timer
    ambientLight amb;

    // Initialization does start the periodic timer
    tempSense temp;

    // Initialization does NOT start the periodic timer
    OximeterSensors ox_sense;

    TreatmentTimer timer = TreatmentTimer(TIMER_GROUP_0, TIMER_0, DEFAULT_DURATION_SEC);

    ////////////////// Treatment Sequence ///////////////////
    ESP_EVENT_DEFINE_BASE(TREAT_SEQ_EVENT);

    // When START TREAT_SEQ_EVENT is posted to treatmentSequence event loop -> start the treatment sequence
    ESP_ERROR_CHECK(esp_event_handler_register(TREATMENT_SEQUENCE_EVENT, SEQUENCE_START, OximeterSensors::getReadHandler, nullptr));
    ESP_LOGI(MAIN_TAG, "TREATMENT START REGISTERED");

    // When STIMULATE TREAT_SEQ_EVENT is posted to treatmentSequence event loop -> run stimulate function
    ESP_ERROR_CHECK(esp_event_handler_register(TREATMENT_SEQUENCE_EVENT, TREATMENT_START, Stimulate, nullptr));
    ESP_LOGI(MAIN_TAG, "TREATMENT STIMULATE REGISTERED");

    // When END_READ TREAT_SEQ_EVENT is posted to treatmentSequence event loop -> run complete function
    ESP_ERROR_CHECK(esp_event_handler_register(TREATMENT_SEQUENCE_EVENT, TREATMENT_END, OximeterSensors::getReadHandler, nullptr));
    ESP_LOGI(MAIN_TAG, "TREATMENT END_READ REGISTERED");

    // When COMPLETE TREAT_SEQ_EVENT is posted to treatmentSequence event loop -> run stimulate function
    ESP_ERROR_CHECK(esp_event_handler_register(TREATMENT_SEQUENCE_EVENT, SEQUENCE_COMPLETE, Complete, nullptr));
    ESP_LOGI(MAIN_TAG, "TREATMENT COMPLETE REGISTERED");
}

void Stimulate(void* handler_args, esp_event_base_t base, int32_t id, void* event_data){
    // turn on photoresistor timers
    ambientLight::startTimer();
    // turn on LED array
    DeviceControls::set_led_enabled(true);
    // turn on treatment timer
    DeviceControls::enable_treatment_lock();
    timer_start(TIMER_GROUP_0, TIMER_0);
}

void Complete(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    // turn on photoresistor timers
    ambientLight::startTimer();
    // turn off LED array
    DeviceControls::disable_treatment_lock();
    DeviceControls::set_led_enabled(false);
    // reset treatment timer here
    auto timer_info = (timer_info_t *) event_data;
    timer_set_counter_value(timer_info->group, timer_info->timer, 0);
    timer_set_alarm_value(timer_info->group, timer_info->timer, timer_info->duration * TIMER_SCALE);
}






