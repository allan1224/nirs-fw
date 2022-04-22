//
// Created by Allan Frederick on 10/3/21.
//

#include "ambientLight.h"
#include "driver/gpio.h"
#include <driver/adc.h>
#include <esp_event.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#define AMBIENT_SENSOR_TAG "AMBIENT_SENSOR"

static const adc2_channel_t photores_left_channel = ADC2_CHANNEL_8;
static const adc2_channel_t photores_right_channel = ADC2_CHANNEL_9;
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;

ambientLight::ambientLight(){
    adc2_config_channel_atten(photores_left_channel, atten);
    adc2_config_channel_atten(photores_right_channel, atten);

    setBaseAmbientValue();

    // Setup Timer
    const esp_timer_create_args_t lightCheck_timer_args = {
            .callback = &timer_callback_lightCheck,
            .name = "periodic_ambientLightCheck"
    };
    ESP_ERROR_CHECK(esp_timer_create(&lightCheck_timer_args, &TIMER_lightCheck));
    ESP_LOGI(AMBIENT_SENSOR_TAG, "AMBIENT LIGHT TIMER CREATED");

    // Start Timer
    startTimer();

}

void ambientLight::startTimer(){
    ESP_ERROR_CHECK(esp_timer_start_periodic(TIMER_lightCheck, 1000000)); // Check placement every second
    ESP_LOGI(AMBIENT_SENSOR_TAG, "AMBIENT LIGHT CHECK STARTED");
}

void ambientLight::stopTimer(){
    ESP_ERROR_CHECK(esp_timer_stop(TIMER_lightCheck));
    ESP_LOGI(AMBIENT_SENSOR_TAG, "AMBIENT LIGHT TIMER STOPPED");
}

static void timer_callback_lightCheck(void* arg){
    bool onHead = ambientLight::detectAmbientLight();
    ESP_ERROR_CHECK(esp_event_post(SENSOR_EVENT, AMBIENT_READ, &onHead, sizeof(onHead), portMAX_DELAY));
}

void ambientLight::setBaseAmbientValue() {
    adc2_get_raw(photores_left_channel, width, &ambient_left_base);
    ESP_LOGD(AMBIENT_SENSOR_TAG, "amb_left_base value: %d", ambient_left_base);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    adc2_get_raw(photores_right_channel, width, &ambient_right_base);
    ESP_LOGD(AMBIENT_SENSOR_TAG, "amb_right_base value: %d", ambient_right_base);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

bool ambientLight::detectAmbientLight(){
    bool onHead;

    adc2_get_raw(photores_left_channel, width, &ambient_left_current);
    //ESP_LOGI(AMBIENT_SENSOR_TAG, "amb_left_delta: %d", ambient_left_base - ambient_left_current);
    //ESP_LOGI(AMBIENT_SENSOR_TAG, "pass_threshold: %s", (ambient_left_base - ambient_left_current) > AMBIENT_THRESHOLD ? "yes" : "no");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    adc2_get_raw(photores_right_channel, width, &ambient_right_current);
    //ESP_LOGI(AMBIENT_SENSOR_TAG, "amb_right_delta: %d", ambient_right_base - ambient_right_current);
    //ESP_LOGI(AMBIENT_SENSOR_TAG, "pass_threshold: %s", (ambient_right_base - ambient_right_current) > AMBIENT_THRESHOLD ? "yes" : "no");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    if ((ambient_left_base - ambient_left_current > AMBIENT_THRESHOLD) && (ambient_right_base - ambient_right_current > AMBIENT_THRESHOLD)) {
        onHead = true;
    } else {
        onHead = false;
    }

    ESP_LOGI(AMBIENT_SENSOR_TAG, "on head: %s", onHead ? "yes" : "no");
    return onHead;
}