//
// Created by Allan Frederick on 10/3/21.
//

#include "tempSense.h"
#include "driver/gpio.h"
#include <driver/adc.h>
#include <esp_event.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include <cmath>

#define TEMP_TAG "TEMP_SENSOR"

static const adc1_channel_t TEMP_SENSOR_ADC_CHANNEL = ADC1_CHANNEL_7;
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;

tempSense::tempSense(){
    gpio_num_t temp_sensor_pin;
    adc1_pad_get_io_num(TEMP_SENSOR_ADC_CHANNEL, &temp_sensor_pin);

    gpio_reset_pin(temp_sensor_pin);
    gpio_set_direction(temp_sensor_pin, GPIO_MODE_INPUT);

    adc1_config_width(width);
    adc1_config_channel_atten(TEMP_SENSOR_ADC_CHANNEL, atten);

    // temperature detector timer setup
    const esp_timer_create_args_t tempCheck_timer_args = {
            .callback = &timer_callback_tempCheck,
            .name = "periodic_tempCheck"
    };
    ESP_ERROR_CHECK(esp_timer_create(&tempCheck_timer_args, &TIMER_tempCheck));
    ESP_ERROR_CHECK(esp_timer_start_periodic(TIMER_tempCheck, 2000000)); //  Check temperature every second
}

static void timer_callback_tempCheck(void* arg){
    uint32_t temp_Vout = adc1_get_raw(TEMP_SENSOR_ADC_CHANNEL);
    int temp = tempSense::voltageToCelsius(temp_Vout);
    ESP_LOGI(TEMP_TAG, "Raw value: %d, Temp value: %d°C", temp_Vout, temp);
    ESP_ERROR_CHECK(esp_event_post(SENSOR_EVENT, TEMPERATURE_READ, &temp, sizeof(temp), portMAX_DELAY));
}

int tempSense::voltageToCelsius(uint32_t voltage) {
    double adcMax = 4095.0;   // ADC resolution 12-bit (0-4095)
    int Vs = 3.3;          // supply voltage
    double R1 = 100000.0;   // voltage divider resistor value
    double Beta = 4103.69;  // Beta value
    double To = 298.15;    // Temperature in Kelvin for 25 degree Celsius
    double Ro = 100000.0;   // Resistance of Thermistor at 25 degree Celsius
    double Rt, T, Tc, Tf = 0;

    double Vout = voltage * Vs/adcMax; // convert ADC voltage to analog voltage
    Rt = R1 * Vout / (Vs - Vout); // thermistor resistance
    T = 1/(1/To + log(Rt/Ro)/Beta);  // calculate temp based on Steinhart-Hart model (in Kelvin)
    Tc = T - 273.15;                 // Celsius

    return Tc;
}
