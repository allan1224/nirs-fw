#include "esp_log.h"
#include "oximeter-sensors.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include <driver/adc.h>
#include "esp_event.h"
#include "../device-controls/include/device-controls.h"
#include "../treatment-timer/include/treatment-timer.h"

static const gpio_num_t sensor = GPIO_NUM_36;
static const gpio_num_t redLED = GPIO_NUM_16;
static const gpio_num_t irLED = GPIO_NUM_17;

ESP_EVENT_DEFINE_BASE(SENSOR_EVENT);
ESP_EVENT_DEFINE_BASE(TREATMENT_SEQUENCE_EVENT);

OximeterSensors::OximeterSensors() {
    ESP_LOGI(SPO2_SENSOR_TAG, "Initialize Oximeter Sensors...");
    // Reset pins
    gpio_reset_pin(sensor);
    gpio_reset_pin(redLED);
    gpio_reset_pin(irLED);
    // Set pin direction
    gpio_set_direction(sensor, GPIO_MODE_INPUT);
    gpio_set_direction(redLED, GPIO_MODE_OUTPUT);
    gpio_set_direction(irLED, GPIO_MODE_OUTPUT);
    // Setup ADC
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_0db);
    // Initialize values for signal processing
    T = 20;period = 0;samples = 0;
    for (int i = 0; i < MAX_SAMPLES_PER_PD; i++) {
        readsIRMM[i] = 0;
        readsREDMM[i] = 0;
    }
    for (int i = 0; i < STORED_PERIODS; i++) {
        measuresPeriods[i] = 0;
        measuresR[i] = 0;
    }
    for (int i = 0; i < SAMPLE_SIZE; i++) {
        readsIR[i] = 0;
        readsRED[i] = 0;
    }
    sumIR = 0;sumRED = 0;ptr = 0;samplesCounter = 0;ptrMM = 0;IRmax = 0;IRmin = 0;REDmax = 0;REDmin = 0;R = 0;m = 0;
    finger_status = true;rCalculated = false;

    // FOR START-TREATMENT-SEQUENCE READINGS ("big timer")
    const esp_timer_create_args_t oneShot_timer_args = {
            .callback = &timer_callback_setReadStatus,
            /* name is optional, but may help identify the timer when debugging */
            .name = "One shot timer Oximeter"
    };
    ESP_ERROR_CHECK(esp_timer_create(&oneShot_timer_args, &TIMER_Reading));
}

void OximeterSensors::getReadHandler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data) {
    ESP_LOGI(SPO2_SENSOR_TAG, "Handling SPO2 sensor event...");
    // turn off LED array
    DeviceControls::set_led_enabled(false);
    ESP_LOGI(SPO2_SENSOR_TAG, "TURN OFF LED");
    // turn off ambient light timer
    ambientLight::stopTimer();
    ESP_LOGI(SPO2_SENSOR_TAG, "TURN OFF AMBIENT TIMER");
    // get SpO2 readings
    esp_timer_start_once(TIMER_Reading, 10000000); // 10 seconds big timer
    ESP_LOGI(SPO2_SENSOR_TAG, "START SPO2 TIMER");
    // keeps calculating SPO2 values and posting to sensor-read event until 10 seconds timer is up
    calculate(sensor);
    ESP_LOGI(SPO2_SENSOR_TAG, "END SPO2 TIMER");
    if (id == SEQUENCE_START){
        // Post stimulate treat-sequence event to treatmentSequence handler
        ESP_ERROR_CHECK(esp_event_post(TREATMENT_SEQUENCE_EVENT, TREATMENT_START, nullptr, 0, portMAX_DELAY));
        ESP_LOGI(SPO2_SENSOR_TAG, "POSTED STIMULATE-EVENT");
    }
    else if (id == TREATMENT_END){
        // Post complete treat-sequence event to treatmentSequence handler
        ESP_ERROR_CHECK(esp_event_post(TREATMENT_SEQUENCE_EVENT, SEQUENCE_COMPLETE, event_data, sizeof(timer_info_t), portMAX_DELAY));
        ESP_LOGI(SPO2_SENSOR_TAG, "POSTED COMPLETE-EVENT");
    }
}

static void timer_callback_setReadStatus(void* arg) {
    oximterTimerStatus = false;
}

void OximeterSensors::calculate(gpio_num_t gpio_num) {
    ESP_LOGI(SPO2_SENSOR_TAG, "Calculating SPO2 percentage...");

    if (gpio_num == sensor){
        REDLed = redLED;
        IRLed = irLED;
        sensorPin = sensor;
    }

    while(oximterTimerStatus){
        // turn on IR LED
        gpio_set_level(REDLed, 0);
        gpio_set_level(IRLed, 1);

        // calculate an average of the sensor
        // during a 20 ms (T) period (this will eliminate
        // the 50 Hz noise caused by electric light
        n = 0;
        // start = millis();
        start = getTimeSinceStart();
        reader = 0;
        do
        {
            reader += adc1_get_raw(ADC1_CHANNEL_0);
            n++;
        }
        while (getTimeSinceStart() < start + T);
        reader /= n;  // we got an average
        // Add the newest measurement to an array
        // and subtract the oldest measurement from the array
        // to maintain a sum of last measurements
        sumIR -= readsIR[ptr];
        sumIR += reader;
        readsIR[ptr] = reader;
        lastIR = sumIR / SAMPLE_SIZE;

        // TURN ON RED LED and do the same
        gpio_set_level(REDLed, 1);
        gpio_set_level(IRLed, 0);

        n = 0;
        start = getTimeSinceStart();
        reader = 0.;
        do
        {
            reader += adc1_get_raw(ADC1_CHANNEL_0);
            n++;
        }
        while (getTimeSinceStart() < start + T);
        reader /= n;  // we got an average
        // Add the newest measurement to an array
        // and subtract the oldest measurement from the array
        // to maintain a sum of last measurements
        sumRED -= readsRED[ptr];
        sumRED += reader;
        readsRED[ptr] = reader;
        lastRED = sumRED / SAMPLE_SIZE;

        //
        // R CALCULATION
        // save all the samples of a period both for IR and for RED
        readsIRMM[ptrMM]=lastIR;
        readsREDMM[ptrMM]=lastRED;
        ptrMM++;
        ptrMM %= MAX_SAMPLES_PER_PD;
        samplesCounter++;
        //
        // if I've saved all the samples of a period, look to find
        // max and min values and calculate R parameter
        if(samplesCounter>=samples){
            samplesCounter =0;
            IRmax = 0; IRmin=1023; REDmax = 0; REDmin=1023;
            for(int i=0; i < MAX_SAMPLES_PER_PD; i++) {
                if( readsIRMM[i]> IRmax) IRmax = readsIRMM[i];
                if( readsIRMM[i]>0 && readsIRMM[i]< IRmin ) IRmin = readsIRMM[i];
                readsIRMM[i] =0;
                if( readsREDMM[i]> REDmax) REDmax = readsREDMM[i];
                if( readsREDMM[i]>0 && readsREDMM[i]< REDmin ) REDmin = readsREDMM[i];
                readsREDMM[i] =0;
            }
            R =  ( (REDmax-REDmin) / REDmin) / ( (IRmax-IRmin) / IRmin ) ;

        }


        float avR = 0;
        int avBPM=0;



        if (finger_status==true){

            // lastIR holds the average of the values in the array
            // check for a rising curve (= a heart beat)
            if (lastIR > beforeIR)
            {

                //printf("lastIR is > beforeIR");

                // Flag set here
                //rCalculated = true;


                rise_count++;  // count the number of samples that are rising
                if (!rising && rise_count > RISING_PD_THRESHOLD)
                {
                    // <3
                    // Ok, we have detected a rising curve, which implies a heartbeat.
                    // Record the time since last beat, keep track of the 10 previous
                    // peaks to get an average value.
                    // The rising flag prevents us from detecting the same rise
                    // more than once.
                    rising = true;

                    measuresR[m] = R;
                    measuresPeriods[m] = getTimeSinceStart() - last_beat;
                    last_beat = getTimeSinceStart();
                    int period = 0;
                    for(int i =0; i < STORED_PERIODS; i++) period += measuresPeriods[i];

                    // calculate average period and number of samples
                    // to store to find min and max values
                    period = period / STORED_PERIODS;
                    samples = period / (2*T);

                    int avPeriod = 0;

                    int c = 0;

                    // c stores the number of good STORED_PERIODS (not floating more than 10%),
                    // in the last 10 peaks
                    for(int i =1; i < STORED_PERIODS; i++) {
                        if ( (measuresPeriods[i] <  measuresPeriods[i-1] * 1.1)  &&
                             (measuresPeriods[i] >  measuresPeriods[i-1] / 1.1)  ) {

                            c++;
                            //printf("c: %02d\n", c);
                            avPeriod += measuresPeriods[i];
                            avR += measuresR[i];

                        }
                    }

                    m++;
                    m %= STORED_PERIODS;


                    // bpm and R shown are calculated as the
                    // average of at least 5 good peaks
                    avBPM = 60000 / ( avPeriod / c) ;
                    avR = avR / c ;

                    // if there are at least 1 good STORED_PERIODS...

                    if(c > 1) {

                        //
                        // SATURTION IS A FUNCTION OF R (calibration)
                        // Y = k*x + m
                        // k and m are calculated with another oximeter
                        SpO2 = -19 * R + 112;
                        //printf("SpO2: %02f\n", SpO2);



                    }
                }
            }

            else
            {
                // Ok, the curve is falling
                rising = false;
                rise_count = 0;
            }

            // to compare it with the new value and find peaks
            beforeIR = lastIR;


        } // finger is inside

        // handle the arrays
        ptr++;
        ptr %= SAMPLE_SIZE;

        //printf("SPO2: %02f\n", SpO2);
        //printf("IR: %02f\n", lastIR);
        //printf("R: %02f\n", R);
        ESP_LOGI(SPO2_SENSOR_TAG, "New R Value: %f", R);
        ESP_LOGI(SPO2_SENSOR_TAG, "New SPO2 Value: %f", SpO2);
    }

    ESP_ERROR_CHECK(esp_event_post(SENSOR_EVENT, SPO2_READ, &SpO2, sizeof(SpO2), portMAX_DELAY));
    ESP_LOGI(SPO2_SENSOR_TAG, "POSTED TO LOOP: %f", SpO2);
    gpio_set_level(REDLed, 0);
    gpio_set_level(IRLed, 0);
}

/**
 * Get the time in milliseconds since the %FreeRTOS scheduler started.
 * @return The time in milliseconds since the %FreeRTOS scheduler started.
 */
uint32_t OximeterSensors::getTimeSinceStart() {
    return (uint32_t) (xTaskGetTickCount() * portTICK_PERIOD_MS);
}



/*
void OximeterSensors::startReads(){
    ESP_ERROR_CHECK(esp_timer_start_periodic(TIMER_spo2Check, 100000));
}
void OximeterSensors::stopReads(){
    ESP_ERROR_CHECK(esp_timer_stop(TIMER_spo2Check));
}

*/

/*
    // FOR PERIODICALLY READING SPO2 VALUES ("little timer")
    // start timer that auto reloads
    // calc function post event to loop
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &timer_callback_spo2,
            /name is optional, but may help identify the timer when debugging
            .name = "Oximeter SW Timer"
    };
    */


// ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &TIMER_spo2Check));

/*
static void timer_callback_spo2(void* arg) {
        while (oximterTimerStatus == true){
            rCalculated = false;
            double new_SpO2 = OximeterSensors::calculate(sensor);
            ESP_LOGI(SPO2_SENSOR_TAG, "New SPO2 Value: %f", new_SpO2);
            ESP_ERROR_CHECK(esp_event_post(SENSOR_EVENT, NEW_SPO2_READ, &new_SpO2, sizeof(new_SpO2), portMAX_DELAY));
        }
}
*/