#ifdef __cplusplus
extern "C" {
#endif

#include <cstdio>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"

#ifndef COMPONENTS_BLUETOOTH_H
#define COMPONENTS_BLUETOOTH_H

#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

#define NIRS_PROFILE_NUM 2

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

class BluetoothController {
    public:
        BluetoothController();
        static void gatts_profile_event_handler(esp_gatts_cb_event_t, esp_gatt_if_t, esp_ble_gatts_cb_param_t *);
    private:
        static void plxs_event_handler(void*, esp_event_base_t, int32_t, void*);
        static void timer_event_handler(void*, esp_event_base_t, int32_t, void*);
        static void ambient_event_handler(void*, esp_event_base_t , int32_t , void *);
        static void temp_event_handler(void*, esp_event_base_t , int32_t , void *);
        static bool validate_timer_write(const uint8_t*, uint8_t);
        static const char *handle_to_str(uint16_t);
        static const char *gatts_status_to_name(esp_gatt_status_t);
};

#endif /* COMPONENTS_BLUETOOTH_H */

#ifdef __cplusplus
}
#endif