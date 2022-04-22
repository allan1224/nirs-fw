#ifdef __cplusplus
extern "C" {
#endif

#include <cstdio>
#include "bluetooth.h"

#ifndef BT_DEVCTRL_H
#define BT_DEVCTRL_H

#define DEVCTRL_APP_IDX 0
#define DEVCTRL_APP_ID 0x42
#define DEVCTRL_SVC_INST_ID 0

static const uint16_t DEVCTRL_SERVICE_UUID[8] = {0xEFC9, 0xA3A3, 0xAA8E, 0x95EC, 0x46DC, 0x57B2, 0x2C80, 0xD2A6};
static const uint16_t DEVCTRL_LED_STAT_CHAR_UUID[8] = {0xEFC9, 0xA3A3, 0xAA8E, 0x95EC, 0x46DC, 0x57B2, 0x2C81, 0xD2A6};
static const uint16_t DEVCTRL_TIMER_CHAR_UUID[8] = {0xEFC9, 0xA3A3, 0xAA8E, 0x95EC, 0x46DC, 0x57B2, 0x2C82, 0xD2A6};
static const uint16_t DEVCTRL_PLACEMENT_CHAR_UUID[8] = {0xEFC9, 0xA3A3, 0xAA8E, 0x95EC, 0x46DC, 0x57B2, 0x2C83, 0xD2A6};
static const uint16_t DEVCTRL_TEMP_CHAR_UUID[8] = {0xEFC9, 0xA3A3, 0xAA8E, 0x95EC, 0x46DC, 0x57B2, 0x2C84, 0xD2A6};

static uint8_t devctrl_led_stat_val = 0x06;
static const uint8_t devctrl_led_stat_ccc[2] = {0x00, 0x00};

static uint8_t devctrl_timer_val[4] = {0x00, 0x00, 0x00, 0x0F};
static const uint8_t devctrl_timer_ccc[2] = {0x00, 0x00};

static uint8_t devctrl_placement_val = 0x00;
static const uint8_t devctrl_placement_ccc[2] = {0x00, 0x00};

static uint8_t devctrl_temp_val[4] = {0x0F, 0xFF, 0x0F, 0xFF};
static const uint8_t devctrl_temp_ccc[2] = {0x00, 0x00};

enum devctrl_svc_table_e {
    // Device Control Service Index
    DEVCTRL_SVC_IDX,

    // LED Array Status Characteristic, Value, and CCCD Config Indexes
    DEVCTRL_SVC_LED_STAT_CHAR_IDX,
    DEVCTRL_SVC_LED_STAT_VAL_IDX,
    DEVCTRL_SVC_LED_STAT_NTF_CFG_IDX,

    // Timer Characteristic, Value, and CCCD Config Indexes
    DEVCTRL_SVC_TIMER_CHAR_IDX,
    DEVCTRL_SVC_TIMER_VAL_IDX,
    DEVCTRL_SVC_TIMER_NTF_CFG_IDX,

    // Placement Characteristic, Value, and CCCD Config Indexes
    DEVCTRL_SVC_PLACEMENT_CHAR_IDX,
    DEVCTRL_SVC_PLACEMENT_VAL_IDX,
    DEVCTRL_SVC_PLACEMENT_NTF_CFG_IDX,

    // Temperature Characteristic, Value, and CCCD Config Indexes
    DEVCTRL_SVC_TEMP_CHAR_IDX,
    DEVCTRL_SVC_TEMP_VAL_IDX,
    DEVCTRL_SVC_TEMP_NTF_CFG_IDX,

    DEVCTRL_SVC_IDX_NB,
};

static const esp_gatts_attr_db_t devctrl_gatt_db[DEVCTRL_SVC_IDX_NB] = {
        [DEVCTRL_SVC_IDX] = {
                {ESP_GATT_AUTO_RSP},
                {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
                 sizeof(uint16_t[8]), sizeof(DEVCTRL_SERVICE_UUID), (uint8_t *)&DEVCTRL_SERVICE_UUID}
        },
        [DEVCTRL_SVC_LED_STAT_CHAR_IDX] = {
                {ESP_GATT_AUTO_RSP},
                {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}
        },
        [DEVCTRL_SVC_LED_STAT_VAL_IDX] = {
                {ESP_GATT_AUTO_RSP},
                {ESP_UUID_LEN_128, (uint8_t *)&DEVCTRL_LED_STAT_CHAR_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                        sizeof(uint8_t), sizeof(devctrl_led_stat_val), (uint8_t *)&devctrl_led_stat_val}
        },
        [DEVCTRL_SVC_LED_STAT_NTF_CFG_IDX] = {
                {ESP_GATT_AUTO_RSP},
                {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                        sizeof(uint16_t), sizeof(devctrl_led_stat_ccc), (uint8_t *)devctrl_led_stat_ccc}
        },
        [DEVCTRL_SVC_TIMER_CHAR_IDX] = {
                {ESP_GATT_AUTO_RSP},
                {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}
        },
        [DEVCTRL_SVC_TIMER_VAL_IDX] = {
                {ESP_GATT_AUTO_RSP},
                {ESP_UUID_LEN_128, (uint8_t *)&DEVCTRL_TIMER_CHAR_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                        sizeof(uint32_t), sizeof(devctrl_timer_val), (uint8_t *)&devctrl_timer_val}
        },
        [DEVCTRL_SVC_TIMER_NTF_CFG_IDX] = {
                {ESP_GATT_AUTO_RSP},
                {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                        sizeof(uint16_t), sizeof(devctrl_timer_ccc), (uint8_t *)devctrl_timer_ccc}
        },
        [DEVCTRL_SVC_PLACEMENT_CHAR_IDX] = {
                {ESP_GATT_AUTO_RSP},
                {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}
        },
        [DEVCTRL_SVC_PLACEMENT_VAL_IDX] = {
                {ESP_GATT_AUTO_RSP},
                {ESP_UUID_LEN_128, (uint8_t *)&DEVCTRL_PLACEMENT_CHAR_UUID, ESP_GATT_PERM_READ,
                        sizeof(uint8_t), sizeof(devctrl_placement_val), (uint8_t *)&devctrl_placement_val}
        },
        [DEVCTRL_SVC_PLACEMENT_NTF_CFG_IDX] = {
                {ESP_GATT_AUTO_RSP},
                {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                        sizeof(uint16_t), sizeof(devctrl_placement_ccc), (uint8_t *)devctrl_placement_ccc}
        },[DEVCTRL_SVC_TEMP_CHAR_IDX] = {
                {ESP_GATT_AUTO_RSP},
                {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
                        CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}
        },
        [DEVCTRL_SVC_TEMP_VAL_IDX] = {
                {ESP_GATT_AUTO_RSP},
                {ESP_UUID_LEN_128, (uint8_t *)&DEVCTRL_TEMP_CHAR_UUID, ESP_GATT_PERM_READ,
                        sizeof(uint32_t), sizeof(devctrl_temp_val), (uint8_t *)&devctrl_temp_val}
        },
        [DEVCTRL_SVC_TEMP_NTF_CFG_IDX] = {
                {ESP_GATT_AUTO_RSP},
                {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                        sizeof(uint16_t), sizeof(devctrl_temp_ccc), (uint8_t *)devctrl_temp_ccc}
        },
};

static uint16_t devctrl_handle_table[DEVCTRL_SVC_IDX_NB];

#endif /* BT_DEVCTRL_H */

#ifdef __cplusplus
}
#endif