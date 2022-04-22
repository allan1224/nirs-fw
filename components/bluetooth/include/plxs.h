#ifdef __cplusplus
extern "C" {
#endif

#include <cstdio>

#ifndef BT_PLXS_H
#define BT_PLXS_H

#define PLXS_APP_IDX 1
#define PLXS_APP_ID 0x43
#define PLXS_SVC_INST_ID 1

static const uint16_t PLXS_SERVICE_UUID = 0x1822;
static const uint16_t PLXS_CONT_MEAS_CHAR_UUID = 0x2A5F;
static const uint16_t PLXS_FEAT_CHAR_UUID = 0x2A60;

static uint8_t plxs_cont_meas_val[5] = {
        // Pulse Rate
        0x00,0x00,
        // SpO2
        0x07,0xFF,
        // Flags
        0x00
};
static uint8_t plxs_cont_meas_ccc[2] = {0x00, 0x00};

static uint8_t plxs_feat_val[2] = {0x00, 0x00};

enum plx_svc_table_e {
    // Pulse Oximeter Service Index
    PLX_SVC_IDX,

    // PLXS Continuous Measurement Characteristic, Value, and CCCD Config Indexes
    PLX_SVC_CONT_MEAS_CHAR_IDX,
    PLX_SVC_CONT_MEAS_VAL_IDX,
    PLX_SVC_CONT_MEAS_NTF_CFG_IDX,

    // PLXS Features Characteristic and Value Indexes
    PLX_SVC_FEAT_CHAR_IDX,
    PLX_SVC_FEAT_VAL_IDX,

    PLX_SVC_IDX_NB,
};

static const esp_gatts_attr_db_t plxs_gatt_db[PLX_SVC_IDX_NB] = {
    [PLX_SVC_IDX] = {
            {ESP_GATT_AUTO_RSP},
            {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
             sizeof(uint16_t), sizeof(PLXS_SERVICE_UUID), (uint8_t *)&PLXS_SERVICE_UUID}
    },
    [PLX_SVC_CONT_MEAS_CHAR_IDX] = {
            {ESP_GATT_AUTO_RSP},
            {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
             CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}
    },
    [PLX_SVC_CONT_MEAS_VAL_IDX] = {
            {ESP_GATT_AUTO_RSP},
            {ESP_UUID_LEN_16, (uint8_t *)&PLXS_CONT_MEAS_CHAR_UUID, ESP_GATT_PERM_READ,
             sizeof(uint8_t[5]), sizeof(plxs_cont_meas_val), (uint8_t *)&plxs_cont_meas_val}
    },
    [PLX_SVC_CONT_MEAS_NTF_CFG_IDX] = {
            {ESP_GATT_AUTO_RSP},
            {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
             sizeof(uint16_t), sizeof(plxs_cont_meas_ccc), (uint8_t *)&plxs_cont_meas_ccc}
    },
    [PLX_SVC_FEAT_CHAR_IDX] = {
            {ESP_GATT_AUTO_RSP},
            {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
             CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}
    },
    [PLX_SVC_FEAT_VAL_IDX] = {
            {ESP_GATT_AUTO_RSP},
            {ESP_UUID_LEN_16, (uint8_t *)&PLXS_FEAT_CHAR_UUID, ESP_GATT_PERM_READ,
             sizeof(uint16_t), sizeof(plxs_feat_val), (uint8_t *)&plxs_feat_val}
    },
};

uint16_t plxs_handle_table[PLX_SVC_IDX_NB];

#endif /* BT_PLXS_H */

#ifdef __cplusplus
}
#endif