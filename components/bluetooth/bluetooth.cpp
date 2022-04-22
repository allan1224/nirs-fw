#include <cstring>
#include "esp_gatt_common_api.h"
#include "esp_event.h"
#include "bluetooth.h"
#include "gap.h"
#include "plxs.h"
#include "devctrl.h"
#include "../device-controls/include/device-controls.h"
#include "../oximeter-sensors/include/oximeter-sensors.h"
#include "../treatment-timer/include/treatment-timer.h"

#define BT_MODULE_TAG "NIRS_BT_CONTROLLER"

static uint8_t dummy_data[2] = {0x00, 0x00};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
//    uint16_t app_id;
    uint16_t conn_id;
//    uint16_t service_handle;
//    esp_gatt_srvc_id_t service_id;
//    uint16_t char_handle;
//    esp_bt_uuid_t char_uuid;
//    esp_gatt_perm_t perm;
//    esp_gatt_char_prop_t property;
//    uint16_t descr_handle;
//    esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst nirs_profile_tab[NIRS_PROFILE_NUM] = {
        [DEVCTRL_APP_IDX] = {
                .gatts_cb = BluetoothController::gatts_profile_event_handler,
                .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
                .conn_id = UINT16_MAX,
        },
        [PLXS_APP_IDX] = {
                .gatts_cb = BluetoothController::gatts_profile_event_handler,
                .gatts_if = ESP_GATT_IF_NONE,
                .conn_id = UINT16_MAX,
        },
};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
#ifdef CONFIG_SET_RAW_ADV_DATA
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
#else
            case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
#endif
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(BT_MODULE_TAG, "advertising start failed");
            }else{
                ESP_LOGI(BT_MODULE_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(BT_MODULE_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(BT_MODULE_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(BT_MODULE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                     param->update_conn_params.status,
                     param->update_conn_params.min_int,
                     param->update_conn_params.max_int,
                     param->update_conn_params.conn_int,
                     param->update_conn_params.latency,
                     param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

void BluetoothController::gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(BT_MODULE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
#ifdef CONFIG_SET_RAW_ADV_DATA
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret){
                ESP_LOGE(BT_MODULE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
                ESP_LOGE(BT_MODULE_TAG, "error name = %s", esp_err_to_name(raw_adv_ret));
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret){
                ESP_LOGE(BT_MODULE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#else
            //config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret){
                ESP_LOGE(BT_MODULE_TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            //config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret){
                ESP_LOGE(BT_MODULE_TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#endif
            if (param->reg.app_id == DEVCTRL_APP_ID) {
                esp_err_t create_devctrl_attr_ret = esp_ble_gatts_create_attr_tab(devctrl_gatt_db, gatts_if, DEVCTRL_SVC_IDX_NB, DEVCTRL_SVC_INST_ID);
                if (create_devctrl_attr_ret){
                    ESP_LOGE(BT_MODULE_TAG, "create devctrl attr table failed, error code = %x", create_devctrl_attr_ret);
                }
            } else if (param->reg.app_id == PLXS_APP_ID) {
                esp_err_t create_plxs_attr_ret = esp_ble_gatts_create_attr_tab(plxs_gatt_db, gatts_if, PLX_SVC_IDX_NB, PLXS_SVC_INST_ID);
                if (create_plxs_attr_ret){
                    ESP_LOGE(BT_MODULE_TAG, "create plxs attr table failed, error code = %x", create_plxs_attr_ret);
                }
            }

        }
            break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(BT_MODULE_TAG, "ESP_GATTS_READ_EVT");
            ESP_LOGD(BT_MODULE_TAG, "conn_id = %d, handle = %s, need_rsp = %d",
                     param->read.conn_id, handle_to_str(param->read.handle), param->read.need_rsp);
            break;
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(BT_MODULE_TAG, "ESP_GATTS_WRITE_EVT");
            ESP_LOGD(BT_MODULE_TAG, "handle = %s, len = %d, value :", handle_to_str(param->write.handle), param->write.len);
            ESP_LOG_BUFFER_HEX_LEVEL(BT_MODULE_TAG, param->write.value, param->write.len, ESP_LOG_DEBUG);

            if (devctrl_handle_table[DEVCTRL_SVC_LED_STAT_NTF_CFG_IDX] == param->write.handle && param->write.len == 2) {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == 0x0001) {
                    ESP_LOGI(BT_MODULE_TAG, "LED Array Status - Notify enabled");
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id,
                                                devctrl_handle_table[DEVCTRL_SVC_LED_STAT_VAL_IDX],
                                                sizeof(devctrl_led_stat_val), &devctrl_led_stat_val, false);
                } else if (descr_value == 0x0000) {
                    ESP_LOGI(BT_MODULE_TAG, "LED Array Status - Notify/indicate disabled");
                } else {
                    ESP_LOGE(BT_MODULE_TAG, "Unknown descr value");
                    ESP_LOG_BUFFER_HEX_LEVEL(BT_MODULE_TAG, param->write.value, param->write.len, ESP_LOG_ERROR);
                }

            } else if (devctrl_handle_table[DEVCTRL_SVC_LED_STAT_VAL_IDX] == param->write.handle && param->write.len == 1) {
                ESP_LOGD(BT_MODULE_TAG, "Treatment is %s", DeviceControls::treatment_is_running() ? "RUNNING" : "NOT RUNNING");
                if (!DeviceControls::treatment_is_running()) {
                    memcpy(&devctrl_led_stat_val, param->write.value, param->write.len);
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, param->write.handle, sizeof(devctrl_led_stat_val),
                                                &devctrl_led_stat_val, false);
                    esp_err_t event_err = esp_event_post(DEVCTRL_EVENT, UPDATE_LED_STATE, param->write.value, param->write.len, portMAX_DELAY);
                    if (event_err) ESP_LOGE(BT_MODULE_TAG, "Error = %s", esp_err_to_name(event_err));
                }

            } else if (devctrl_handle_table[DEVCTRL_SVC_TIMER_NTF_CFG_IDX] == param->write.handle && param->write.len == 2) {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == 0x0001) {
                    ESP_LOGI(BT_MODULE_TAG, "Timer - Notify enabled");
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id,
                                                devctrl_handle_table[DEVCTRL_SVC_TIMER_VAL_IDX],
                                                sizeof(devctrl_timer_val), devctrl_timer_val, false);
                } else if (descr_value == 0x0000) {
                    ESP_LOGI(BT_MODULE_TAG, "Timer - notify/indicate disabled");
                } else {
                    ESP_LOGE(BT_MODULE_TAG, "Unknown descr value");
                    ESP_LOG_BUFFER_HEX_LEVEL(BT_MODULE_TAG, param->write.value, param->write.len, ESP_LOG_ERROR);
                }

            } else if (devctrl_handle_table[DEVCTRL_SVC_TIMER_VAL_IDX] == param->write.handle && param->write.len == 4) {
                bool valid_timer_write = validate_timer_write(param->write.value, param->write.len);
                if (valid_timer_write) {
                    memcpy(&devctrl_timer_val, param->write.value, param->write.len);
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id,
                                                devctrl_handle_table[DEVCTRL_SVC_TIMER_VAL_IDX],
                                                sizeof(devctrl_timer_val), devctrl_timer_val, false);
                    esp_err_t event_err = esp_event_post(TREATMENT_TIMER_EVENT, UPDATE_TIMER_STATE, param->write.value, param->write.len, portMAX_DELAY);
                    if (event_err) ESP_LOGE(BT_MODULE_TAG, "Error = %s", esp_err_to_name(event_err));
                    ESP_LOGI(BT_MODULE_TAG, "Updated Timer.");
                } else {
                    ESP_LOGI(BT_MODULE_TAG, "Write was invalid. No changes made to timer.");
                }

            } else if (devctrl_handle_table[DEVCTRL_SVC_PLACEMENT_NTF_CFG_IDX] == param->write.handle && param->write.len == 2) {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == 0x0001) {
                    ESP_LOGI(BT_MODULE_TAG, "Ambient Light - Notify enabled");
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id,
                                                devctrl_handle_table[DEVCTRL_SVC_PLACEMENT_VAL_IDX],
                                                sizeof(devctrl_placement_val), &devctrl_placement_val, false);
                } else if (descr_value == 0x0000) {
                    ESP_LOGI(BT_MODULE_TAG, "Ambient Light - Notify/indicate disabled");
                } else {
                    ESP_LOGE(BT_MODULE_TAG, "Unknown descr value");
                    ESP_LOG_BUFFER_HEX_LEVEL(BT_MODULE_TAG, param->write.value, param->write.len, ESP_LOG_ERROR);
                }

            } else if (devctrl_handle_table[DEVCTRL_SVC_TEMP_NTF_CFG_IDX] == param->write.handle && param->write.len == 2) {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == 0x0001) {
                    ESP_LOGI(BT_MODULE_TAG, "Temperature - Notify enabled");
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id,
                                                devctrl_handle_table[DEVCTRL_SVC_TEMP_VAL_IDX],
                                                sizeof(devctrl_temp_val), devctrl_temp_val, false);
                } else if (descr_value == 0x0000) {
                    ESP_LOGI(BT_MODULE_TAG, "Temperature - Notify/indicate disabled");
                } else {
                    ESP_LOGE(BT_MODULE_TAG, "Unknown descr value");
                    ESP_LOG_BUFFER_HEX_LEVEL(BT_MODULE_TAG, param->write.value, param->write.len, ESP_LOG_ERROR);
                }

            } else if (plxs_handle_table[PLX_SVC_CONT_MEAS_NTF_CFG_IDX] == param->write.handle && param->write.len == 2) {
                uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                if (descr_value == 0x0001) {
                    ESP_LOGI(BT_MODULE_TAG, "SPO2 - Notify enabled");
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, plxs_handle_table[PLX_SVC_CONT_MEAS_VAL_IDX],
                                                sizeof(plxs_cont_meas_val), plxs_cont_meas_val, false);
                } else if (descr_value == 0x0000) {
                    ESP_LOGI(BT_MODULE_TAG, "SPO2 - Notify/indicate disabled");
                } else {
                    ESP_LOGE(BT_MODULE_TAG, "Unknown descr value");
                    ESP_LOG_BUFFER_HEX_LEVEL(BT_MODULE_TAG, param->write.value, param->write.len, ESP_LOG_ERROR);
                }
            }

            if (param->write.need_rsp){
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, nullptr);
            }
            break;
        case ESP_GATTS_SET_ATTR_VAL_EVT:
            ESP_LOGI(BT_MODULE_TAG, "ESP_GATTS_SET_ATTR_VAL_EVT");
            ESP_LOGD(BT_MODULE_TAG, "handle = %s, status = %s", handle_to_str(param->set_attr_val.attr_handle),
                     gatts_status_to_name(param->set_attr_val.status));
            break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            ESP_LOGI(BT_MODULE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(BT_MODULE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(BT_MODULE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(BT_MODULE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT: {
            ESP_LOGI(BT_MODULE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(BT_MODULE_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params;
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            if (nirs_profile_tab[PLX_SVC_IDX].gatts_if == gatts_if) {
                nirs_profile_tab[PLX_SVC_IDX].conn_id = param->connect.conn_id;
            }
            if (nirs_profile_tab[DEVCTRL_SVC_IDX].gatts_if == gatts_if) {
                nirs_profile_tab[DEVCTRL_SVC_IDX].conn_id = param->connect.conn_id;
            }
        }
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(BT_MODULE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            if (nirs_profile_tab[PLX_SVC_IDX].gatts_if == gatts_if) {
                nirs_profile_tab[PLX_SVC_IDX].conn_id = UINT16_MAX;
            }
            if (nirs_profile_tab[DEVCTRL_SVC_IDX].gatts_if == gatts_if) {
                nirs_profile_tab[DEVCTRL_SVC_IDX].conn_id = UINT16_MAX;
            }
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(BT_MODULE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.svc_inst_id == DEVCTRL_SVC_INST_ID) {
                if (param->add_attr_tab.num_handle != DEVCTRL_SVC_IDX_NB) {
                    ESP_LOGE(BT_MODULE_TAG,
                             "create attribute table abnormally, num_handle (%d) doesn't equal to DEVCTRL_SVC_IDX_NB(%d)",
                             param->add_attr_tab.num_handle, DEVCTRL_SVC_IDX_NB);
                } else {
                    ESP_LOGI(BT_MODULE_TAG, "create attribute table successfully, number of handles = %d\n", param->add_attr_tab.num_handle);
                    memcpy(devctrl_handle_table, param->add_attr_tab.handles, sizeof(devctrl_handle_table));
                    esp_ble_gatts_start_service(devctrl_handle_table[DEVCTRL_SVC_IDX]);
                }
            }
            else if (param->add_attr_tab.svc_inst_id == PLXS_SVC_INST_ID) {
                if (param->add_attr_tab.num_handle != PLX_SVC_IDX_NB) {
                    ESP_LOGE(BT_MODULE_TAG,
                             "create attribute table abnormally, num_handle (%d) doesn't equal to PLX_SVC_IDX_NB(%d)",
                             param->add_attr_tab.num_handle, PLX_SVC_IDX_NB);
                } else {
                    ESP_LOGI(BT_MODULE_TAG, "create attribute table successfully, number of handles = %d\n", param->add_attr_tab.num_handle);
                    memcpy(plxs_handle_table, param->add_attr_tab.handles, sizeof(plxs_handle_table));
                    esp_ble_gatts_start_service(plxs_handle_table[PLX_SVC_IDX]);
                }
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            if (param->reg.app_id == DEVCTRL_APP_ID) {
                nirs_profile_tab[DEVCTRL_APP_IDX].gatts_if = gatts_if;
            } else if (param->reg.app_id == PLXS_APP_ID) {
                nirs_profile_tab[PLXS_APP_IDX].gatts_if = gatts_if;
            }
        } else {
            ESP_LOGE(BT_MODULE_TAG, "reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }

    int idx;
    for (idx = 0; idx < NIRS_PROFILE_NUM; idx++) {
        /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
        if (gatts_if == ESP_GATT_IF_NONE || gatts_if == nirs_profile_tab[idx].gatts_if) {
            if (nirs_profile_tab[idx].gatts_cb) {
                nirs_profile_tab[idx].gatts_cb(event, gatts_if, param);
            }
        }
    }
}

BluetoothController::BluetoothController() {
    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(BT_MODULE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(BT_MODULE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(BT_MODULE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(BT_MODULE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(BT_MODULE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(BT_MODULE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(DEVCTRL_APP_ID);
    if (ret){
        ESP_LOGE(BT_MODULE_TAG, "devctrl gatts app register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(PLXS_APP_ID);
    if (ret){
        ESP_LOGE(BT_MODULE_TAG, "plxs gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(BT_MODULE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
    esp_event_handler_register(TREATMENT_SEQUENCE_EVENT, SEQUENCE_COMPLETE, BluetoothController::timer_event_handler, nullptr);
    esp_event_handler_register(SENSOR_EVENT, SPO2_READ, BluetoothController::plxs_event_handler, nullptr);
    esp_event_handler_register(SENSOR_EVENT, AMBIENT_READ, BluetoothController::ambient_event_handler, nullptr);
    esp_event_handler_register(SENSOR_EVENT, TEMPERATURE_READ, BluetoothController::temp_event_handler, nullptr);
}

void BluetoothController::plxs_event_handler(void *handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    ESP_LOGI(BT_MODULE_TAG, "Handling PLXS event");
    if (event_id == SPO2_READ) {
        // auto spO2_read = *((double *) event_data);
        dummy_data[0] += 1;
        dummy_data[1] += 1;
        uint8_t new_plxs_value[5] = {0x00, 0x00, dummy_data[1], dummy_data[0], 0x00};

        esp_ble_gatts_set_attr_value(plxs_handle_table[PLX_SVC_CONT_MEAS_VAL_IDX], sizeof(new_plxs_value), new_plxs_value);
        memcpy(plxs_cont_meas_val, new_plxs_value, sizeof(new_plxs_value));
        esp_gatt_if_t gatts_if = nirs_profile_tab[PLX_SVC_IDX].gatts_if;
        uint16_t conn_id = nirs_profile_tab[PLX_SVC_IDX].conn_id;
        if (conn_id != UINT16_MAX) esp_ble_gatts_send_indicate(gatts_if, conn_id, plxs_handle_table[PLX_SVC_CONT_MEAS_VAL_IDX],
                                                               sizeof(new_plxs_value), new_plxs_value, false);
    }
}

void BluetoothController::timer_event_handler(void *handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    ESP_LOGI(BT_MODULE_TAG, "Handling timer event");
    if (event_id == SEQUENCE_COMPLETE) {
        uint8_t new_timer_val[4] = {0x00, 0x00, devctrl_timer_val[2], devctrl_timer_val[3]};
        esp_ble_gatts_set_attr_value(devctrl_handle_table[DEVCTRL_SVC_TIMER_VAL_IDX], sizeof(new_timer_val), new_timer_val);
        memcpy(devctrl_timer_val, new_timer_val, sizeof(new_timer_val));
        esp_gatt_if_t gatts_if = nirs_profile_tab[DEVCTRL_SVC_IDX].gatts_if;
        uint16_t conn_id = nirs_profile_tab[DEVCTRL_SVC_IDX].conn_id;
        if (conn_id != UINT16_MAX) esp_ble_gatts_send_indicate(gatts_if, conn_id, devctrl_handle_table[DEVCTRL_SVC_TIMER_VAL_IDX],
                                                               sizeof(new_timer_val), new_timer_val, false);
    }
}

void BluetoothController::ambient_event_handler(void *handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    ESP_LOGI(BT_MODULE_TAG, "Handling ambient event");
    if (event_id == AMBIENT_READ) {
        auto data = (bool *) event_data;
        bool valid_placement = *data;
        uint8_t new_placement_val = 0;

        if (valid_placement) {
            new_placement_val = 0x03;
        } else {
            new_placement_val = 0x00;
        }

        esp_ble_gatts_set_attr_value(devctrl_handle_table[DEVCTRL_SVC_PLACEMENT_VAL_IDX], sizeof(new_placement_val), &new_placement_val);
        memcpy(&devctrl_placement_val, &new_placement_val, sizeof(new_placement_val));
        esp_gatt_if_t gatts_if = nirs_profile_tab[DEVCTRL_SVC_IDX].gatts_if;
        uint16_t conn_id = nirs_profile_tab[DEVCTRL_SVC_IDX].conn_id;
        if (conn_id != UINT16_MAX) esp_ble_gatts_send_indicate(gatts_if, conn_id, devctrl_handle_table[DEVCTRL_SVC_PLACEMENT_VAL_IDX],
                                                               sizeof(new_placement_val), &new_placement_val, false);
    }
}

void BluetoothController::temp_event_handler(void *handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    ESP_LOGI(BT_MODULE_TAG, "Handling temperature event");
    if (event_id == TEMPERATURE_READ) {
        auto data = (uint32_t *) event_data;

        uint8_t new_temp_val[4];
        for (int i = 0; i < 4; i++) {
            new_temp_val[3 - i] = ((*data) >> (i * 8)) & 0xFF;
        }

        esp_ble_gatts_set_attr_value(devctrl_handle_table[DEVCTRL_SVC_TEMP_VAL_IDX], sizeof(new_temp_val), new_temp_val);
        memcpy(&devctrl_temp_val, &new_temp_val, sizeof(new_temp_val));
        esp_gatt_if_t gatts_if = nirs_profile_tab[DEVCTRL_SVC_IDX].gatts_if;
        uint16_t conn_id = nirs_profile_tab[DEVCTRL_SVC_IDX].conn_id;
        if (conn_id != UINT16_MAX) esp_ble_gatts_send_indicate(gatts_if, conn_id, devctrl_handle_table[DEVCTRL_SVC_TEMP_VAL_IDX],
                                                               sizeof(new_temp_val), new_temp_val, false);
    }
}

/**
 * Ensures timer write will trigger a valid operation before updating in-memory value.
 * */
bool BluetoothController::validate_timer_write(const uint8_t *value, uint8_t length) {
    if (length != 4) {
        return false;
    }

    bool new_timer_en = value[0] == 0x80;
    uint16_t new_duration_sec = (value[2] << 8) | value[3];
    uint16_t current_duration_sec = (devctrl_timer_val[2] << 8) | devctrl_timer_val[3];

    if (new_timer_en && current_duration_sec != new_duration_sec) {
        return false;
    }

    return true;
}

const char * BluetoothController::handle_to_str(uint16_t handle) {
    if (handle == devctrl_handle_table[DEVCTRL_SVC_LED_STAT_CHAR_IDX]) {
        return ("LED_ARRAY_CHAR");
    } else if (handle == devctrl_handle_table[DEVCTRL_SVC_LED_STAT_VAL_IDX]) {
        return ("LED_ARRAY_VAL");
    } else if (handle == devctrl_handle_table[DEVCTRL_SVC_LED_STAT_NTF_CFG_IDX]) {
        return ("LED_ARRAY_NOTIF");
    } else if (handle == devctrl_handle_table[DEVCTRL_SVC_PLACEMENT_CHAR_IDX]) {
        return ("PLACEMENT_CHAR");
    } else if (handle == devctrl_handle_table[DEVCTRL_SVC_PLACEMENT_VAL_IDX]) {
        return ("PLACEMENT_VAL");
    } else if (handle == devctrl_handle_table[DEVCTRL_SVC_PLACEMENT_NTF_CFG_IDX]) {
        return ("PLACEMENT_NOTIF");
    } else if (handle == devctrl_handle_table[DEVCTRL_SVC_TIMER_CHAR_IDX]) {
        return ("TIMER_CHAR");
    } else if (handle == devctrl_handle_table[DEVCTRL_SVC_TIMER_VAL_IDX]) {
        return ("TIMER_VAL");
    } else if (handle == devctrl_handle_table[DEVCTRL_SVC_TIMER_NTF_CFG_IDX]) {
        return ("TIMER_NOTIF");
    } else if (handle == devctrl_handle_table[DEVCTRL_SVC_TEMP_CHAR_IDX]) {
        return ("TEMP_CHAR");
    } else if (handle == devctrl_handle_table[DEVCTRL_SVC_TEMP_VAL_IDX]) {
        return ("TEMP_VAL");
    } else if (handle == devctrl_handle_table[DEVCTRL_SVC_TEMP_NTF_CFG_IDX]) {
        return ("TEMP_NOTIF");
    } else if (handle == plxs_handle_table[PLX_SVC_CONT_MEAS_CHAR_IDX]) {
        return ("SPO2_CHAR");
    } else if (handle == plxs_handle_table[PLX_SVC_CONT_MEAS_VAL_IDX]) {
        return ("SPO2_VAL");
    } else if (handle == plxs_handle_table[PLX_SVC_CONT_MEAS_NTF_CFG_IDX]) {
        return ("SPO2_NOTIF");
    } else if (handle == plxs_handle_table[PLX_SVC_FEAT_CHAR_IDX]) {
        return ("SPO2_FEAT_CHAR");
    } else if (handle == plxs_handle_table[PLX_SVC_FEAT_VAL_IDX]) {
        return ("SPO2_FEAT_VAL");
    }

    return "UNKNOWN";
}

const char * BluetoothController::gatts_status_to_name(esp_gatt_status_t status) {
    switch(status) {
        case ESP_GATT_INVALID_HANDLE:
            return "ESP_GATT_INVALID_HANDLE";
        case ESP_GATT_READ_NOT_PERMIT:
            return "ESP_GATT_READ_NOT_PERMIT";
        case ESP_GATT_WRITE_NOT_PERMIT:
            return "ESP_GATT_WRITE_NOT_PERMIT";
        case ESP_GATT_INVALID_PDU:
            return "ESP_GATT_INVALID_PDU";
        case ESP_GATT_INSUF_AUTHENTICATION:
            return "ESP_GATT_INSUF_AUTHENTICATION";
        case ESP_GATT_REQ_NOT_SUPPORTED:
            return "ESP_GATT_REQ_NOT_SUPPORTED";
        case ESP_GATT_INVALID_OFFSET:
            return "ESP_GATT_INVALID_OFFSET";
        case ESP_GATT_INSUF_AUTHORIZATION:
            return "ESP_GATT_INSUF_AUTHORIZATION";
        case ESP_GATT_PREPARE_Q_FULL:
            return "ESP_GATT_PREPARE_Q_FULL";
        case ESP_GATT_NOT_FOUND:
            return "ESP_GATT_NOT_FOUND";
        case ESP_GATT_NOT_LONG:
            return "ESP_GATT_NOT_LONG";
        case ESP_GATT_INSUF_KEY_SIZE:
            return "ESP_GATT_INSUF_KEY_SIZE";
        case ESP_GATT_INVALID_ATTR_LEN:
            return "ESP_GATT_INVALID_ATTR_LEN";
        case ESP_GATT_ERR_UNLIKELY:
            return "ESP_GATT_ERR_UNLIKELY";
        case ESP_GATT_INSUF_ENCRYPTION:
            return "ESP_GATT_INSUF_ENCRYPTION";
        case ESP_GATT_UNSUPPORT_GRP_TYPE:
            return "ESP_GATT_UNSUPPORT_GRP_TYPE";
        case ESP_GATT_INSUF_RESOURCE:
            return "ESP_GATT_INSUF_RESOURCE";
        case ESP_GATT_NO_RESOURCES:
            return "ESP_GATT_NO_RESOURCES";
        case ESP_GATT_INTERNAL_ERROR:
            return "ESP_GATT_INTERNAL_ERROR";
        case ESP_GATT_WRONG_STATE:
            return "ESP_GATT_WRONG_STATE";
        case ESP_GATT_DB_FULL:
            return "ESP_GATT_DB_FULL";
        case ESP_GATT_BUSY:
            return "ESP_GATT_BUSY";
        case ESP_GATT_ERROR:
            return "ESP_GATT_ERROR";
        case ESP_GATT_CMD_STARTED:
            return "ESP_GATT_CMD_STARTED";
        case ESP_GATT_ILLEGAL_PARAMETER:
            return "ESP_GATT_ILLEGAL_PARAMETER";
        case ESP_GATT_PENDING:
            return "ESP_GATT_PENDING";
        case ESP_GATT_AUTH_FAIL:
            return "ESP_GATT_AUTH_FAIL";
        case ESP_GATT_MORE:
            return "ESP_GATT_MORE";
        case ESP_GATT_INVALID_CFG:
            return "ESP_GATT_INVALID_CFG";
        case ESP_GATT_SERVICE_STARTED:
            return "ESP_GATT_SERVICE_STARTED";
        case ESP_GATT_ENCRYPED_NO_MITM:
            return "ESP_GATT_ENCRYPED_NO_MITM";
        case ESP_GATT_NOT_ENCRYPTED:
            return "ESP_GATT_NOT_ENCRYPTED";
        case ESP_GATT_CONGESTED:
            return "ESP_GATT_CONGESTED";
        case ESP_GATT_DUP_REG:
            return "ESP_GATT_DUP_REG";
        case ESP_GATT_ALREADY_OPEN:
            return "ESP_GATT_ALREADY_OPEN";
        case ESP_GATT_CANCEL:
            return "ESP_GATT_CANCEL";
        case ESP_GATT_STACK_RSP:
            return "ESP_GATT_STACK_RSP";
        case ESP_GATT_APP_RSP:
            return "ESP_GATT_APP_RSP";
        case ESP_GATT_UNKNOWN_ERROR:
            return "ESP_GATT_UNKNOWN_ERROR";
        case ESP_GATT_CCC_CFG_ERR:
            return "ESP_GATT_CCC_CFG_ERR";
        case ESP_GATT_PRC_IN_PROGRESS:
            return "ESP_GATT_PRC_IN_PROGRESS";
        case ESP_GATT_OUT_OF_RANGE:
            return "ESP_GATT_OUT_OF_RANGE";
        case ESP_GATT_OK:
            return "ESP_GATT_OK";
    }

    return "UNKNOWN";
}
