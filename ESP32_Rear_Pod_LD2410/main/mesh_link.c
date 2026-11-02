#include "mesh_link.h"
#include "rear_config.h"

#include <stdatomic.h>
#include <string.h>
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_log.h"

#define OP_COMMAND ESP_BLE_MESH_MODEL_OP_3(0x10, TG_COMPANY_ID)
#define OP_EVENT   ESP_BLE_MESH_MODEL_OP_3(0x11, TG_COMPANY_ID)

static const char *TAG = "mesh";
static rear_command_cb_t command_callback;
static atomic_bool friend_ready;
static uint8_t device_uuid[16] = {0x54, 0x47, 0x52, 0x50}; // "TGRP"

static esp_ble_mesh_cfg_srv_t config_server = {
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_NOT_SUPPORTED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
    .default_ttl = 5,
};

static esp_ble_mesh_model_op_t vendor_ops[] = {
    ESP_BLE_MESH_MODEL_OP(OP_COMMAND, 8),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
};

static esp_ble_mesh_model_t vendor_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(TG_COMPANY_ID, TG_MODEL_ID,
                              vendor_ops, NULL, NULL),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vendor_models),
};

static esp_ble_mesh_comp_t composition = {
    .cid = TG_COMPANY_ID,
    .element_count = 1,
    .elements = elements,
};

static esp_ble_mesh_prov_t provisioning = {
    .uuid = device_uuid,
};

static void provisioning_callback(esp_ble_mesh_prov_cb_event_t event,
                                  esp_ble_mesh_prov_cb_param_t *param)
{
    (void)param;
    switch (event) {
    case ESP_BLE_MESH_LPN_FRIENDSHIP_ESTABLISH_EVT:
        atomic_store(&friend_ready, true);
        ESP_LOGI(TAG, "Friendship established");
        break;
    case ESP_BLE_MESH_LPN_FRIENDSHIP_TERMINATE_EVT:
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        atomic_store(&friend_ready, false);
        ESP_LOGW(TAG, "Friendship unavailable");
        break;
    default:
        break;
    }
}

static void model_callback(esp_ble_mesh_model_cb_event_t event,
                           esp_ble_mesh_model_cb_param_t *param)
{
    if (event != ESP_BLE_MESH_MODEL_OPERATION_EVT) {
        return;
    }
    if (param->model_operation.opcode != OP_COMMAND ||
        param->model_operation.length != 8) {
        return;
    }

    esp_ble_mesh_msg_ctx_t *context = param->model_operation.ctx;
    if (context->addr != TG_GATEWAY_ADDRESS ||
        context->app_idx != TG_APP_INDEX ||
        context->net_idx != TG_NET_INDEX) {
        ESP_LOGW(TAG, "Ignored command from untrusted address 0x%04x",
                 context->addr);
        return;
    }

    const uint8_t *data = param->model_operation.msg;
    if (data[0] != 3 || data[1] < CMD_START_SCAN ||
        data[1] > CMD_CLEAR_FAULT || data[7] != 0) {
        ESP_LOGW(TAG, "Ignored invalid command payload");
        return;
    }

    rear_command_t command = {
        .type = data[1],
        .id = (uint32_t)data[2] |
              ((uint32_t)data[3] << 8) |
              ((uint32_t)data[4] << 16) |
              ((uint32_t)data[5] << 24),
        .arg = data[6],
    };
    if (command_callback != NULL) {
        command_callback(&command);
    }
}

esp_err_t mesh_link_init(rear_command_cb_t callback)
{
    command_callback = callback;

    esp_bt_controller_config_t controller = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t error = esp_bt_controller_init(&controller);
    if (error != ESP_OK) return error;
    error = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (error != ESP_OK) return error;
    error = esp_bluedroid_init();
    if (error != ESP_OK) return error;
    error = esp_bluedroid_enable();
    if (error != ESP_OK) return error;

    memcpy(device_uuid + 4, esp_bt_dev_get_address(), 6);
    error = esp_ble_mesh_register_prov_callback(provisioning_callback);
    if (error != ESP_OK) return error;
    error = esp_ble_mesh_register_custom_model_callback(model_callback);
    if (error != ESP_OK) return error;
    error = esp_ble_mesh_init(&provisioning, &composition);
    if (error != ESP_OK) return error;

    if (!esp_ble_mesh_node_is_provisioned()) {
        return esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV |
                                             ESP_BLE_MESH_PROV_GATT);
    }
    ESP_LOGI(TAG, "Restored provisioned Mesh node");
    return ESP_OK;
}

esp_err_t mesh_link_send(uint8_t event, uint32_t id,
                         uint8_t flags, uint8_t detail)
{
    if (!esp_ble_mesh_node_is_provisioned()) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t payload[8] = {
        3, event,
        (uint8_t)id, (uint8_t)(id >> 8),
        (uint8_t)(id >> 16), (uint8_t)(id >> 24),
        flags, detail,
    };
    esp_ble_mesh_msg_ctx_t context = {
        .net_idx = TG_NET_INDEX,
        .app_idx = TG_APP_INDEX,
        .addr = TG_GATEWAY_ADDRESS,
        .send_ttl = 5,
    };
    return esp_ble_mesh_server_model_send_msg(&vendor_models[0], &context,
                                               OP_EVENT,
                                               sizeof(payload), payload);
}

bool mesh_link_friend_ready(void)
{
    return atomic_load(&friend_ready);
}

bool mesh_link_is_provisioned(void)
{
    return esp_ble_mesh_node_is_provisioned();
}
