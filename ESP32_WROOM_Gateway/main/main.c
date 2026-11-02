#include <assert.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/spi_master.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "gateway_config.h"

#define OP_COMMAND ESP_BLE_MESH_MODEL_OP_3(0x10, TG_COMPANY_ID)
#define OP_EVENT   ESP_BLE_MESH_MODEL_OP_3(0x11, TG_COMPANY_ID)
#define STORE_MAGIC 0x47573332UL /* GW32: SPI one-way */
#define CONFIG_TIMEOUT_MS 0
#define CONFIG_TTL 5

typedef struct {
    uint16_t address;
    uint16_t model_id;
    uint32_t last_acked[6];
} node_record_t;

typedef struct {
    uint32_t magic;
    uint32_t command_counter;
    node_record_t nodes[MAX_TRACKED_NODES];
} gateway_store_t;

typedef struct {
    uint16_t source;
    uint8_t payload[8];
} mesh_event_message_t;

static const char *TAG = "gateway";
static uint8_t gateway_uuid[16] = {'T', 'G', 'G', 'W'};
static uint8_t app_key[16] = {
    0x54, 0x47, 0x2d, 0x41, 0x50, 0x50, 0x2d, 0x4b,
    0x45, 0x59, 0x2d, 0x30, 0x30, 0x30, 0x31, 0x21,
};

static nvs_handle_t storage;
static gateway_store_t store;
static QueueHandle_t mesh_event_queue;
static spi_device_handle_t stm32_spi;
static uint16_t stm32_sequence;

static esp_ble_mesh_cfg_srv_t config_server = {
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
    .default_ttl = 5,
};

static esp_ble_mesh_client_t config_client;

static const esp_ble_mesh_client_op_pair_t vendor_op_pair[] = {
    {OP_COMMAND, OP_EVENT},
};

static esp_ble_mesh_client_t vendor_client = {
    .op_pair_size = ARRAY_SIZE(vendor_op_pair),
    .op_pair = vendor_op_pair,
};

static esp_ble_mesh_model_op_t vendor_ops[] = {
    ESP_BLE_MESH_MODEL_OP(OP_EVENT, 8),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_CFG_CLI(&config_client),
};

static esp_ble_mesh_model_t vendor_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(TG_COMPANY_ID, TG_GATEWAY_MODEL_ID,
                              vendor_ops, NULL, &vendor_client),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vendor_models),
};

static esp_ble_mesh_comp_t composition = {
    .cid = TG_COMPANY_ID,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

static esp_ble_mesh_prov_t provisioning = {
    .prov_uuid = gateway_uuid,
    .prov_unicast_addr = TG_GATEWAY_ADDRESS,
    .prov_start_address = TG_FIRST_NODE_ADDRESS,
};

static uint32_t get_u32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
           ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

static void put_u16(uint8_t *bytes, uint16_t value)
{
    bytes[0] = (uint8_t)value;
    bytes[1] = (uint8_t)(value >> 8);
}

static uint16_t next_stm32_sequence(void)
{
    // Periodic Mesh radar updates intentionally reuse their transaction ID.
    // Give every forwarded event a fresh SPI sequence; repeated copies of one
    // critical event still share the same sequence and are deduplicated.
    ++stm32_sequence;
    if (stm32_sequence == 0) {
        ++stm32_sequence;
    }
    return stm32_sequence;
}

static uint16_t crc16_ccitt_false(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t index = 0; index < length; ++index) {
        crc ^= (uint16_t)data[index] << 8;
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void save_store(void)
{
    esp_err_t error = nvs_set_blob(storage, "state", &store, sizeof(store));
    if (error == ESP_OK) error = nvs_commit(storage);
    if (error != ESP_OK) {
        ESP_LOGE(TAG, "NVS save failed: %s", esp_err_to_name(error));
    }
}

static void load_store(void)
{
    size_t size = sizeof(store);
    esp_err_t error = nvs_get_blob(storage, "state", &store, &size);
    if (error != ESP_OK || size != sizeof(store) || store.magic != STORE_MAGIC) {
        memset(&store, 0, sizeof(store));
        store.magic = STORE_MAGIC;
        save_store();
    }
}

static uint16_t model_from_uuid(const uint8_t uuid[16])
{
    if (memcmp(uuid, "TGRP", 4) == 0) return TG_REAR_MODEL_ID;
    if (memcmp(uuid, "TGR3", 4) == 0) return TG_SOS_MODEL_ID;
    return 0;
}

static node_record_t *find_node(uint16_t address)
{
    for (size_t index = 0; index < MAX_TRACKED_NODES; ++index) {
        if (store.nodes[index].address == address) return &store.nodes[index];
    }

    esp_ble_mesh_node_t *mesh_node =
        esp_ble_mesh_provisioner_get_node_with_addr(address);
    if (mesh_node == NULL) return NULL;
    uint16_t model_id = model_from_uuid(mesh_node->dev_uuid);
    if (model_id == 0) return NULL;

    for (size_t index = 0; index < MAX_TRACKED_NODES; ++index) {
        if (store.nodes[index].address == 0) {
            store.nodes[index].address = address;
            store.nodes[index].model_id = model_id;
            save_store();
            return &store.nodes[index];
        }
    }
    return NULL;
}

static node_record_t *remember_node(uint16_t address, uint16_t model_id)
{
    node_record_t *node = find_node(address);
    if (node != NULL) {
        node->model_id = model_id;
        save_store();
        return node;
    }
    for (size_t index = 0; index < MAX_TRACKED_NODES; ++index) {
        if (store.nodes[index].address == 0) {
            store.nodes[index].address = address;
            store.nodes[index].model_id = model_id;
            save_store();
            return &store.nodes[index];
        }
    }
    ESP_LOGE(TAG, "Node table full; address=0x%04x", address);
    return NULL;
}

static uint32_t next_command_id(void)
{
    ++store.command_counter;
    if (store.command_counter == 0) store.command_counter = 1;
    save_store();
    return store.command_counter;
}

static esp_err_t send_mesh_command(uint16_t destination, uint16_t model_id,
                                   uint8_t command, uint32_t id, uint8_t argument)
{
    if (destination == 0 || id == 0 ||
        (model_id != TG_REAR_MODEL_ID && model_id != TG_SOS_MODEL_ID)) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t payload[8] = {
        TG_PROTOCOL_VERSION, command,
        (uint8_t)id, (uint8_t)(id >> 8),
        (uint8_t)(id >> 16), (uint8_t)(id >> 24),
        argument, 0,
    };
    esp_ble_mesh_msg_ctx_t context = {
        .net_idx = TG_NET_INDEX,
        .app_idx = TG_APP_INDEX,
        .addr = destination,
        .send_ttl = 5,
    };
    esp_err_t error = esp_ble_mesh_client_model_send_msg(
        vendor_client.model, &context, OP_COMMAND, sizeof(payload), payload,
        0, false, ROLE_PROVISIONER);
    if (error == ESP_OK) {
        ESP_LOGI(TAG, "MESH TX dst=0x%04x model=0x%04x cmd=%u id=%" PRIu32,
                 destination, model_id, command, id);
    } else {
        ESP_LOGE(TAG, "MESH TX failed dst=0x%04x: %s", destination,
                 esp_err_to_name(error));
    }
    return error;
}

static void send_mesh_ack(uint16_t source, uint16_t model_id, uint32_t id)
{
    uint8_t command = model_id == TG_REAR_MODEL_ID
                          ? REAR_CMD_ACK_EVENT
                          : SOS_CMD_SOS_ACK;
    send_mesh_command(source, model_id, command, id, 0);
}

static void notify_sos_remotes_of_rear(void)
{
    uint32_t id = next_command_id();
    for (size_t index = 0; index < MAX_TRACKED_NODES; ++index) {
        if (store.nodes[index].address != 0 &&
            store.nodes[index].model_id == TG_SOS_MODEL_ID) {
            send_mesh_command(store.nodes[index].address, TG_SOS_MODEL_ID,
                              SOS_CMD_REAR_CONFIRM, id, 0);
        }
    }
}

static bool build_stm32_frame(uint16_t model_id,
                              const uint8_t mesh_payload[8],
                              uint16_t sequence,
                              uint8_t frame[STM32_FRAME_SIZE],
                              uint8_t *stm32_event)
{
    uint8_t mesh_event = mesh_payload[1];
    uint8_t flags = mesh_payload[6];
    uint8_t detail = mesh_payload[7];
    uint16_t value = 0;

    if (model_id == TG_SOS_MODEL_ID) {
        if (mesh_event != TG_EVENT_PRIMARY) return false;
        *stm32_event = STM32_EVENT_REMOTE_SOS;
        value = 1;
    } else if (model_id == TG_REAR_MODEL_ID) {
        switch (mesh_event) {
        case TG_EVENT_STATUS:
            *stm32_event = STM32_EVENT_RADAR_HEALTH;
            value = (flags & REAR_FLAG_RADAR_FAULT) ? 0 : 1;
            break;
        case TG_EVENT_PRIMARY:
            *stm32_event = STM32_EVENT_REAR_CONFIRM;
            value = 1;
            break;
        case TG_EVENT_UPDATE_OR_DONE:
            if (flags & REAR_FLAG_RADAR_FAULT) {
                *stm32_event = STM32_EVENT_RADAR_HEALTH;
                value = 0;
            } else if (flags & REAR_FLAG_TARGET) {
                *stm32_event = STM32_EVENT_PERSON;
                value = detail == 0xFF ? 0 : (uint16_t)detail * 10U;
            } else {
                *stm32_event = STM32_EVENT_RADAR_CLEAR;
                value = 0;
            }
            break;
        case TG_EVENT_SCAN_DONE:
            *stm32_event = STM32_EVENT_RADAR_CLEAR;
            value = 0;
            break;
        case TG_EVENT_FAULT:
            *stm32_event = STM32_EVENT_RADAR_HEALTH;
            /* EVENT_FAULT is retried until its Mesh ACK reaches the LPN.  By
             * the time an old retry arrives the radar may already have
             * recovered, and the payload flags always describe the current
             * state.  Do not let a stale fault event latch STM32 unhealthy. */
            value = (flags & REAR_FLAG_RADAR_FAULT) ? 0 : 1;
            break;
        default:
            return false;
        }
    } else {
        return false;
    }

    frame[0] = STM32_FRAME_MAGIC;
    frame[1] = STM32_PROTOCOL_VERSION;
    frame[2] = *stm32_event;
    frame[3] = STM32_NODE_REMOTE;
    put_u16(frame + 4, sequence);
    put_u16(frame + 6, value);
    frame[8] = 0;
    uint16_t crc = crc16_ccitt_false(frame, 9);
    put_u16(frame + 9, crc);
    frame[11] = STM32_FRAME_END;
    return true;
}

static esp_err_t spi_send_frame(const uint8_t frame[STM32_FRAME_SIZE])
{
    spi_transaction_t transaction = {
        .length = STM32_FRAME_SIZE * 8,
        .tx_buffer = frame,
    };
    return spi_device_transmit(stm32_spi, &transaction);
}

static esp_err_t spi_send_remote_event(uint16_t source, uint16_t model_id,
                                       const uint8_t mesh_payload[8],
                                       unsigned repeats)
{
    uint8_t frame[STM32_FRAME_SIZE];
    uint8_t stm32_event;
    uint16_t sequence = next_stm32_sequence();
    if (!build_stm32_frame(model_id, mesh_payload, sequence, frame,
                           &stm32_event)) {
        ESP_LOGW(TAG, "No STM32 mapping model=0x%04x event=%u",
                 model_id, mesh_payload[1]);
        return ESP_ERR_NOT_SUPPORTED;
    }

    for (unsigned attempt = 0; attempt < repeats; ++attempt) {
        esp_err_t error = spi_send_frame(frame);
        if (error != ESP_OK) {
            ESP_LOGE(TAG, "SPI TX STM32 failed: %s", esp_err_to_name(error));
            return error;
        }
        if (attempt + 1 < repeats) {
            vTaskDelay(pdMS_TO_TICKS(SPI_REPEAT_GAP_MS));
        }
    }
    ESP_LOGI(TAG, "SPI TX STM32 src=0x%04x mesh_event=%u stm_event=0x%02x "
                  "mesh_id=%" PRIu32 " spi_seq=%u copies=%u",
             source, mesh_payload[1], stm32_event,
             get_u32(mesh_payload + 2),
             (unsigned)sequence, repeats);
    return ESP_OK;
}

static bool event_requires_ack(uint16_t model_id, uint8_t event)
{
    if (model_id == TG_REAR_MODEL_ID) {
        return event == TG_EVENT_PRIMARY || event == TG_EVENT_SCAN_DONE ||
               event == TG_EVENT_FAULT;
    }
    return model_id == TG_SOS_MODEL_ID && event == TG_EVENT_PRIMARY;
}

static void process_mesh_event(const mesh_event_message_t *message)
{
    const uint8_t *payload = message->payload;
    node_record_t *node = find_node(message->source);
    if (node == NULL) {
        ESP_LOGE(TAG, "Drop event from unknown node 0x%04x", message->source);
        return;
    }
    uint8_t event = payload[1];
    uint32_t id = get_u32(payload + 2);
    bool valid = payload[0] == TG_PROTOCOL_VERSION &&
                 ((node->model_id == TG_REAR_MODEL_ID && event >= 1 && event <= 5) ||
                  (node->model_id == TG_SOS_MODEL_ID && event >= 1 && event <= 3));
    if (!valid) {
        ESP_LOGW(TAG, "Drop invalid event src=0x%04x model=0x%04x",
                 message->source, node->model_id);
        return;
    }

    ESP_LOGW(TAG, "MESH RX src=0x%04x model=0x%04x event=%u id=%" PRIu32
                  " flags=0x%02x detail=%u",
             message->source, node->model_id, event, id, payload[6], payload[7]);

    bool critical = event_requires_ack(node->model_id, event);
    if (!critical) {
        spi_send_remote_event(message->source, node->model_id, payload, 1);
        return;
    }
    if (id == 0) {
        ESP_LOGW(TAG, "Drop critical event with zero id");
        return;
    }
    if (node->last_acked[event] >= id) {
        send_mesh_ack(message->source, node->model_id, id);
        return;
    }

    esp_err_t error = spi_send_remote_event(message->source, node->model_id,
                                             payload, SPI_CRITICAL_REPEATS);
    if (error != ESP_OK) {
        return; /* Remote will retry because no Mesh ACK was sent. */
    }

    if (node->model_id == TG_REAR_MODEL_ID &&
        event == TG_EVENT_PRIMARY) {
        notify_sos_remotes_of_rear();
    }
    node->last_acked[event] = id;
    save_store();
    send_mesh_ack(message->source, node->model_id, id);
}

static void set_config_common(esp_ble_mesh_client_common_param_t *common,
                              esp_ble_mesh_node_t *node, uint32_t opcode)
{
    memset(common, 0, sizeof(*common));
    common->opcode = opcode;
    common->model = config_client.model;
    common->ctx.net_idx = TG_NET_INDEX;
    common->ctx.app_idx = TG_APP_INDEX;
    common->ctx.addr = node->unicast_addr;
    common->ctx.send_ttl = CONFIG_TTL;
    common->msg_timeout = CONFIG_TIMEOUT_MS;
}

static esp_err_t send_composition_get(esp_ble_mesh_node_t *node)
{
    esp_ble_mesh_client_common_param_t common;
    esp_ble_mesh_cfg_client_get_state_t get = {0};
    set_config_common(&common, node,
                      ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET);
    get.comp_data_get.page = 0;
    return esp_ble_mesh_config_client_get_state(&common, &get);
}

static esp_err_t send_app_key_add(esp_ble_mesh_node_t *node)
{
    esp_ble_mesh_client_common_param_t common;
    esp_ble_mesh_cfg_client_set_state_t set = {0};
    set_config_common(&common, node, ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD);
    set.app_key_add.net_idx = TG_NET_INDEX;
    set.app_key_add.app_idx = TG_APP_INDEX;
    memcpy(set.app_key_add.app_key, app_key, sizeof(app_key));
    return esp_ble_mesh_config_client_set_state(&common, &set);
}

static esp_err_t send_model_bind(esp_ble_mesh_node_t *node, uint16_t model_id)
{
    esp_ble_mesh_client_common_param_t common;
    esp_ble_mesh_cfg_client_set_state_t set = {0};
    set_config_common(&common, node, ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND);
    set.model_app_bind.element_addr = node->unicast_addr;
    set.model_app_bind.model_app_idx = TG_APP_INDEX;
    set.model_app_bind.model_id = model_id;
    set.model_app_bind.company_id = TG_COMPANY_ID;
    return esp_ble_mesh_config_client_set_state(&common, &set);
}

static void provisioning_callback(esp_ble_mesh_prov_cb_event_t event,
                                  esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "Mesh registered err=%d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT: {
        uint16_t model_id = model_from_uuid(
            param->provisioner_recv_unprov_adv_pkt.dev_uuid);
        if (model_id == 0) break;
        esp_ble_mesh_unprov_dev_add_t device = {0};
        memcpy(device.addr, param->provisioner_recv_unprov_adv_pkt.addr,
               sizeof(device.addr));
        device.addr_type = param->provisioner_recv_unprov_adv_pkt.addr_type;
        memcpy(device.uuid, param->provisioner_recv_unprov_adv_pkt.dev_uuid,
               sizeof(device.uuid));
        device.oob_info = param->provisioner_recv_unprov_adv_pkt.oob_info;
        device.bearer = param->provisioner_recv_unprov_adv_pkt.bearer;
        esp_err_t error = esp_ble_mesh_provisioner_add_unprov_dev(
            &device, ADD_DEV_RM_AFTER_PROV_FLAG | ADD_DEV_START_PROV_NOW_FLAG |
                         ADD_DEV_FLUSHABLE_DEV_FLAG);
        if (error == ESP_OK) {
            ESP_LOGW(TAG, "Provisioning discovered model=0x%04x", model_id);
        } else if (error != ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "Add unprovisioned device: %s",
                     esp_err_to_name(error));
        }
        break;
    }
    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "Provisioning link opened");
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "Provisioning link closed reason=0x%02x",
                 param->provisioner_prov_link_close.reason);
        break;
    case ESP_BLE_MESH_PROVISIONER_PROV_COMPLETE_EVT: {
        uint16_t address = param->provisioner_prov_complete.unicast_addr;
        uint16_t model_id = model_from_uuid(
            param->provisioner_prov_complete.device_uuid);
        remember_node(address, model_id);
        char name[12];
        snprintf(name, sizeof(name), model_id == TG_REAR_MODEL_ID
                                         ? "REAR-%04X" : "SOS-%04X", address);
        esp_ble_mesh_provisioner_set_node_name(
            param->provisioner_prov_complete.node_idx, name);
        esp_ble_mesh_node_t *node =
            esp_ble_mesh_provisioner_get_node_with_addr(address);
        ESP_LOGW(TAG, "Provisioned %s addr=0x%04x model=0x%04x",
                 name, address, model_id);
        if (node != NULL) {
            esp_err_t error = send_composition_get(node);
            if (error != ESP_OK) {
                ESP_LOGE(TAG, "Composition Get failed: %s",
                         esp_err_to_name(error));
            }
        }
        break;
    }
    case ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT:
        ESP_LOGI(TAG, "Local AppKey add err=%d",
                 param->provisioner_add_app_key_comp.err_code);
        if (param->provisioner_add_app_key_comp.err_code == 0) {
            esp_err_t error = esp_ble_mesh_provisioner_bind_app_key_to_local_model(
                TG_GATEWAY_ADDRESS, TG_APP_INDEX,
                TG_GATEWAY_MODEL_ID, TG_COMPANY_ID);
            if (error != ESP_OK) {
                ESP_LOGE(TAG, "Local model bind failed: %s",
                         esp_err_to_name(error));
            }
        }
        break;
    case ESP_BLE_MESH_PROVISIONER_BIND_APP_KEY_TO_MODEL_COMP_EVT:
        ESP_LOGI(TAG, "Local model bound err=%d",
                 param->provisioner_bind_app_key_to_model_comp.err_code);
        break;
    default:
        break;
    }
}

static void config_client_callback(esp_ble_mesh_cfg_client_cb_event_t event,
                                   esp_ble_mesh_cfg_client_cb_param_t *param)
{
    esp_ble_mesh_node_t *node =
        esp_ble_mesh_provisioner_get_node_with_addr(param->params->ctx.addr);
    if (node == NULL) return;
    node_record_t *record = find_node(node->unicast_addr);
    if (record == NULL) return;

    if (param->error_code != 0) {
        ESP_LOGW(TAG, "Config error=%d opcode=0x%04" PRIx32 " addr=0x%04x",
                 param->error_code, param->params->opcode, node->unicast_addr);
    }

    if (event == ESP_BLE_MESH_CFG_CLIENT_GET_STATE_EVT &&
        param->params->opcode == ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET &&
        param->error_code == 0) {
        esp_err_t error = esp_ble_mesh_provisioner_store_node_comp_data(
            node->unicast_addr,
            param->status_cb.comp_data_status.composition_data->data,
            param->status_cb.comp_data_status.composition_data->len);
        if (error != ESP_OK) {
            ESP_LOGW(TAG, "Store composition failed: %s", esp_err_to_name(error));
        }
        error = send_app_key_add(node);
        if (error != ESP_OK) ESP_LOGE(TAG, "AppKey Add failed: %s", esp_err_to_name(error));
        return;
    }

    if (event == ESP_BLE_MESH_CFG_CLIENT_SET_STATE_EVT &&
        param->error_code == 0) {
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD) {
            esp_err_t error = send_model_bind(node, record->model_id);
            if (error != ESP_OK) ESP_LOGE(TAG, "Model Bind failed: %s", esp_err_to_name(error));
        } else if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND) {
            ESP_LOGW(TAG, "NODE READY addr=0x%04x model=0x%04x",
                     node->unicast_addr, record->model_id);
        }
        return;
    }

    if (event == ESP_BLE_MESH_CFG_CLIENT_TIMEOUT_EVT) {
        esp_err_t error = ESP_FAIL;
        if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET) {
            error = send_composition_get(node);
        } else if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD) {
            error = send_app_key_add(node);
        } else if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND) {
            error = send_model_bind(node, record->model_id);
        }
        ESP_LOGW(TAG, "Config timeout; retry result=%s", esp_err_to_name(error));
    }
}

static void queue_received_mesh_event(uint16_t source, uint32_t opcode,
                                      uint16_t length, const uint8_t *data)
{
    if (opcode != OP_EVENT || length != 8 || data == NULL) return;
    mesh_event_message_t message = {.source = source};
    memcpy(message.payload, data, sizeof(message.payload));
    if (xQueueSend(mesh_event_queue, &message, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Mesh event queue full");
    }
}

static void custom_model_callback(esp_ble_mesh_model_cb_event_t event,
                                  esp_ble_mesh_model_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_MODEL_OPERATION_EVT) {
        queue_received_mesh_event(param->model_operation.ctx->addr,
                                  param->model_operation.opcode,
                                  param->model_operation.length,
                                  param->model_operation.msg);
    } else if (event == ESP_BLE_MESH_CLIENT_MODEL_RECV_PUBLISH_MSG_EVT) {
        queue_received_mesh_event(param->client_recv_publish_msg.ctx->addr,
                                  param->client_recv_publish_msg.opcode,
                                  param->client_recv_publish_msg.length,
                                  param->client_recv_publish_msg.msg);
    } else if (event == ESP_BLE_MESH_MODEL_SEND_COMP_EVT &&
               param->model_send_comp.err_code != 0) {
        ESP_LOGW(TAG, "Mesh send complete error=%d opcode=0x%06" PRIx32,
                 param->model_send_comp.err_code,
                 param->model_send_comp.opcode);
    }
}

static esp_err_t initialise_bluetooth(void)
{
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_config_t controller = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t error = esp_bt_controller_init(&controller);
    if (error != ESP_OK) return error;
    error = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (error != ESP_OK) return error;
    error = esp_bluedroid_init();
    if (error != ESP_OK) return error;
    return esp_bluedroid_enable();
}

static esp_err_t initialise_mesh(void)
{
    memcpy(gateway_uuid + 4, esp_bt_dev_get_address(), 6);
    ESP_ERROR_CHECK(esp_ble_mesh_register_prov_callback(provisioning_callback));
    ESP_ERROR_CHECK(esp_ble_mesh_register_config_client_callback(config_client_callback));
    ESP_ERROR_CHECK(esp_ble_mesh_register_custom_model_callback(custom_model_callback));
    esp_err_t error = esp_ble_mesh_init(&provisioning, &composition);
    if (error != ESP_OK) return error;
    error = esp_ble_mesh_client_model_init(&vendor_models[0]);
    if (error != ESP_OK) return error;
    const uint8_t match[] = {'T', 'G', 'R'};
    error = esp_ble_mesh_provisioner_set_dev_uuid_match(match, sizeof(match),
                                                        0, false);
    if (error != ESP_OK) return error;
    error = esp_ble_mesh_provisioner_prov_enable(
        ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
    if (error != ESP_OK) return error;
    error = esp_ble_mesh_provisioner_add_local_app_key(
        app_key, TG_NET_INDEX, TG_APP_INDEX);
    return error;
}

static void initialise_spi(void)
{
    spi_bus_config_t bus = {
        .mosi_io_num = STM32_SPI_MOSI_GPIO,
        .miso_io_num = -1,
        .sclk_io_num = STM32_SPI_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = STM32_FRAME_SIZE,
    };
    spi_device_interface_config_t device = {
        .clock_speed_hz = STM32_SPI_CLOCK_HZ,
        .mode = STM32_SPI_MODE,
        .spics_io_num = STM32_SPI_CS_GPIO,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(STM32_SPI_HOST, &bus,
                                       SPI_DMA_DISABLED));
    ESP_ERROR_CHECK(spi_bus_add_device(STM32_SPI_HOST, &device, &stm32_spi));
}

static void gateway_task(void *argument)
{
    (void)argument;
    for (;;) {
        mesh_event_message_t message;
        if (xQueueReceive(mesh_event_queue, &message,
                          pdMS_TO_TICKS(20)) == pdTRUE) {
            process_mesh_event(&message);
        }
    }
}

void app_main(void)
{
    esp_err_t error = nvs_flash_init();
    if (error == ESP_ERR_NVS_NO_FREE_PAGES ||
        error == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        error = nvs_flash_init();
    }
    ESP_ERROR_CHECK(error);
    ESP_ERROR_CHECK(nvs_open("tripguard_gw", NVS_READWRITE, &storage));
    load_store();

    mesh_event_queue = xQueueCreate(24, sizeof(mesh_event_message_t));
    assert(mesh_event_queue != NULL);
    initialise_spi();
    ESP_ERROR_CHECK(initialise_bluetooth());
    ESP_ERROR_CHECK(initialise_mesh());

    BaseType_t created = xTaskCreate(gateway_task, "gateway_loop", 6144,
                                     NULL, 5, NULL);
    assert(created == pdPASS);
    ESP_LOGW(TAG, "READY: Mesh Provisioner/Friend; STM32 SPI master "
                  "SCK18 MOSI23 CS17 1MHz mode0");
}
