#include "mesh_link.h"
#include "remote_config.h"
#include <string.h>
#include <stdatomic.h>
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_log.h"
#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"
#define OP_COMMAND ESP_BLE_MESH_MODEL_OP_3(0x10, TG_COMPANY_ID)
#define OP_EVENT ESP_BLE_MESH_MODEL_OP_3(0x11, TG_COMPANY_ID)
static remote_command_cb command_cb;
static atomic_bool friend_ready;
static uint8_t uuid[16] = {0x54,0x47,0x52,0x33};
static esp_ble_mesh_cfg_srv_t cfg = {
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2,20),
    .relay = ESP_BLE_MESH_RELAY_NOT_SUPPORTED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
    .default_ttl = 5,
};
static esp_ble_mesh_model_op_t ops[] = {
    ESP_BLE_MESH_MODEL_OP(OP_COMMAND, 8), ESP_BLE_MESH_MODEL_OP_END,
};
static esp_ble_mesh_model_t sig[] = { ESP_BLE_MESH_MODEL_CFG_SRV(&cfg) };
static esp_ble_mesh_model_t vendor[] = {
    ESP_BLE_MESH_VENDOR_MODEL(TG_COMPANY_ID,TG_MODEL_ID,ops,NULL,NULL),
};
static esp_ble_mesh_elem_t elements[] = { ESP_BLE_MESH_ELEMENT(0,sig,vendor) };
static esp_ble_mesh_comp_t comp = {.cid=TG_COMPANY_ID,.element_count=1,.elements=elements};
static esp_ble_mesh_prov_t prov = {.uuid=uuid};
static void provisioning(esp_ble_mesh_prov_cb_event_t event, esp_ble_mesh_prov_cb_param_t *p)
{
    (void)p;
    if (event == ESP_BLE_MESH_LPN_FRIENDSHIP_ESTABLISH_EVT) {
        atomic_store(&friend_ready,true);
        ESP_LOGI("mesh","Friend established");
    } else if (event == ESP_BLE_MESH_LPN_FRIENDSHIP_TERMINATE_EVT ||
               event == ESP_BLE_MESH_NODE_PROV_RESET_EVT) {
        atomic_store(&friend_ready,false);
        ESP_LOGW("mesh","Friend unavailable");
    }
}
static void model_callback(esp_ble_mesh_model_cb_event_t event, esp_ble_mesh_model_cb_param_t *p)
{
    if (event != ESP_BLE_MESH_MODEL_OPERATION_EVT) return;
    if (p->model_operation.opcode != OP_COMMAND || p->model_operation.length != 8) return;
    esp_ble_mesh_msg_ctx_t *ctx = p->model_operation.ctx;
    if (ctx->addr != TG_GATEWAY_ADDRESS || ctx->app_idx != TG_APP_INDEX ||
        ctx->net_idx != TG_NET_INDEX) return;
    const uint8_t *b = p->model_operation.msg;
    if (b[0] != 3 || b[1] < CMD_REAR_CONFIRM || b[1] > CMD_REAR_STOP || b[7]) return;
    remote_command_t cmd = {.type=b[1],.arg=b[6],
        .id=(uint32_t)b[2] | ((uint32_t)b[3]<<8) | ((uint32_t)b[4]<<16) | ((uint32_t)b[5]<<24)};
    if (command_cb) command_cb(&cmd);
}
esp_err_t mesh_link_init(remote_command_cb cb)
{
    command_cb=cb;
    esp_bt_controller_config_t controller=BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t e=esp_bt_controller_init(&controller); if(e) return e;
    e=esp_bt_controller_enable(ESP_BT_MODE_BLE); if(e) return e;
    e=esp_bluedroid_init(); if(e) return e;
    e=esp_bluedroid_enable(); if(e) return e;
    memcpy(uuid+4,esp_bt_dev_get_address(),6);
    e=esp_ble_mesh_register_prov_callback(provisioning); if(e) return e;
    e=esp_ble_mesh_register_custom_model_callback(model_callback); if(e) return e;
    e=esp_ble_mesh_init(&prov,&comp); if(e) return e;
    return esp_ble_mesh_node_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);
}
esp_err_t mesh_link_send(uint8_t type,uint32_t id,uint8_t flags,uint8_t detail)
{
    if (!esp_ble_mesh_node_is_provisioned()) return ESP_ERR_INVALID_STATE;
    uint8_t b[8]={3,type,id,id>>8,id>>16,id>>24,flags,detail};
    esp_ble_mesh_msg_ctx_t ctx={.net_idx=TG_NET_INDEX,.app_idx=TG_APP_INDEX,
        .addr=TG_GATEWAY_ADDRESS,.send_ttl=5};
    return esp_ble_mesh_server_model_send_msg(&vendor[0],&ctx,OP_EVENT,sizeof(b),b);
}
bool mesh_link_friend_ready(void) { return atomic_load(&friend_ready); }
bool mesh_link_is_provisioned(void) { return esp_ble_mesh_node_is_provisioned(); }
