/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_timer.h>
#if CONFIG_PM_ENABLE
#include <esp_pm.h>
#endif

#include <inttypes.h>

#include <esp_matter.h>
#include <esp_matter_console.h>
#include <esp_matter_ota.h>
// For binding manager init and controller helpers
#include <esp_matter_client.h>
// CHIP cluster ids & command ids
#include <app-common/zap-generated/cluster-objects.h>
// TLV utilities (header name differs across versions: prefer TLV.h)
#include <lib/core/TLV.h>

#include <common_macros.h>
#include <app_priv.h>
#include "app_config.h"
#include "lights/light_manager.h"
#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
#include <platform/ESP32/OpenthreadLauncher.h>
#endif

#include <app/server/CommissioningWindowManager.h>
#include <app/server/Server.h>

static const char *TAG = "app_main";

// Define globals declared in light_manager.h
uint16_t g_onoff_endpoint_ids[LIGHT_CHANNELS] = {0};
uint16_t g_temp_endpoint_id = 0;
uint16_t g_humidity_endpoint_id = 0;
// Prevent light sleep during debug to keep USB Serial/JTAG stable
#if CONFIG_PM_ENABLE
static esp_pm_lock_handle_t s_pm_no_ls_lock = nullptr;
#endif
// Endpoint IDs for our new device
extern uint16_t g_onoff_endpoint_ids[LIGHT_CHANNELS];
// Instrumentation counters for request callbacks
volatile uint32_t g_reqcb_unicast_count = 0;
volatile uint32_t g_reqcb_group_count = 0;
extern uint16_t g_temp_endpoint_id;
extern uint16_t g_humidity_endpoint_id;

// Watchdog timer for detecting stuck initialization
static esp_timer_handle_t init_watchdog_timer = NULL;
static bool matter_started = false;

// ---- Shadow Binding Structures ----
struct ShadowBindingEntry {
    bool is_group; // false = unicast
    uint64_t node_id;
    uint16_t endpoint;
    uint32_t cluster_id; // typically 0x0006
    uint16_t group_id;   // valid if is_group
};

static constexpr int MAX_SHADOW_BINDINGS_PER_CH = 10;
struct ShadowBindingList {
    int count;
    ShadowBindingEntry entries[MAX_SHADOW_BINDINGS_PER_CH];
};
static ShadowBindingList s_shadow_lists[LIGHT_CHANNELS];

// Lightweight accessor so other compilation units (light_manager) can iterate shadow entries for unicast.
extern "C" const ShadowBindingList * shadow_binding_get_list(int ch) {
    if (ch < 0 || ch >= LIGHT_CHANNELS) return nullptr;
    return &s_shadow_lists[ch];
}

// NVS namespace & key pattern
static const char *k_bind_nvs_namespace = "bindcfg";

static esp_err_t shadow_binding_save_nvs(int ch) {
    if (ch < 0 || ch >= LIGHT_CHANNELS) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t err = nvs_open(k_bind_nvs_namespace, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    char key[8];
    snprintf(key, sizeof(key), "ch%d", ch);
    err = nvs_set_blob(h, key, &s_shadow_lists[ch], sizeof(ShadowBindingList));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Saved shadow bindings ch%d to NVS (count=%d)", ch, s_shadow_lists[ch].count);
    } else {
        ESP_LOGE(TAG, "Failed saving shadow bindings ch%d err=%d", ch, (int)err);
    }
    return err;
}

static esp_err_t shadow_binding_load_nvs(int ch) {
    if (ch < 0 || ch >= LIGHT_CHANNELS) return ESP_ERR_INVALID_ARG;
    nvs_handle_t h;
    esp_err_t err = nvs_open(k_bind_nvs_namespace, NVS_READONLY, &h);
    if (err != ESP_OK) return err;
    char key[8];
    snprintf(key, sizeof(key), "ch%d", ch);
    size_t len = sizeof(ShadowBindingList);
    ShadowBindingList tmp = {};
    err = nvs_get_blob(h, key, &tmp, &len);
    nvs_close(h);
    if (err == ESP_OK && len == sizeof(ShadowBindingList)) {
        s_shadow_lists[ch] = tmp;
        if (s_shadow_lists[ch].count > MAX_SHADOW_BINDINGS_PER_CH) s_shadow_lists[ch].count = 0; // sanitize
        ESP_LOGI(TAG, "Loaded shadow bindings ch%d from NVS (count=%d)", ch, s_shadow_lists[ch].count);
    }
    return err;
}

static void shadow_binding_load_all_nvs() {
    for (int ch = 0; ch < LIGHT_CHANNELS; ch++) {
        shadow_binding_load_nvs(ch);
    }
}

extern "C" void shadow_binding_clear_channel(int ch) {
    if (ch < 0 || ch >= LIGHT_CHANNELS) return;
    s_shadow_lists[ch].count = 0;
}

static void shadow_binding_log(int ch) {
    if (ch < 0 || ch >= LIGHT_CHANNELS) return;
    ESP_LOGI(TAG, "Shadow bindings ch%u count=%d", ch, s_shadow_lists[ch].count);
    for (int i = 0; i < s_shadow_lists[ch].count; i++) {
        auto &e = s_shadow_lists[ch].entries[i];
        if (e.is_group) {
            ESP_LOGI(TAG, "  [%d] GROUP 0x%04X", i, e.group_id);
        } else {
            ESP_LOGI(TAG, "  [%d] UNICAST Node=0x%016" PRIX64 " EP=%u Cl=0x%04X", i, (uint64_t)e.node_id, e.endpoint, (unsigned)e.cluster_id);
        }
    }
}

static esp_err_t shadow_binding_add_unicast(int ch, uint64_t node_id, uint16_t endpoint, uint32_t cluster_id) {
    if (ch < 0 || ch >= LIGHT_CHANNELS) return ESP_ERR_INVALID_ARG;
    auto &list = s_shadow_lists[ch];
    // Dedup
    for (int i = 0; i < list.count; i++) {
        auto &e = list.entries[i];
        if (!e.is_group && e.node_id == node_id && e.endpoint == endpoint && e.cluster_id == cluster_id) {
            ESP_LOGI(TAG, "Shadow binding already present");
            return ESP_OK;
        }
    }
    if (list.count >= MAX_SHADOW_BINDINGS_PER_CH) return ESP_ERR_NO_MEM;
    auto &e = list.entries[list.count++];
    e.is_group = false; e.node_id = node_id; e.endpoint = endpoint; e.cluster_id = cluster_id; e.group_id = 0;
    return ESP_OK;
}

// Rewrites Binding attribute with combined existing (shadow) + new entry.
static esp_err_t shadow_binding_commit(int ch) {
    if (ch < 0 || ch >= LIGHT_CHANNELS) return ESP_ERR_INVALID_ARG;
    uint16_t ep = g_onoff_endpoint_ids[ch];
    ESP_LOGI(TAG, "Committing shadow bindings (Option C external writes) -> ep %u entries=%d", ep, s_shadow_lists[ch].count);
    shadow_binding_log(ch);
    // Persist shadow
    shadow_binding_save_nvs(ch);
    return ESP_OK;
}

// Console command to add unicast binding: bind-add <channel> <node_id_hex> <endpoint> [cluster=0x0006]
static int cmd_bind_add(int argc, char **argv) {
    if (argc < 4) {
        printf("Usage: bind-add <channel 0-%d> <node-id-hex> <endpoint> [cluster-hex]\n", LIGHT_CHANNELS-1);
        return 0;
    }
    int ch = atoi(argv[1]);
    uint64_t node_id = strtoull(argv[2], nullptr, 16);
    uint16_t ep = (uint16_t)strtoul(argv[3], nullptr, 0);
    uint32_t cluster = 0x0006;
    if (argc > 4) cluster = (uint32_t)strtoul(argv[4], nullptr, 16);
    if (shadow_binding_add_unicast(ch, node_id, ep, cluster) == ESP_OK) {
        shadow_binding_commit(ch);
    }
    return 0;
}

// bind-list [ch]
static int cmd_bind_list(int argc, char **argv) {
    if (argc == 1) {
        for (int ch = 0; ch < LIGHT_CHANNELS; ch++) shadow_binding_log(ch);
        return 0;
    }
    int ch = atoi(argv[1]);
    shadow_binding_log(ch);
    return 0;
}

// bind-remove <ch> <index>
static int cmd_bind_remove(int argc, char **argv) {
    if (argc < 3) { printf("Usage: bind-remove <channel> <index>\n"); return 0; }
    int ch = atoi(argv[1]);
    int idx = atoi(argv[2]);
    if (ch < 0 || ch >= LIGHT_CHANNELS) return 0;
    auto &list = s_shadow_lists[ch];
    if (idx < 0 || idx >= list.count) { printf("Index out of range\n"); return 0; }
    for (int i = idx; i < list.count - 1; i++) list.entries[i] = list.entries[i+1];
    list.count--;
    shadow_binding_commit(ch);
    return 0;
}

// bind-clear <ch>
static int cmd_bind_clear(int argc, char **argv) {
    if (argc < 2) { printf("Usage: bind-clear <channel>\n"); return 0; }
    int ch = atoi(argv[1]);
    shadow_binding_clear_channel(ch);
    shadow_binding_commit(ch);
    return 0;
}

// bind-commit <ch>
static int cmd_bind_commit(int argc, char **argv) {
    if (argc < 2) { printf("Usage: bind-commit <channel>\n"); return 0; }
    int ch = atoi(argv[1]);
    shadow_binding_commit(ch);
    return 0;
}

static void init_watchdog_callback(void* arg)
{
    if (!matter_started) {
        ESP_LOGE(TAG, "Matter initialization appears stuck, restarting device...");
        esp_restart();
    }
}

using namespace esp_matter;
using namespace esp_matter::attribute;
using namespace esp_matter::endpoint;
using namespace chip::app::Clusters;
using namespace chip;

constexpr auto k_timeout_seconds = 300;

#if CONFIG_ENABLE_ENCRYPTED_OTA
extern const char decryption_key_start[] asm("_binary_esp_image_encryption_key_pem_start");
extern const char decryption_key_end[] asm("_binary_esp_image_encryption_key_pem_end");

static const char *s_decryption_key = decryption_key_start;
static const uint16_t s_decryption_key_len = decryption_key_end - decryption_key_start;
#endif // CONFIG_ENABLE_ENCRYPTED_OTA

static void app_event_cb(const ChipDeviceEvent *event, intptr_t arg)
{
    // No need to check for contact sensor updates here anymore
    // Updates are now scheduled directly on the Matter thread
    
    switch (event->Type) {
    case chip::DeviceLayer::DeviceEventType::kInterfaceIpAddressChanged:
        ESP_LOGI(TAG, "Interface IP Address changed");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningComplete:
        ESP_LOGI(TAG, "Commissioning complete");
        break;

    case chip::DeviceLayer::DeviceEventType::kFailSafeTimerExpired:
        ESP_LOGI(TAG, "Commissioning failed, fail safe timer expired");
        // Try to reopen commissioning window after failure
        {
            chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
            constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);
            if (!commissionMgr.IsCommissioningWindowOpen())
            {
                CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(kTimeoutSeconds,
                                                chip::CommissioningWindowAdvertisement::kDnssdOnly);
                if (err != CHIP_NO_ERROR)
                {
                    ESP_LOGE(TAG, "Failed to reopen commissioning window after fail safe, err:%" CHIP_ERROR_FORMAT, err.Format());
                }
                else
                {
                    ESP_LOGI(TAG, "Reopened commissioning window after fail safe timer expired");
                }
            }
        }
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStarted:
        ESP_LOGI(TAG, "Commissioning session started");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningSessionStopped:
        ESP_LOGI(TAG, "Commissioning session stopped");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowOpened:
        ESP_LOGI(TAG, "Commissioning window opened");
        break;

    case chip::DeviceLayer::DeviceEventType::kCommissioningWindowClosed:
        ESP_LOGI(TAG, "Commissioning window closed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricRemoved:
        {
            ESP_LOGI(TAG, "Fabric removed successfully");
            if (chip::Server::GetInstance().GetFabricTable().FabricCount() == 0)
            {
                chip::CommissioningWindowManager & commissionMgr = chip::Server::GetInstance().GetCommissioningWindowManager();
                constexpr auto kTimeoutSeconds = chip::System::Clock::Seconds16(k_timeout_seconds);
                if (!commissionMgr.IsCommissioningWindowOpen())
                {
                    /* After removing last fabric, this example does not remove the Wi-Fi credentials
                     * and still has IP connectivity so, only advertising on DNS-SD.
                     */
                    CHIP_ERROR err = commissionMgr.OpenBasicCommissioningWindow(kTimeoutSeconds,
                                                    chip::CommissioningWindowAdvertisement::kDnssdOnly);
                    if (err != CHIP_NO_ERROR)
                    {
                        ESP_LOGE(TAG, "Failed to open commissioning window, err:%" CHIP_ERROR_FORMAT, err.Format());
                    }
                }
            }
        break;
        }

    case chip::DeviceLayer::DeviceEventType::kFabricWillBeRemoved:
        ESP_LOGI(TAG, "Fabric will be removed");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricUpdated:
        ESP_LOGI(TAG, "Fabric is updated");
        break;

    case chip::DeviceLayer::DeviceEventType::kFabricCommitted:
        ESP_LOGI(TAG, "Fabric is committed");
        break;

    case chip::DeviceLayer::DeviceEventType::kBLEDeinitialized:
        ESP_LOGI(TAG, "BLE deinitialized and memory reclaimed");
        break;
        
    // Matter connection events
    case chip::DeviceLayer::DeviceEventType::kCHIPoBLEConnectionEstablished:
        ESP_LOGI(TAG, "BLE connection established");
        break;
        
    case chip::DeviceLayer::DeviceEventType::kCHIPoBLEConnectionClosed:
        ESP_LOGI(TAG, "BLE connection closed");
        break;
        
    // Handle BLE advertising errors
    case chip::DeviceLayer::DeviceEventType::kCHIPoBLEAdvertisingChange:
        ESP_LOGI(TAG, "BLE advertising state changed");
        break;
        
    case chip::DeviceLayer::DeviceEventType::kSecureSessionEstablished:
        ESP_LOGI(TAG, "Secure session established");
        break;
        
    // Thread/OpenThread events
    case chip::DeviceLayer::DeviceEventType::kThreadConnectivityChange:
        ESP_LOGI(TAG, "Thread connectivity changed");
        break;
        
    case chip::DeviceLayer::DeviceEventType::kThreadStateChange:
        ESP_LOGI(TAG, "Thread state changed");
        break;
        
    // Add error handling for connectivity issues
    case chip::DeviceLayer::DeviceEventType::kDnssdInitialized:
        ESP_LOGI(TAG, "DNS-SD initialized");
        break;

    default:
        // Log unknown events for debugging
        ESP_LOGD(TAG, "Unhandled device event: %d", event->Type);
        break;
    }
}

// This callback is invoked when clients interact with the Identify Cluster.
// In the callback implementation, an endpoint can identify itself. (e.g., by flashing an LED or light).
static esp_err_t app_identification_cb(identification::callback_type_t type, uint16_t endpoint_id, uint8_t effect_id,
                                       uint8_t effect_variant, void *priv_data)
{
    ESP_LOGI(TAG, "Identification callback: type: %u, effect: %u, variant: %u", type, effect_id, effect_variant);
    return ESP_OK;
}

// This callback is called for every attribute update. The callback implementation shall
// handle the desired attributes and return an appropriate error code. If the attribute
// is not of your interest, please do not return an error code and strictly return ESP_OK.
// Custom attribute callback for contact sensor
// For controller mode, we do not mirror local On/Off server state to hardware.

static esp_err_t app_attribute_update_cb(attribute::callback_type_t type, uint16_t endpoint_id, uint32_t cluster_id,
                                         uint32_t attribute_id, esp_matter_attr_val_t *val, void *priv_data)
{
    esp_err_t err = ESP_OK;

    // ---- Shadow Binding handling ----
    // We keep a per-endpoint shadow list of unicast binding targets so we can append via a helper command.
    // NOTE: Full TLV parsing of the Binding list is not implemented here (esp-matter public API limitation);
    // you can extend this later when attribute value decoding for lists is exposed.

    constexpr uint32_t kBindingClusterId = chip::app::Clusters::Binding::Id; // 0xF000
    constexpr uint32_t kBindingAttrId = 0x0000;

    if (cluster_id == kBindingClusterId && attribute_id == kBindingAttrId) {
        if (type == attribute::PRE_UPDATE) {
            ESP_LOGI(TAG, "Binding PRE_UPDATE ep=%u (incoming list replaces shadow)", endpoint_id);
        } else if (type == attribute::POST_UPDATE) {
            ESP_LOGI(TAG, "Binding POST_UPDATE ep=%u (shadow invalidated -> mark dirty)", endpoint_id);
            // Mark shadow dirty so next 'add' command will rebuild from NVS or assume empty.
            // (Simplified: we just clear it.)
            for (int ch = 0; ch < LIGHT_CHANNELS; ch++) {
                if (g_onoff_endpoint_ids[ch] == endpoint_id) {
                    // Clear shadow entry count (defined later) via extern helper
                    extern void shadow_binding_clear_channel(int ch);
                    shadow_binding_clear_channel(ch);
                    break;
                }
            }
        }
        return err;
    }

    // Interpret important attribute updates for easier debugging
    if (cluster_id == chip::app::Clusters::Binding::Id) {
        // Binding cluster writes/reads. Attribute 0x0000 is the Binding table list.
        if (type == attribute::PRE_UPDATE) {
            ESP_LOGI(TAG, "Binding attribute PRE_UPDATE: ep=%u attr=0x%08" PRIx32, endpoint_id, attribute_id);
        } else if (type == attribute::POST_UPDATE) {
            ESP_LOGI(TAG, "Binding attribute POST_UPDATE: ep=%u attr=0x%08" PRIx32, endpoint_id, attribute_id);
        } else {
            ESP_LOGI(TAG, "Binding attribute CB: ep=%u attr=0x%08" PRIx32 " type=%d", endpoint_id, attribute_id, (int)type);
        }
        return err;
    }

    // No local attribute mirroring in controller mode

    return err;
}

extern "C" void app_main()
{
    // Ensure logging is visible as early as possible
    ESP_EARLY_LOGI(TAG, "app_main start (build %s %s)", __DATE__, __TIME__);
    // Set global log level to INFO (and our tags explicitly) in case sdkconfig differs
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("app_main", ESP_LOG_INFO);
    esp_log_level_set("light_manager", ESP_LOG_INFO);
    esp_log_level_set("BindingManager", ESP_LOG_DEBUG);
    esp_log_level_set("IM", ESP_LOG_DEBUG);
    esp_log_level_set("CommandSender", ESP_LOG_DEBUG);
    esp_log_level_set("ExchangeMgr", ESP_LOG_DEBUG);

    esp_err_t err = ESP_OK;

#if CONFIG_PM_ENABLE
    // Acquire a PM lock to disable light sleep; helps OpenOCD keep JTAG connected
    if (s_pm_no_ls_lock == nullptr) {
        esp_err_t lerr = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "debug", &s_pm_no_ls_lock);
        if (lerr == ESP_OK) {
            esp_pm_lock_acquire(s_pm_no_ls_lock);
            ESP_LOGW(TAG, "Light sleep disabled via PM lock for debugging");
        } else {
            ESP_LOGW(TAG, "Failed to create PM lock, err=%d", (int)lerr);
        }
    }
#endif

    /* Initialize the ESP NVS layer */
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition was truncated and needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "NVS initialized successfully");
    
    // Clear BLE bonding data to resolve "Failed to restore IRKs from store" errors
    // This is necessary when the BLE bonding data becomes corrupted
    nvs_handle_t nvs_handle;
    err = nvs_open("bt_cfg", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        size_t required_size = 0;
        err = nvs_get_blob(nvs_handle, "bt_cfg", NULL, &required_size);
        if (err == ESP_OK && required_size > 0) {
            ESP_LOGI(TAG, "Found existing BLE configuration data (%d bytes), clearing to prevent IRK errors", required_size);
            nvs_erase_key(nvs_handle, "bt_cfg");
            ESP_LOGI(TAG, "Cleared potentially corrupted BLE configuration");
        }
        nvs_close(nvs_handle);
    }
    
    // Also clear other BLE-related NVS keys that might be corrupted
    err = nvs_open("nimble_bond", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_erase_all(nvs_handle);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "Cleared NimBLE bonding data");
    }
    
    err = nvs_open("bt_config", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_erase_all(nvs_handle);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "Cleared BT config data");
    }

#if CONFIG_PM_ENABLE
    esp_pm_config_t pm_config = {
        .max_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
        .min_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true
#endif
    };
    err = esp_pm_configure(&pm_config);
#endif

    /* Create a Matter node and add the mandatory Root Node device type on endpoint 0 */
    node::config_t node_config;

    // node handle can be used to add/modify other endpoints.
    node_t *node = node::create(&node_config, app_attribute_update_cb, app_identification_cb);
    ABORT_APP_ON_FAILURE(node != nullptr, ESP_LOGE(TAG, "Failed to create Matter node"));

    // Create up to 4 On/Off Light Switch controller endpoints (OnOff CLIENT + Binding SERVER + Binding CLIENT)
    for (int i = 0; i < LIGHT_CHANNELS; i++) {
        // Use generic endpoint creator then add desired clusters manually for clarity.
        endpoint_t *ep = endpoint::create(node, ENDPOINT_FLAG_NONE, NULL);
        ABORT_APP_ON_FAILURE(ep != nullptr, ESP_LOGE(TAG, "Failed to create endpoint for switch %d", i));
        g_onoff_endpoint_ids[i] = endpoint::get_id(ep);
        // Add device type: On/Off Light Switch (0x0103) so ecosystems show a switch, not a lamp.
        endpoint::add_device_type(ep, 0x0103, 1 /*rev*/);
    // Add OnOff client cluster (needs config struct, then flags)
    cluster::on_off::config_t onoff_cfg = {};
    cluster_t *onoff_client = cluster::on_off::create(ep, &onoff_cfg, CLUSTER_FLAG_CLIENT, 0);
        (void)onoff_client;
    // Add Binding server & client clusters (client will allow issuing Bind/Unbind commands later)
    cluster::common::config_t binding_srv_cfg = {};
    cluster_t *binding_srv = cluster::binding::create(ep, &binding_srv_cfg, CLUSTER_FLAG_SERVER);
        (void)binding_srv;
    cluster::common::config_t binding_cli_cfg = {};
    cluster_t *binding_cli = cluster::binding::create(ep, &binding_cli_cfg, CLUSTER_FLAG_CLIENT);
        (void)binding_cli;
        ESP_LOGI(TAG, "Switch channel %d endpoint_id=%d (OnOff client)", i, g_onoff_endpoint_ids[i]);
    }

    // Create Temperature and Humidity sensor endpoints
    {
        temperature_sensor::config_t tcfg = {};
        endpoint_t *tep = temperature_sensor::create(node, &tcfg, ENDPOINT_FLAG_NONE, NULL);
        ABORT_APP_ON_FAILURE(tep != nullptr, ESP_LOGE(TAG, "Failed to create temperature sensor endpoint"));
        g_temp_endpoint_id = endpoint::get_id(tep);
        ESP_LOGI(TAG, "Temperature sensor endpoint_id=%d", g_temp_endpoint_id);
    }
    {
        humidity_sensor::config_t hcfg = {};
        endpoint_t *hep = humidity_sensor::create(node, &hcfg, ENDPOINT_FLAG_NONE, NULL);
        ABORT_APP_ON_FAILURE(hep != nullptr, ESP_LOGE(TAG, "Failed to create humidity sensor endpoint"));
        g_humidity_endpoint_id = endpoint::get_id(hep);
        ESP_LOGI(TAG, "Humidity sensor endpoint_id=%d", g_humidity_endpoint_id);
    }

#if CHIP_DEVICE_CONFIG_ENABLE_THREAD
    /* Set OpenThread platform config */
    esp_openthread_platform_config_t config = {
        .radio_config = ESP_OPENTHREAD_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_OPENTHREAD_DEFAULT_HOST_CONFIG(),
        .port_config = ESP_OPENTHREAD_DEFAULT_PORT_CONFIG(),
    };
    set_openthread_platform_config(&config);
#endif

    // Initialize local drivers (buttons/LEDs) and sensor task
    light_manager_init();

    // Load any persisted shadow bindings before starting Matter (will commit after start)
    shadow_binding_load_all_nvs();

    // Start a watchdog timer to detect if Matter initialization gets stuck
    esp_timer_create_args_t timer_args = {
        .callback = init_watchdog_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "init_watchdog"
    };
    esp_timer_create(&timer_args, &init_watchdog_timer);
    esp_timer_start_once(init_watchdog_timer, 30000000); // 30 seconds timeout
    ESP_LOGI(TAG, "Started initialization watchdog timer (30s timeout)");

    /* Matter start */
    err = esp_matter::start(app_event_cb);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to start Matter, err:%d", err));
    // Initialize binding manager so cluster_update works for group/unicast
    esp_matter::client::binding_manager_init();
    // Install request callbacks to observe unicast/group routing decisions
    esp_matter::client::set_request_callback(
        [](chip::DeviceProxy * device, esp_matter::client::request_handle * req, void * priv){
            if (!device || !req) return;
            chip::NodeId nid = device->GetDeviceId();
            g_reqcb_unicast_count++;
            extern volatile uint32_t g_last_press_tick; // from light_manager
            uint32_t now = xTaskGetTickCount();
            uint32_t delta_ms = (now - g_last_press_tick) * portTICK_PERIOD_MS;
            ESP_LOGI("ReqCB","UNCAST node=0x%016" PRIx64 " ep=%u cluster=0x%08" PRIx32 " cmd=0x%08" PRIx32 " latency=%ums -> send",
                     (uint64_t)nid,
                     (unsigned)req->command_path.mEndpointId,
                     (uint32_t)req->command_path.mClusterId,
                     (uint32_t)req->command_path.mCommandId,
                     (unsigned)delta_ms);
            // Only process OnOff::Toggle
            if (req->command_path.mClusterId != chip::app::Clusters::OnOff::Id ||
                req->command_path.mCommandId != chip::app::Clusters::OnOff::Commands::Toggle::Id) {
                return; }
            // Include necessary headers locally
            using namespace chip::app;
            class ToggleSenderCallback : public CommandSender::Callback {
            public:
                void OnResponse(CommandSender * cs, const ConcreteCommandPath & path, const StatusIB & status, TLV::TLVReader * data) override {
                    ESP_LOGI("ToggleSend","Resp ep=%u status=0x%02X", (unsigned)path.mEndpointId, (unsigned)status.mStatus);
                }
                void OnError(const CommandSender * cs, CHIP_ERROR err) override {
                    ESP_LOGE("ToggleSend","Error %" CHIP_ERROR_FORMAT, err.Format());
                }
                void OnDone(CommandSender * cs) override {
                    chip::Platform::Delete(cs);
                    chip::Platform::Delete(this);
                }
            };
            auto * cb = chip::Platform::New<ToggleSenderCallback>();
            if (!cb) { ESP_LOGE("ToggleSend","No mem cb"); return; }
            auto * sender = chip::Platform::New<CommandSender>(cb, InteractionModelEngine::GetInstance()->GetExchangeManager());
            if (!sender) { ESP_LOGE("ToggleSend","No mem sender"); chip::Platform::Delete(cb); return; }
            CommandPathParams cp(req->command_path.mEndpointId,
                                  /*group*/ 0,
                                  chip::app::Clusters::OnOff::Id,
                                  chip::app::Clusters::OnOff::Commands::Toggle::Id,
                                  CommandPathFlags::kEndpointIdValid);
            CHIP_ERROR cerr = sender->PrepareCommand(cp);
            if (cerr != CHIP_NO_ERROR) { ESP_LOGE("ToggleSend","Prepare fail %" CHIP_ERROR_FORMAT, cerr.Format()); goto fail; }
            cerr = sender->FinishCommand();
            if (cerr != CHIP_NO_ERROR) { ESP_LOGE("ToggleSend","Finish fail %" CHIP_ERROR_FORMAT, cerr.Format()); goto fail; }
            {
                auto session = device->GetSecureSession();
                if (!session.HasValue()) { ESP_LOGE("ToggleSend","No session"); goto fail; }
                cerr = sender->SendCommandRequest(session.Value());
                if (cerr != CHIP_NO_ERROR) { ESP_LOGE("ToggleSend","Send fail %" CHIP_ERROR_FORMAT, cerr.Format()); goto fail; }
            }
            ESP_LOGI("ToggleSend","Sent Toggle to node=0x%016" PRIx64 " ep=%u", (uint64_t)nid, (unsigned)req->command_path.mEndpointId);
            return;
        fail:
            chip::Platform::Delete(sender);
            chip::Platform::Delete(cb);
        },
        [](uint8_t group_id, esp_matter::client::request_handle * req, void * priv){
            if (!req) return;
            g_reqcb_group_count++;
            ESP_LOGI("ReqCB","GROUP gid=%u ep=%u cluster=0x%08" PRIx32 " cmd=0x%08" PRIx32, (unsigned)group_id,
                     (unsigned)req->command_path.mEndpointId, (uint32_t)req->command_path.mClusterId, (uint32_t)req->command_path.mCommandId);
        },
        nullptr);
    // Commit any restored shadow bindings to live Binding attribute (placeholder writer)
    for (int ch = 0; ch < LIGHT_CHANNELS; ch++) {
        if (s_shadow_lists[ch].count > 0) shadow_binding_commit(ch);
    }

    // Periodic instrumentation log (every 30s) for request callback counters
    static esp_timer_handle_t reqcb_timer;
    esp_timer_create_args_t reqcb_args = {
        .callback = [](void*){
            ESP_LOGI("ReqCB","Counts: unicast=%lu group=%lu", (unsigned long)g_reqcb_unicast_count, (unsigned long)g_reqcb_group_count);
        },
        .arg = nullptr,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "reqcb_tmr"
    };
    if (esp_timer_create(&reqcb_args, &reqcb_timer) == ESP_OK) {
        esp_timer_start_periodic(reqcb_timer, 30000000); // 30s
    }
    
    // Start DHT22 task after Matter start
    dht22_start_task();

    // Mark that Matter has started successfully
    matter_started = true;
    if (init_watchdog_timer) {
        esp_timer_stop(init_watchdog_timer);
        esp_timer_delete(init_watchdog_timer);
        init_watchdog_timer = NULL;
        ESP_LOGI(TAG, "Matter started successfully, stopped watchdog timer");
    }

#if CONFIG_ENABLE_ENCRYPTED_OTA
    err = esp_matter_ota_requestor_encrypted_init(s_decryption_key, s_decryption_key_len);
    ABORT_APP_ON_FAILURE(err == ESP_OK, ESP_LOGE(TAG, "Failed to initialized the encrypted OTA, err: %d", err));
#endif // CONFIG_ENABLE_ENCRYPTED_OTA

#if CONFIG_ENABLE_CHIP_SHELL
    esp_matter::console::diagnostics_register_commands();
    esp_matter::console::wifi_register_commands();
    esp_matter::console::factoryreset_register_commands();
#if CONFIG_OPENTHREAD_CLI
    esp_matter::console::otcli_register_commands();
#endif
    esp_matter::console::init();
#endif
}
