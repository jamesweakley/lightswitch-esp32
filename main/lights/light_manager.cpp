/*
 * Light manager for 1-4 on/off channels and DHT22 sensor.
 *
 * Responsibilities:
 *  - GPIO init (buttons + LEDs)
 *  - Button polling & dispatch of Toggle commands via esp-matter binding manager
 *  - Brief LED blink feedback on press
 *  - Boot-time initial remote OnOff attribute read (Phase 1) after bindings committed
 */

#include "light_manager.h"

#include <esp_log.h>
#include <string.h>
#include <memory>
#include <driver/gpio.h>
#include <esp_timer.h>
#include "esp_rom_sys.h"
#include "freertos/queue.h"
#include <platform/PlatformManager.h>

#include <inttypes.h>

#include <esp_matter.h>
// Client API for bindings and group/unicast requests
#include <esp_matter_client.h>
// CHIP cluster ids & command ids
#include <app-common/zap-generated/cluster-objects.h>
#include <app/CommandPathParams.h>
#include <app/InteractionModelEngine.h>
#include <app/ReadClient.h>
// For fabric table, CASE session manager
#include <app/server/Server.h>
#include <protocols/secure_channel/SessionEstablishmentDelegate.h>
#include <lib/core/ScopedNodeId.h>

// (Binding table internal CHIP headers not included in esp-matter public API; detailed per-target logging limited.)
// Subscription for continuous remote state tracking is a future phase (not yet implemented).

using namespace esp_matter;
using namespace esp_matter::attribute;

// Logging tag
static const char *TAG = "light_manager";
// Track last button press tick for latency diagnostics
volatile uint32_t g_last_press_tick = 0;

// static void subscriptions_init() { }

// --- State & GPIO mappings (restored) ---
static bool s_led_any_on[LIGHT_CHANNELS] = {false};
static uint8_t s_on_count[LIGHT_CHANNELS] = {0};

static const gpio_num_t s_button_gpios[LIGHT_CHANNELS] = { BUTTON_GPIO_0, BUTTON_GPIO_1, BUTTON_GPIO_2, BUTTON_GPIO_3 };
static const gpio_num_t s_led_gpios[LIGHT_CHANNELS]    = { LED_GPIO_0, LED_GPIO_1, LED_GPIO_2, LED_GPIO_3 };

// Public endpoint IDs (defined in app_main)
extern uint16_t g_onoff_endpoint_ids[LIGHT_CHANNELS];

// Button polling task handle
static TaskHandle_t s_button_task = nullptr;      // polling task (lightweight)
static TaskHandle_t s_button_act_task = nullptr;  // action task (does logging & Matter calls)
// static TaskHandle_t s_dht_task = nullptr; // future sensor task
static QueueHandle_t s_button_evt_queue = nullptr; // queue of uint8_t channel indices

// LED blink timers (one-shot) to avoid blocking delays on button tasks
static esp_timer_handle_t s_led_blink_timers[LIGHT_CHANNELS] = {nullptr};

// Forward declare
static void send_group_toggle(uint8_t ch);

static void apply_led(uint8_t ch, bool on)
{
    if (ch >= LIGHT_CHANNELS) return;
    gpio_set_level(s_led_gpios[ch], on ? 1 : 0);
}

bool light_manager_get(uint8_t channel) {
    if (channel >= LIGHT_CHANNELS) {
        return false;
    }
    return s_led_any_on[channel];
}

static void buttons_init()
{
    gpio_config_t in_cfg = {};
    in_cfg.intr_type = GPIO_INTR_DISABLE;
    in_cfg.mode = GPIO_MODE_INPUT;
    in_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    in_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    for (int i = 0; i < LIGHT_CHANNELS; i++) {
        if (s_button_gpios[i] == GPIO_NUM_NC) continue;
        in_cfg.pin_bit_mask = (1ULL << s_button_gpios[i]);
        gpio_config(&in_cfg);
        gpio_set_pull_mode(s_button_gpios[i], GPIO_PULLUP_ONLY);
    }
}

static void leds_init()
{
    gpio_config_t out_cfg = {};
    out_cfg.intr_type = GPIO_INTR_DISABLE;
    out_cfg.mode = GPIO_MODE_OUTPUT;
    out_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    out_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    for (int i = 0; i < LIGHT_CHANNELS; i++) {
        if (s_led_gpios[i] == GPIO_NUM_NC) continue;
        out_cfg.pin_bit_mask = (1ULL << s_led_gpios[i]);
        gpio_config(&out_cfg);
        gpio_set_level(s_led_gpios[i], 0);
    }
}

static void button_task(void *arg)
{
    uint8_t stable_cnt[LIGHT_CHANNELS] = {0};
    uint8_t last_read[LIGHT_CHANNELS] = {1,1,1,1};
    while (true) {
        for (int i = 0; i < LIGHT_CHANNELS; i++) {
            int level = gpio_get_level(s_button_gpios[i]);
            if (level == last_read[i]) { if (stable_cnt[i] < 255) stable_cnt[i]++; }
            else { stable_cnt[i] = 0; last_read[i] = level; }
            if (last_read[i] == 0 && stable_cnt[i] == BUTTON_STABLE_CNT) {
                uint8_t ch = i;
                if (s_button_evt_queue) xQueueSend(s_button_evt_queue, &ch, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}

static void led_blink_timer_cb(void *arg) {
    uint32_t ch = (uint32_t)arg;
    if (ch < LIGHT_CHANNELS) {
        // Restore steady LED state after transient blink
        apply_led(ch, s_led_any_on[ch]);
    }
}

// ---- Initial state sync (multi-binding) ----
namespace {
class InitialReadCallback : public chip::app::ReadClient::Callback {
public:
    explicit InitialReadCallback(uint8_t channel) : mChannel(channel) {}
    void SetClient(chip::app::ReadClient * client) { mClient = client; }
    void OnReportBegin() override { mGotAttribute = false; }
    void OnAttributeData(const chip::app::ConcreteDataAttributePath & path, chip::TLV::TLVReader * data, const chip::app::StatusIB & status) override {
        if (status.mStatus != chip::Protocols::InteractionModel::Status::Success) return;
        if (path.mClusterId != chip::app::Clusters::OnOff::Id || path.mAttributeId != chip::app::Clusters::OnOff::Attributes::OnOff::Id) return;
        bool on = false;
        if (data && data->Get(on) == CHIP_NO_ERROR) {
            mGotAttribute = true;
            bool prevAggregate = s_led_any_on[mChannel];
            bool ledChanged = false;
            if (on) {
                if (!prevAggregate) { s_led_any_on[mChannel] = true; ledChanged = true; }
            } else {
                // OFF only matters if no other target reported ON yet (aggregate semantics: ANY target ON -> LED ON)
                if (!prevAggregate) { s_led_any_on[mChannel] = false; ledChanged = false; }
            }
            apply_led(mChannel, s_led_any_on[mChannel]);
            ESP_LOGI(TAG,
                     "CH%u: initial read ep=%u OnOff=%s -> aggregate=%s (LED %s)",
                     mChannel,
                     (unsigned)path.mEndpointId,
                     on?"ON":"OFF",
                     s_led_any_on[mChannel]?"ON":"OFF",
                     ledChanged? (s_led_any_on[mChannel]?"turned ON":"turned OFF") : "unchanged");
        } else {
            ESP_LOGW(TAG, "CH%u: failed to decode initial OnOff TLV", mChannel);
        }
    }
    void OnDone(chip::app::ReadClient * client) override {
        if (!mGotAttribute) {
            ESP_LOGI(TAG, "CH%u: initial read completed with no OnOff attribute report", mChannel);
        }
        if (client) chip::Platform::Delete(client);
        chip::Platform::Delete(this);
    }
    // Unused callbacks
    void OnError(CHIP_ERROR err) override { ESP_LOGW(TAG, "CH%u: initial read IM error %" CHIP_ERROR_FORMAT, mChannel, err.Format()); }
    void OnReportEnd() override {}
    void OnSubscriptionEstablished(chip::SubscriptionId) override {}
private:
    uint8_t mChannel;
    chip::app::ReadClient * mClient = nullptr;
    bool mGotAttribute = false;
};

struct SessionCtx { uint8_t ch; chip::EndpointId ep; };

    struct PendingInitialRead { uint8_t ch; uint64_t node; chip::EndpointId ep; uint8_t fabric_index; };
    static PendingInitialRead s_pending_reads[LIGHT_CHANNELS];
    static int s_pending_count = 0;

    static void send_initial_read(const PendingInitialRead & item) {
        chip::FabricIndex fabricIndex = item.fabric_index;
        if (fabricIndex == chip::kUndefinedFabricIndex) {
            // fallback search (older saved entries may lack fabric)
            for (auto & fabricInfo : chip::Server::GetInstance().GetFabricTable()) if (fabricInfo.IsInitialized()) { fabricIndex = fabricInfo.GetFabricIndex(); break; }
        }
        if (fabricIndex == chip::kUndefinedFabricIndex) { ESP_LOGW(TAG, "CH%u: no fabric for initial read", item.ch); return; }
        auto * caseMgr = chip::Server::GetInstance().GetCASESessionManager(); if (!caseMgr) { ESP_LOGW(TAG, "CH%u: no CASE mgr", item.ch); return; }
        struct Ctx { PendingInitialRead item; };
        auto * ctx = chip::Platform::New<Ctx>(); if (!ctx) return; ctx->item = item;
        auto onConnected = [](void * c2, chip::Messaging::ExchangeManager & em, const chip::SessionHandle & sessionHandle){
            std::unique_ptr<Ctx, void(*)(Ctx*)> guard(static_cast<Ctx*>(c2), [](Ctx* p){ chip::Platform::Delete(p); });
            auto & it = guard->item;
            ESP_LOGI(TAG, "CH%u: session established node=0x%016" PRIX64, it.ch, it.node);
            auto * cb = chip::Platform::New<InitialReadCallback>(it.ch);
            if (!cb) return;
            auto * client = chip::Platform::New<chip::app::ReadClient>(chip::app::InteractionModelEngine::GetInstance(), &em, *cb, chip::app::ReadClient::InteractionType::Read);
            if (!client) { chip::Platform::Delete(cb); return; }
            chip::app::AttributePathParams attrPath; attrPath.mEndpointId = it.ep; attrPath.mClusterId = chip::app::Clusters::OnOff::Id; attrPath.mAttributeId = chip::app::Clusters::OnOff::Attributes::OnOff::Id;
            chip::app::AttributePathParams paths[1] = { attrPath };
            chip::app::ReadPrepareParams params(sessionHandle);
            params.mpAttributePathParamsList = paths; params.mAttributePathParamsListSize = 1;
            auto err = client->SendRequest(params);
            if (err != CHIP_NO_ERROR) {
                ESP_LOGW(TAG, "CH%u: SendRequest failed (%" CHIP_ERROR_FORMAT ")", it.ch, err.Format());
                chip::Platform::Delete(client); chip::Platform::Delete(cb);
            } else {
                ESP_LOGI(TAG, "CH%u: sent initial OnOff read (ep %u)", it.ch, it.ep);
            }
        };
        auto onFailure = [](void * c2, const chip::ScopedNodeId & peerId, CHIP_ERROR error){
            std::unique_ptr<Ctx, void(*)(Ctx*)> guard(static_cast<Ctx*>(c2), [](Ctx* p){ chip::Platform::Delete(p); });
            auto & it = guard->item;
            ESP_LOGW(TAG, "CH%u: session failure node=0x%016" PRIX64 " err=%" CHIP_ERROR_FORMAT, it.ch, (uint64_t)peerId.GetNodeId(), error.Format());
        };
        auto * cb1 = chip::Platform::New<chip::Callback::Callback<chip::OnDeviceConnected>>(onConnected, ctx);
        auto * cb2 = chip::Platform::New<chip::Callback::Callback<chip::OnDeviceConnectionFailure>>(onFailure, ctx);
        if (!cb1 || !cb2) { if (cb1) chip::Platform::Delete(cb1); if (cb2) chip::Platform::Delete(cb2); chip::Platform::Delete(ctx); return; }
        chip::ScopedNodeId scoped(item.node, fabricIndex);
        ESP_LOGI(TAG, "CH%u: establishing session to node 0x%016" PRIX64 " ep %u", item.ch, (uint64_t)item.node, (unsigned)item.ep);
        caseMgr->FindOrEstablishSession(scoped, cb1, cb2);
    }

    static void schedule_single_initial_read(uint8_t ch, const ShadowBindingEntry & entry, uint32_t delay_ms) {
        if (entry.is_group) return;
        if (s_pending_count >= (int)(sizeof(s_pending_reads)/sizeof(s_pending_reads[0]))) return;
    s_pending_reads[s_pending_count++] = PendingInitialRead{ch, entry.node_id, (chip::EndpointId)entry.endpoint, entry.fabric_index};
        // Use a one-shot timer to allow staggered start and give DNS-SD time.
        struct TimerCtx { PendingInitialRead item; };
        auto * tctx = chip::Platform::New<TimerCtx>(); if (!tctx) return; tctx->item = s_pending_reads[s_pending_count-1];
        chip::DeviceLayer::SystemLayer().StartTimer(chip::System::Clock::Milliseconds32(delay_ms), [](chip::System::Layer*, void * arg){
            auto * t = static_cast<TimerCtx*>(arg);
            chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t a){
                auto * t2 = reinterpret_cast<TimerCtx*>(a);
                send_initial_read(t2->item);
                chip::Platform::Delete(t2);
            }, reinterpret_cast<intptr_t>(t));
        }, tctx);
    }

} // anonymous namespace

void light_manager_sync_initial_state() {
    // Baseline: assume OFF; will OR-in any ON from responses
    for (int ch = 0; ch < LIGHT_CHANNELS; ch++) { s_led_any_on[ch] = false; apply_led(ch, false); }
    // NOTE: Previously skipped small node IDs; re-enabled reads to recover LED state even if node IDs look small.
    // To cap timeout impact, only schedule the first unicast per channel for now.
    // Schedule reads per unicast binding
    for (int ch = 0; ch < LIGHT_CHANNELS; ch++) {
        auto * list = shadow_binding_get_list(ch);
        if (!list || list->count == 0) { ESP_LOGI(TAG, "CH%u: no bindings; skip initial read", ch); continue; }
        bool any=false;
        for (int i=0;i<list->count;i++) {
            const auto & e = list->entries[i];
            if (e.is_group) continue; // only unicast for now
            any = true;
            schedule_single_initial_read(ch, e, 0);
        }
        if (!any) ESP_LOGI(TAG, "CH%u: only group bindings; skip initial read", ch);
    }
    // Nothing else to do; timers will fire and schedule reads.
}

// ---- Button press handling ----
void light_manager_button_press(uint8_t channel) {
    if (channel >= LIGHT_CHANNELS) return;
    g_last_press_tick = (uint32_t)xTaskGetTickCount();
    // Diagnostics: log current shadow binding summary for this channel before dispatch
    const ShadowBindingList * list = shadow_binding_get_list(channel);
    int unicastCount=0; if (list) { for (int i=0;i<list->count;i++) if(!list->entries[i].is_group) unicastCount++; }
    ESP_LOGI(TAG, "Button press CH%u (unicast bindings=%d)", channel, unicastCount);
    // Transient blink: invert quickly then restore
    if (s_led_gpios[channel] != GPIO_NUM_NC) {
        apply_led(channel, !s_led_any_on[channel]);
        // create one-shot timer if not exists
        if (!s_led_blink_timers[channel]) {
            esp_timer_create_args_t args = { .callback = &led_blink_timer_cb, .arg = (void*)(uintptr_t)channel, .dispatch_method = ESP_TIMER_TASK, .name = "ledblink" };
            esp_timer_create(&args, &s_led_blink_timers[channel]);
        }
        if (s_led_blink_timers[channel]) esp_timer_start_once(s_led_blink_timers[channel], 40 * 1000); // 40ms
    }
    send_group_toggle(channel);
}

// ---- Command dispatch helpers ----
static void send_group_toggle(uint8_t ch) {
    if (ch >= LIGHT_CHANNELS) return;
    // Optimistic local state flip for steady LED feedback until remote subscription implemented.
    s_led_any_on[ch] = !s_led_any_on[ch];
    apply_led(ch, s_led_any_on[ch]);
    // Defer actual Matter command to CHIP thread to satisfy stack lock requirements.
    chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t arg){
        uint8_t ch_i = static_cast<uint8_t>(arg);
        esp_matter::client::request_handle req = {};
        chip::app::CommandPathParams path(
            g_onoff_endpoint_ids[ch_i], /* group id */ 0,
            chip::app::Clusters::OnOff::Id,
            chip::app::Clusters::OnOff::Commands::Toggle::Id,
            static_cast<chip::app::CommandPathFlags>(0));
        req.command_path = path;
        esp_err_t err = esp_matter::client::cluster_update(g_onoff_endpoint_ids[ch_i], &req);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "cluster_update failed (scheduled) for ch%u (ep %u): %d", ch_i, g_onoff_endpoint_ids[ch_i], err);
        } else {
            ESP_LOGI(TAG, "CH%u: Toggle dispatched (scheduled)", ch_i);
        }
    }, static_cast<intptr_t>(ch));
}

// ---- Init ----
esp_err_t light_manager_init() {
    buttons_init();
    leds_init();
    s_button_evt_queue = xQueueCreate(8, sizeof(uint8_t));
    if (!s_button_evt_queue) return ESP_ERR_NO_MEM;
    // Button polling task: low priority, small stack
    xTaskCreate(button_task, "btn_poll", 2048, nullptr, tskIDLE_PRIORITY+1, &s_button_task); // latency <100ms requirement ok with poll interval
    // Action task: process button events and dispatch cluster updates
    auto action_task = [](void * arg){
        uint8_t ch;
        while (true) {
            if (xQueueReceive(s_button_evt_queue, &ch, portMAX_DELAY) == pdTRUE) {
                light_manager_button_press(ch);
            }
        }
    };
    xTaskCreate(action_task, "btn_act", 3072, nullptr, tskIDLE_PRIORITY+2, &s_button_act_task);
    ESP_LOGI(TAG, "Light manager init complete");
    return ESP_OK;
}

// DHT22 sensor task start stub (implemented elsewhere or future)
void dht22_start_task() {}

