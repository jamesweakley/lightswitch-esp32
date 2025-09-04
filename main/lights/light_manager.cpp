/* Clean replacement file (corruption fixed). */
#include "light_manager.h"
#include <esp_log.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include "freertos/queue.h"
#include <app/ReadClient.h>
#include <app/InteractionModelEngine.h>
#include <app/server/Server.h>
#include <esp_matter.h>
#include "../temp/temp_manager.h"  // sensor task now lives in temp module
#include <esp_matter_client.h>
#include <platform/PlatformManager.h>
#include <lib/core/ScopedNodeId.h>

using namespace esp_matter;
using namespace esp_matter::attribute;

static const char * TAG = "light_manager";
volatile uint32_t g_last_press_tick = 0;

extern uint16_t g_onoff_endpoint_ids[LIGHT_CHANNELS];
extern const ShadowBindingList * shadow_binding_get_list(int ch);
extern "C" void temp_manager_start();

static bool s_led_any_on[LIGHT_CHANNELS] = {false};
static uint8_t s_pending_read_counts[LIGHT_CHANNELS] = {0};
static bool s_round_any_on[LIGHT_CHANNELS] = {false};
static const gpio_num_t s_button_gpios[LIGHT_CHANNELS] = { BUTTON_GPIO_0, BUTTON_GPIO_1, BUTTON_GPIO_2, BUTTON_GPIO_3 };
static const gpio_num_t s_led_gpios[LIGHT_CHANNELS]    = { LED_GPIO_0, LED_GPIO_1, LED_GPIO_2, LED_GPIO_3 };
static TaskHandle_t s_button_task = nullptr;
static TaskHandle_t s_button_act_task = nullptr;
static QueueHandle_t s_button_evt_queue = nullptr;
static esp_timer_handle_t s_led_blink_timers[LIGHT_CHANNELS] = {nullptr};

static void apply_led(uint8_t ch, bool on){ if (ch < LIGHT_CHANNELS) gpio_set_level(s_led_gpios[ch], on?1:0); }
bool light_manager_get(uint8_t ch){ return (ch<LIGHT_CHANNELS)? s_led_any_on[ch]: false; }

static void buttons_init(){ gpio_config_t in_cfg={}; in_cfg.intr_type=GPIO_INTR_DISABLE; in_cfg.mode=GPIO_MODE_INPUT; in_cfg.pull_down_en=GPIO_PULLDOWN_DISABLE; in_cfg.pull_up_en=GPIO_PULLUP_ENABLE; for(int i=0;i<LIGHT_CHANNELS;i++){ if (s_button_gpios[i]==GPIO_NUM_NC) continue; in_cfg.pin_bit_mask=(1ULL<<s_button_gpios[i]); gpio_config(&in_cfg); gpio_set_pull_mode(s_button_gpios[i], GPIO_PULLUP_ONLY);} }
static void leds_init(){ gpio_config_t out_cfg={}; out_cfg.intr_type=GPIO_INTR_DISABLE; out_cfg.mode=GPIO_MODE_OUTPUT; for(int i=0;i<LIGHT_CHANNELS;i++){ if (s_led_gpios[i]==GPIO_NUM_NC) continue; out_cfg.pin_bit_mask=(1ULL<<s_led_gpios[i]); gpio_config(&out_cfg); gpio_set_level(s_led_gpios[i],0);} }

static void button_task(void*){ uint8_t stable[LIGHT_CHANNELS]={0}; uint8_t last[LIGHT_CHANNELS]={1,1,1,1}; while(true){ for(int i=0;i<LIGHT_CHANNELS;i++){ int lvl=gpio_get_level(s_button_gpios[i]); if(lvl==last[i]){ if(stable[i]<255) stable[i]++; } else { stable[i]=0; last[i]=lvl; } if(last[i]==0 && stable[i]==BUTTON_STABLE_CNT){ uint8_t ch=i; if(s_button_evt_queue) xQueueSend(s_button_evt_queue,&ch,0); } } vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS)); } }
static void led_blink_timer_cb(void* arg){ uint32_t ch=(uint32_t)arg; if(ch<LIGHT_CHANNELS) apply_led(ch, s_led_any_on[ch]); }

class InitialReadCallback : public chip::app::ReadClient::Callback { public: explicit InitialReadCallback(uint8_t c):mCh(c){} void OnReportBegin() override {mGot=false;} void OnAttributeData(const chip::app::ConcreteDataAttributePath & path, chip::TLV::TLVReader * data,const chip::app::StatusIB & status) override { if(status.mStatus!=chip::Protocols::InteractionModel::Status::Success) return; if(path.mClusterId!=chip::app::Clusters::OnOff::Id || path.mAttributeId!=chip::app::Clusters::OnOff::Attributes::OnOff::Id) return; bool on=false; if(data && data->Get(on)==CHIP_NO_ERROR){ mGot=true; if(on){ s_round_any_on[mCh]=true; if(!s_led_any_on[mCh]){ s_led_any_on[mCh]=true; apply_led(mCh,true);} } } } void OnDone(chip::app::ReadClient * c) override { if (s_pending_read_counts[mCh]>0){ s_pending_read_counts[mCh]--; if(s_pending_read_counts[mCh]==0 && !s_round_any_on[mCh] && s_led_any_on[mCh]) { s_led_any_on[mCh]=false; apply_led(mCh,false);} } if(c) chip::Platform::Delete(c); chip::Platform::Delete(this);} void OnError(CHIP_ERROR err) override { ESP_LOGW(TAG,"CH%u read error %" CHIP_ERROR_FORMAT,mCh,err.Format()); } void OnReportEnd() override {} void OnSubscriptionEstablished(chip::SubscriptionId) override {} private: uint8_t mCh; bool mGot=false; };

struct PendingInitialRead { uint8_t ch; uint64_t node; chip::EndpointId ep; uint8_t fabric_index; };
static PendingInitialRead s_pending_reads[LIGHT_CHANNELS * MAX_SHADOW_BINDINGS_PER_CH];
static int s_pending_count=0;

static void send_initial_read(const PendingInitialRead & item){ chip::FabricIndex fi=item.fabric_index; if(fi==chip::kUndefinedFabricIndex){ for(auto &f: chip::Server::GetInstance().GetFabricTable()) if(f.IsInitialized()){ fi=f.GetFabricIndex(); break; } } if(fi==chip::kUndefinedFabricIndex) return; auto * caseMgr=chip::Server::GetInstance().GetCASESessionManager(); if(!caseMgr) return; struct Ctx{ PendingInitialRead it; }; auto * ctx=chip::Platform::New<Ctx>(); if(!ctx) return; ctx->it=item; auto onConn=[](void * c2, chip::Messaging::ExchangeManager & em, const chip::SessionHandle & sh){ std::unique_ptr<Ctx, void(*)(Ctx*)> guard((Ctx*)c2,[](Ctx* p){ chip::Platform::Delete(p); }); auto & it=guard->it; auto * cb=chip::Platform::New<InitialReadCallback>(it.ch); if(!cb) return; auto * client=chip::Platform::New<chip::app::ReadClient>(chip::app::InteractionModelEngine::GetInstance(), &em, *cb, chip::app::ReadClient::InteractionType::Read); if(!client){ chip::Platform::Delete(cb); return;} chip::app::AttributePathParams path; path.mEndpointId=it.ep; path.mClusterId=chip::app::Clusters::OnOff::Id; path.mAttributeId=chip::app::Clusters::OnOff::Attributes::OnOff::Id; chip::app::AttributePathParams paths[1]={path}; chip::app::ReadPrepareParams params(sh); params.mpAttributePathParamsList=paths; params.mAttributePathParamsListSize=1; if(client->SendRequest(params)!=CHIP_NO_ERROR){ chip::Platform::Delete(client); chip::Platform::Delete(cb);} }; auto onFail=[](void * c2, const chip::ScopedNodeId & peer, CHIP_ERROR e){ std::unique_ptr<Ctx, void(*)(Ctx*)> guard((Ctx*)c2,[](Ctx* p){ chip::Platform::Delete(p); }); ESP_LOGW(TAG,"Session fail node=0x%016" PRIX64 " err=%" CHIP_ERROR_FORMAT,(uint64_t)peer.GetNodeId(), e.Format()); }; auto * cb1=chip::Platform::New<chip::Callback::Callback<chip::OnDeviceConnected>>(onConn, ctx); auto * cb2=chip::Platform::New<chip::Callback::Callback<chip::OnDeviceConnectionFailure>>(onFail, ctx); if(!cb1||!cb2){ if(cb1) chip::Platform::Delete(cb1); if(cb2) chip::Platform::Delete(cb2); chip::Platform::Delete(ctx); return;} chip::ScopedNodeId scoped(item.node, fi); caseMgr->FindOrEstablishSession(scoped, cb1, cb2); }

static void schedule_single_initial_read(uint8_t ch, const ShadowBindingEntry & e){ if(e.is_group) return; if(s_pending_count >= (int)(sizeof(s_pending_reads)/sizeof(s_pending_reads[0]))) return; s_pending_reads[s_pending_count++]=PendingInitialRead{ch,e.node_id,(chip::EndpointId)e.endpoint,e.fabric_index}; s_pending_read_counts[ch]++; struct TimerCtx{ PendingInitialRead it; }; auto * tctx=chip::Platform::New<TimerCtx>(); if(!tctx) return; tctx->it=s_pending_reads[s_pending_count-1]; chip::DeviceLayer::SystemLayer().StartTimer(chip::System::Clock::Milliseconds32(0), [](chip::System::Layer*, void * arg){ auto * t=(TimerCtx*)arg; chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t a){ auto * t2=(TimerCtx*)a; send_initial_read(t2->it); chip::Platform::Delete(t2); }, (intptr_t)t); }, tctx); }

void light_manager_sync_initial_state(){ bool any=false; for(int ch=0;ch<LIGHT_CHANNELS;ch++) if(s_pending_read_counts[ch]) { any=true; break;} if(any) return; for(int ch=0;ch<LIGHT_CHANNELS;ch++){ s_round_any_on[ch]=false; s_pending_read_counts[ch]=0;} s_pending_count=0; for(int ch=0; ch<LIGHT_CHANNELS; ch++){ auto * list=shadow_binding_get_list(ch); if(!list||!list->count) continue; for(int i=0;i<list->count;i++){ if(!list->entries[i].is_group) schedule_single_initial_read(ch, list->entries[i]); } } }

static void send_group_toggle(uint8_t ch); // forward
void light_manager_button_press(uint8_t channel){ if(channel>=LIGHT_CHANNELS) return; g_last_press_tick=(uint32_t)xTaskGetTickCount(); const ShadowBindingList * list=shadow_binding_get_list(channel); int uni=0; if(list) for(int i=0;i<list->count;i++) if(!list->entries[i].is_group) uni++; ESP_LOGI(TAG,"Button press CH%u (unicast=%d)",channel,uni); if(s_led_gpios[channel]!=GPIO_NUM_NC){ apply_led(channel, !s_led_any_on[channel]); if(!s_led_blink_timers[channel]){ esp_timer_create_args_t a={ .callback=&led_blink_timer_cb, .arg=(void*)(uintptr_t)channel, .dispatch_method=ESP_TIMER_TASK, .name="ledblink" }; esp_timer_create(&a,&s_led_blink_timers[channel]); } if(s_led_blink_timers[channel]) esp_timer_start_once(s_led_blink_timers[channel], 40*1000); } send_group_toggle(channel); }

static void send_group_toggle(uint8_t ch){ if(ch>=LIGHT_CHANNELS) return; s_led_any_on[ch]=!s_led_any_on[ch]; apply_led(ch, s_led_any_on[ch]); chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t arg){ uint8_t ch_i=(uint8_t)arg; esp_matter::client::request_handle req={}; chip::app::CommandPathParams path(g_onoff_endpoint_ids[ch_i],0, chip::app::Clusters::OnOff::Id, chip::app::Clusters::OnOff::Commands::Toggle::Id, (chip::app::CommandPathFlags)0); req.command_path=path; esp_err_t err=esp_matter::client::cluster_update(g_onoff_endpoint_ids[ch_i], &req); if(err!=ESP_OK) ESP_LOGW(TAG,"cluster_update failed ch%u err=%d", ch_i, err); else ESP_LOGI(TAG,"CH%u: Toggle dispatched", ch_i); }, (intptr_t)ch); }

esp_err_t light_manager_init(){ buttons_init(); leds_init(); s_button_evt_queue=xQueueCreate(8,sizeof(uint8_t)); if(!s_button_evt_queue) return ESP_ERR_NO_MEM; xTaskCreate(button_task,"btn_poll",2048,nullptr,tskIDLE_PRIORITY+1,&s_button_task); auto act=[](void*){ uint8_t ch; while(true){ if(xQueueReceive(s_button_evt_queue,&ch,portMAX_DELAY)==pdTRUE) light_manager_button_press(ch);} }; xTaskCreate(act,"btn_act",3072,nullptr,tskIDLE_PRIORITY+2,&s_button_act_task); ESP_LOGI(TAG,"Light manager init complete"); return ESP_OK; }

void dht22_start_task(){ temp_manager_start(); }

