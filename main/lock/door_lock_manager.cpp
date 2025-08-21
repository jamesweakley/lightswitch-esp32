/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "door_lock_manager.h"

#include <app-common/zap-generated/attributes/Accessors.h>
#include <app-common/zap-generated/ids/Clusters.h>
#include <app-common/zap-generated/ids/Attributes.h>
#include <cstring>
#include <esp_log.h>
#include <esp_matter.h>
#include <platform/CHIPDeviceLayer.h>

static const char *TAG = "garagedoor_manager";

// External reference to the endpoint IDs defined in app_main.cpp
extern uint16_t door_lock_endpoint_id;
extern uint16_t contact_sensor_endpoint_id;

BoltLockManager BoltLockManager::sLock;

// Initialize static variables
bool BoltLockManager::sContactSensorStateChanged = false;
bool BoltLockManager::sContactSensorState = false;
SemaphoreHandle_t BoltLockManager::sContactSensorMutex = NULL;

using namespace chip;
using namespace chip::app;
using namespace chip::app::Clusters;
using namespace chip::app::Clusters::DoorLock;

BoltLockManager::~BoltLockManager()
{
    // Clean up the mutex if it exists
    if (sContactSensorMutex != NULL) {
        vSemaphoreDelete(sContactSensorMutex);
        sContactSensorMutex = NULL;
        ESP_LOGI(TAG, "Cleaned up contact sensor mutex");
    }
    
    // Stop the garage door sensor task if it's running
    if (mDoorSensorTaskHandle != NULL) {
        vTaskDelete(mDoorSensorTaskHandle);
        mDoorSensorTaskHandle = NULL;
        ESP_LOGI(TAG, "Stopped garage door sensor task");
    }
}

CHIP_ERROR BoltLockManager::Init(DataModel::Nullable<DlLockState> state)
{
    ESP_LOGI(TAG, "Initializing garage door controller");
    
    // Create mutex for thread-safe access to contact sensor state
    if (sContactSensorMutex == NULL) {
        sContactSensorMutex = xSemaphoreCreateMutex();
        if (sContactSensorMutex == NULL) {
            ESP_LOGE(TAG, "Failed to create contact sensor mutex");
            return CHIP_ERROR_NO_MEMORY;
        }
        ESP_LOGI(TAG, "Created contact sensor mutex for thread-safe access");
    }
    
    // Defer GPIO initialization to prevent USB disconnection during Matter startup
    // GPIO will be initialized later when the system is stable
    ESP_LOGI(TAG, "GPIO initialization deferred until system is stable");
    
    // Create a delayed initialization task that will run after Matter is stable
    BaseType_t taskResult = xTaskCreate(delayedGpioInitTask, "delayed_gpio_init", 4096, this, 5, NULL);
    
    if (taskResult != pdPASS) {
        ESP_LOGE(TAG, "Failed to create delayed GPIO initialization task");
        return CHIP_ERROR_NO_MEMORY;
    }
    ESP_LOGI(TAG, "Delayed GPIO initialization task created successfully");
    
    return CHIP_NO_ERROR;
}

void BoltLockManager::delayedGpioInitTask(void *pvParameters)
{
    BoltLockManager* manager = static_cast<BoltLockManager*>(pvParameters);
    
    // Wait for Matter stack to stabilize
    vTaskDelay(pdMS_TO_TICKS(5000)); // 5 second delay
    
    ESP_LOGI(TAG, "Starting delayed GPIO initialization...");
    
    // Now initialize GPIO safely
    manager->initRelayPin();
    ESP_LOGI(TAG, "Relay pin initialization completed successfully");
    
    manager->initDoorSensor();
    ESP_LOGI(TAG, "Door sensor initialization completed successfully");
    
    // Start the door sensor monitoring task
    xTaskCreate(doorSensorTask, "garage_door_sensor_task", 2048, manager, 5, &manager->mDoorSensorTaskHandle);
    
    // Delete this initialization task
    vTaskDelete(NULL);
}

void BoltLockManager::initRelayPin()
{
    ESP_LOGI(TAG, "Initializing garage door MOSFET control pin GP%d for SW-M221", GARAGE_DOOR_RELAY_PIN);
    
    // Reset the pin to ensure clean state
    gpio_reset_pin(GARAGE_DOOR_RELAY_PIN);
    
    // For SW-M221 MOSFET, we can safely use push-pull output
    // MOSFETs are voltage-controlled with minimal current draw
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << GARAGE_DOOR_RELAY_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    
    // Set pin LOW before configuring to ensure MOSFET starts OFF
    gpio_set_level(GARAGE_DOOR_RELAY_PIN, 0);
    
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Use minimal drive capability - MOSFETs need very little current
    ESP_ERROR_CHECK(gpio_set_drive_capability(GARAGE_DOOR_RELAY_PIN, GPIO_DRIVE_CAP_0));
    
    // Verify configuration
    gpio_drive_cap_t drive_cap;
    ESP_ERROR_CHECK(gpio_get_drive_capability(GARAGE_DOOR_RELAY_PIN, &drive_cap));
    
    ESP_LOGI(TAG, "MOSFET control pin initialized: PIN=%d (push-pull, drive=%d, state=LOW/OFF)",
             GARAGE_DOOR_RELAY_PIN, drive_cap);
}

void BoltLockManager::initDoorSensor()
{
    ESP_LOGI(TAG, "Initializing reed switch garage door sensor with single pin GP%d (INVERTED LOGIC)",
             REED_SWITCH_PIN);
    
    // Reset the pin to default state
    gpio_reset_pin(REED_SWITCH_PIN);
    
    // Configure the pin as input with pull-up
    gpio_config_t input_conf = {};
    input_conf.intr_type = GPIO_INTR_DISABLE;
    input_conf.mode = GPIO_MODE_INPUT;
    input_conf.pin_bit_mask = (1ULL << REED_SWITCH_PIN);
    input_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    input_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&input_conf);
    
    // Explicitly set the internal pull-up resistor for the input pin
    gpio_set_pull_mode(REED_SWITCH_PIN, GPIO_PULLUP_ONLY);
    
    // Remove delays during initialization to prevent USB disconnection
    // GPIO stabilization will happen naturally
    
    // Read the initial state (single read is sufficient during init)
    int state1 = gpio_get_level(REED_SWITCH_PIN);
    int state2 = gpio_get_level(REED_SWITCH_PIN);
    int state3 = gpio_get_level(REED_SWITCH_PIN);
    
    // Initialize door state
    mDoorIsOpen = getDoorState();
    
    ESP_LOGI(TAG, "Garage door sensor initialized: PIN=%d (sense), Raw GPIO readings: %d, %d, %d, Initial state: %s",
             REED_SWITCH_PIN, state1, state2, state3, mDoorIsOpen ? "OPEN" : "CLOSED");
}

bool BoltLockManager::getDoorState()
{
    // CORRECTED LOGIC with pull-up resistor on a single pin:
    // When the pin is shorted to ground, the pin reads LOW (0) = DOOR CLOSED
    // When the pin is not shorted, the pin reads HIGH (1) = DOOR OPEN
    
    // Read the pin multiple times to debounce (without delays to prevent USB issues)
    int reading1 = gpio_get_level(REED_SWITCH_PIN);
    int reading2 = gpio_get_level(REED_SWITCH_PIN);
    int reading3 = gpio_get_level(REED_SWITCH_PIN);
    
    // Use the majority reading
    int state;
    if ((reading1 + reading2 + reading3) >= 2) {
        state = 1; // Majority HIGH
    } else {
        state = 0; // Majority LOW
    }
    
    // Enhanced debugging information
    ESP_LOGI(TAG, "Reed switch readings: PIN=GP%d, Raw GPIO=%d,%d,%d (final=%d)",
             REED_SWITCH_PIN, reading1, reading2, reading3, state);
    
    // CORRECTED LOGIC to match actual hardware behavior:
    // When pin is shorted to ground, GPIO reads LOW (0), meaning door is CLOSED
    // When pin is not shorted, GPIO reads HIGH (1), meaning door is OPEN
    bool doorState = (state == 1) ? DOOR_STATE_OPEN : DOOR_STATE_CLOSED;
    
    // Log the door state with clear instructions for testing
    ESP_LOGI(TAG, "Garage door state: %s (GPIO=%d) - %s",
            doorState ? "OPEN" : "CLOSED",
            state,
            doorState ? "Reed switch NOT shorted" : "Reed switch shorted to ground");
    
    return doorState;
}

void BoltLockManager::updateContactSensorState(bool isOpen)
{
    // Check if the state has actually changed
    static bool lastState = !isOpen; // Initialize to opposite to force first update
    bool stateChanged = (lastState != isOpen);
    
    if (stateChanged) {
        // Log the state change with clear information
        ESP_LOGI(TAG, "Contact sensor state CHANGED: %s -> %s",
                lastState ? "OPEN (active)" : "CLOSED (inactive)",
                isOpen ? "OPEN (active)" : "CLOSED (inactive)");
        
        // Update the last state
        lastState = isOpen;
    } else {
        // Log that the state hasn't changed
        ESP_LOGI(TAG, "Contact sensor state UNCHANGED: Still %s",
                isOpen ? "OPEN (active)" : "CLOSED (inactive)");
    }
    
    // Schedule the update on the Matter thread
    ScheduleContactSensorUpdate(isOpen);
}

void BoltLockManager::ScheduleContactSensorUpdate(bool isOpen)
{
    // Pass the state directly as the context parameter
    // Convert bool to intptr_t: 0 = false (closed), 1 = true (open)
    intptr_t stateValue = isOpen ? 1 : 0;
    
    // Schedule the work on the Matter thread
    ESP_LOGI(TAG, "Scheduling contact sensor update on Matter thread: %s", isOpen ? "OPEN" : "CLOSED");
    chip::DeviceLayer::PlatformMgr().ScheduleWork(ContactSensorUpdateHandler, stateValue);
}

void BoltLockManager::ContactSensorUpdateHandler(intptr_t context)
{
    // This method runs in the Matter thread context
    // Convert intptr_t back to bool: 0 = false (closed), 1 = true (open)
    bool isOpen = (context != 0);
    
    // Get the endpoint ID from the global variable
    extern uint16_t contact_sensor_endpoint_id;
    
    if (contact_sensor_endpoint_id > 0) {
        // Use the esp_matter API to update the contact sensor state
        // BooleanState cluster ID: 0x0045, StateValue attribute ID: 0x0000
        esp_matter_attr_val_t val = esp_matter_bool(isOpen);
        
        // This is safe because we're in the Matter context
        esp_err_t err = esp_matter::attribute::report(contact_sensor_endpoint_id, 0x0045, 0x0000, &val);
        
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Matter thread: Updated contact sensor state to %s",
                     isOpen ? "OPEN (active)" : "CLOSED (inactive)");
        } else {
            ESP_LOGE(TAG, "Failed to update contact sensor state: %d", err);
        }
    } else {
        ESP_LOGE(TAG, "Contact sensor endpoint ID not set");
    }
}

void BoltLockManager::updateDoorState(bool isOpen)
{
    // Update the internal state
    mDoorIsOpen = isOpen;
    
    // Log the door state change
    ESP_LOGI(TAG, "Garage door state changed: %s", isOpen ? "OPEN" : "CLOSED");
    
    // Schedule the door lock state update on the Matter thread
    struct DoorStateContext {
        bool isOpen;
        EndpointId lockEndpointId;
    };
    
    static DoorStateContext doorContext;
    doorContext.isOpen = isOpen;
    doorContext.lockEndpointId = 1;
    
    // Schedule the door lock state update on the Matter thread
    ESP_LOGI(TAG, "Scheduling door lock state update on Matter thread");
    chip::DeviceLayer::PlatformMgr().ScheduleWork(
        [](intptr_t context) {
            DoorStateContext* ctx = reinterpret_cast<DoorStateContext*>(context);
            
            // Set lock state to match actual door position
            DlLockState newLockState = ctx->isOpen ? DlLockState::kUnlocked : DlLockState::kLocked;
            DoorLockServer::Instance().SetLockState(ctx->lockEndpointId, newLockState);
            
            ESP_LOGI(TAG, "Updated lock state to %s to match door position (%s)",
                     ctx->isOpen ? "UNLOCKED" : "LOCKED",
                     ctx->isOpen ? "OPEN" : "CLOSED");
        },
        reinterpret_cast<intptr_t>(&doorContext)
    );
    
    // Update the contact sensor state (inverted logic for contact sensor)
    // Door open = contact sensor inactive (false), Door closed = contact sensor active (true)
    updateContactSensorState(!isOpen);
}

void BoltLockManager::doorSensorTask(void *pvParameters)
{
    BoltLockManager *manager = static_cast<BoltLockManager *>(pvParameters);
    bool lastDoorState = manager->getDoorState();
    
    // Log initial state
    ESP_LOGI(TAG, "Garage door sensor task started. Initial state: %s",
             lastDoorState ? "OPEN" : "CLOSED");
    
    // Force an initial update to ensure the contact sensor state is set correctly
    manager->updateDoorState(lastDoorState);
    
    // Counter for periodic status logging
    int logCounter = 0;
    
    while (1) {
        // Read current door state
        bool currentDoorState = manager->getDoorState();
        
        // If door state has changed, update it immediately
        if (currentDoorState != lastDoorState) {
            ESP_LOGI(TAG, "Garage door state changed from %s to %s",
                     lastDoorState ? "OPEN" : "CLOSED",
                     currentDoorState ? "OPEN" : "CLOSED");
            
            // Update the door state in the system
            manager->updateDoorState(currentDoorState);
            lastDoorState = currentDoorState;
        }
        
        // Periodically log the current state (every ~5 seconds)
        if (++logCounter >= 50) {
            ESP_LOGI(TAG, "Garage door sensor periodic status: %s",
                     currentDoorState ? "OPEN" : "CLOSED");
            logCounter = 0;
        }
        
        // Check every 100ms for more responsive detection
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void BoltLockManager::toggleGarageDoor()
{
    ESP_LOGI(TAG, "Garage door: Scheduling MOSFET toggle operation");
    
    // Schedule the MOSFET toggle on a separate task to avoid blocking the Matter thread
    xTaskCreate([](void* param) {
        const int mosfet_activation_time = 1000; // 1 second activation
        
        ESP_LOGI(TAG, "Garage door: Activating MOSFET");
        
        // Activate the MOSFET (set pin HIGH)
        // SW-M221 is a low-side N-channel MOSFET: HIGH = ON, LOW = OFF
        gpio_set_level(GARAGE_DOOR_RELAY_PIN, 1);
        ESP_LOGI(TAG, "Garage door MOSFET ACTIVATED (GPIO=%d HIGH)", GARAGE_DOOR_RELAY_PIN);
        
        // Keep the MOSFET activated for 1 second
        vTaskDelay(pdMS_TO_TICKS(mosfet_activation_time));
        
        // Deactivate the MOSFET (set pin LOW)
        gpio_set_level(GARAGE_DOOR_RELAY_PIN, 0);
        ESP_LOGI(TAG, "Garage door MOSFET DEACTIVATED (GPIO=%d LOW)", GARAGE_DOOR_RELAY_PIN);
        
        ESP_LOGI(TAG, "Garage door: MOSFET toggle operation completed - door should be moving");
        
        // Delete this task as it's a one-time operation
        vTaskDelete(NULL);
    }, "garage_door_toggle", 2048, NULL, 5, NULL);
}

bool BoltLockManager::Lock(EndpointId endpointId, const Optional<ByteSpan> & pin, OperationErrorEnum & err)
{
    ESP_LOGI(TAG, "Garage Door App: Lock command received [endpointId=%d]", endpointId);
    // These methods are called from the Matter context, so they're safe to use
    return setLockState(endpointId, DlLockState::kLocked, pin, err);
}

bool BoltLockManager::Unlock(EndpointId endpointId, const Optional<ByteSpan> & pin, OperationErrorEnum & err)
{
    ESP_LOGI(TAG, "Garage Door App: Unlock command received [endpointId=%d]", endpointId);
    // These methods are called from the Matter context, so they're safe to use
    return setLockState(endpointId, DlLockState::kUnlocked, pin, err);
}

const char * BoltLockManager::lockStateToString(DlLockState lockState) const
{
    switch (lockState)
    {
    case DlLockState::kNotFullyLocked:
        return "Not Fully Locked";
    case DlLockState::kLocked:
        return "Locked";
    case DlLockState::kUnlocked:
        return "Unlocked";
    case DlLockState::kUnlatched:
        return "Unlatched";
    case DlLockState::kUnknownEnumValue:
        break;
    }

    return "Unknown";
}

bool BoltLockManager::setLockState(EndpointId endpointId, DlLockState lockState, const Optional<ByteSpan> & pin,
                                   OperationErrorEnum & err)
{
    ESP_LOGI(TAG, "Garage Door App: Setting door lock state to \"%s\" [endpointId=%d]", lockStateToString(lockState), endpointId);
    
    // Get current door physical state
    bool doorIsCurrentlyOpen = mDoorIsOpen;
    
    // Determine if we need to toggle based on desired state vs current physical state
    bool shouldToggle = false;
    
    if (lockState == DlLockState::kLocked && doorIsCurrentlyOpen) {
        // Want to lock (close) but door is currently open
        shouldToggle = true;
        ESP_LOGI(TAG, "Garage Door: Need to close door (currently open, want locked)");
    } else if (lockState == DlLockState::kUnlocked && !doorIsCurrentlyOpen) {
        // Want to unlock (open) but door is currently closed
        shouldToggle = true;
        ESP_LOGI(TAG, "Garage Door: Need to open door (currently closed, want unlocked)");
    } else {
        ESP_LOGI(TAG, "Garage Door: Door is already in desired state, no toggle needed");
        // Set the lock state to match current physical state
        DlLockState currentPhysicalState = doorIsCurrentlyOpen ? DlLockState::kUnlocked : DlLockState::kLocked;
        DoorLockServer::Instance().SetLockState(endpointId, currentPhysicalState);
        return true;
    }
    
    // If we need to toggle, set intermediate state and schedule completion check
    if (shouldToggle) {
        // Set intermediate state to show operation in progress
        if (lockState == DlLockState::kLocked) {
            ESP_LOGI(TAG, "Setting intermediate state: LOCKING");
            // For garage doors, we don't have a "Locking" state, so we'll check after delay
        } else {
            ESP_LOGI(TAG, "Setting intermediate state: UNLOCKING");
            // For garage doors, we don't have an "Unlocking" state, so we'll check after delay
        }
        
        ESP_LOGI(TAG, "Garage Door: Triggering toggle operation");
        toggleGarageDoor();
        
        // Schedule a delayed check to verify the door has moved (15 seconds for garage door)
        struct DelayedStateCheck {
            EndpointId endpointId;
            DlLockState targetState;
            BoltLockManager* manager;
        };
        
        static DelayedStateCheck checkContext;
        checkContext.endpointId = endpointId;
        checkContext.targetState = lockState;
        checkContext.manager = this;
        
        // Create a task to check the door state after 15 seconds
        xTaskCreate([](void* param) {
            DelayedStateCheck* ctx = static_cast<DelayedStateCheck*>(param);
            
            // Wait 15 seconds for garage door to complete movement
            vTaskDelay(pdMS_TO_TICKS(15000));
            
            ESP_LOGI(TAG, "Checking door state after 15-second delay...");
            
            // Read current door state and update lock state accordingly
            bool currentDoorState = ctx->manager->getDoorState();
            DlLockState actualLockState = currentDoorState ? DlLockState::kUnlocked : DlLockState::kLocked;
            
            // Update the lock state to match actual door position
            chip::DeviceLayer::PlatformMgr().ScheduleWork([](intptr_t context) {
                DelayedStateCheck* ctx = reinterpret_cast<DelayedStateCheck*>(context);
                bool doorState = ctx->manager->getDoorState();
                DlLockState finalState = doorState ? DlLockState::kUnlocked : DlLockState::kLocked;
                
                DoorLockServer::Instance().SetLockState(ctx->endpointId, finalState);
                ESP_LOGI(TAG, "Final lock state set to %s after garage door operation",
                         doorState ? "UNLOCKED" : "LOCKED");
            }, reinterpret_cast<intptr_t>(ctx));
            
            // Delete this task
            vTaskDelete(NULL);
        }, "delayed_state_check", 2048, &checkContext, 5, NULL);
    }
    
    return true;
}

CHIP_ERROR BoltLockManager::InitLockState()
{
    ESP_LOGI(TAG, "Starting lock state initialization");
    
    // Initial lock state
    DataModel::Nullable<DlLockState> state;
    EndpointId lockEndpointId{ 1 };
    
    // Try to get the current lock state, but don't fail if it's not available yet
    auto stateResult = DoorLock::Attributes::LockState::Get(lockEndpointId, state);
    if (stateResult != chip::Protocols::InteractionModel::Status::Success) {
        ESP_LOGW(TAG, "Could not get initial lock state, using default");
        state.SetNull(); // Use null state as default
    }

    // Initialize the simplified door lock manager
    CHIP_ERROR err = BoltLockMgr().Init(state);
    if (err != CHIP_NO_ERROR)
    {
        ESP_LOGE(TAG, "BoltLockMgr().Init() failed: %" CHIP_ERROR_FORMAT, err.Format());
        return err;
    }
    ESP_LOGI(TAG, "BoltLockMgr initialized successfully");

    // Add a small delay to ensure GPIO is stable
    vTaskDelay(pdMS_TO_TICKS(200));

    // Check initial door state
    bool doorIsOpen = BoltLockMgr().getDoorState();
    ESP_LOGI(TAG, "Initial door state read: %s", doorIsOpen ? "OPEN" : "CLOSED");

    // Set initial state to locked (without triggering garage door)
    // We use DoorLockServer directly to avoid triggering the relay during initialization
    DoorLockServer::Instance().SetLockState(lockEndpointId, DlLockState::kLocked);
    ESP_LOGI(TAG, "Initial lock state set to LOCKED");
    
    // Update contact sensor state with initial door state
    BoltLockMgr().updateContactSensorState(doorIsOpen);
    ESP_LOGI(TAG, "Contact sensor state updated");
    
    if (doorIsOpen) {
        // If garage door is open but lock state is locked, update to not fully locked
        DoorLockServer::Instance().SetLockState(lockEndpointId, DlLockState::kNotFullyLocked);
        ESP_LOGI(TAG, "Initial garage door state is OPEN, setting lock state to NOT_FULLY_LOCKED");
    }
    
    ESP_LOGI(TAG, "Garage door controller and contact sensor initialized successfully");
    return CHIP_NO_ERROR;
}
