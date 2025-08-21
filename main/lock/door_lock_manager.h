  /*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once
#include <app/clusters/door-lock-server/door-lock-server.h>

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h" // provides FreeRTOS timer support
#include "freertos/task.h"
#include "freertos/queue.h"

#include <lib/core/CHIPError.h>
#include "driver/gpio.h"

// GPIO pins for garage door control
#define GARAGE_DOOR_RELAY_PIN  GPIO_NUM_6  // GPIO pin to control the garage door MOSFET (SW-M221)

// GPIO pins for reed switch door sensor
#define REED_SWITCH_PIN  GPIO_NUM_5  // Input pin for reed switch

// Door state definitions
#define DOOR_STATE_CLOSED  0
#define DOOR_STATE_OPEN    1

using namespace chip;
using namespace chip::app::Clusters::DoorLock;

class BoltLockManager
{
public:
    // Destructor to clean up resources
    ~BoltLockManager();
    enum Action_t
    {
        LOCK_ACTION = 0,
        UNLOCK_ACTION,

        INVALID_ACTION
    } Action;

    CHIP_ERROR InitLockState();
    CHIP_ERROR Init(chip::app::DataModel::Nullable<DlLockState> state);

    bool Lock(chip::EndpointId endpointId, const Optional<chip::ByteSpan> & pin, OperationErrorEnum & err);
    bool Unlock(chip::EndpointId endpointId, const Optional<chip::ByteSpan> & pin, OperationErrorEnum & err);

    bool setLockState(chip::EndpointId endpointId, DlLockState lockState, const Optional<chip::ByteSpan> & pin,
                       OperationErrorEnum & err);
    const char * lockStateToString(DlLockState lockState) const;
    
    // Door sensor methods
    void initDoorSensor();
    bool getDoorState();
    void updateDoorState(bool isOpen);
    static void doorSensorTask(void *pvParameters);
    
    // Delayed GPIO initialization task
    static void delayedGpioInitTask(void *pvParameters);
    
    // Contact sensor methods
    void updateContactSensorState(bool isOpen);
    
    // Method to schedule contact sensor state update on Matter thread
    static void ScheduleContactSensorUpdate(bool isOpen);
    
    // Work handler for Matter thread
    static void ContactSensorUpdateHandler(intptr_t context);

private:
    friend BoltLockManager & BoltLockMgr();
    
    // Initialize GPIO pins for garage door relay control
    void initRelayPin();
    
    // Toggle garage door relay
    void toggleGarageDoor();
    
    // Door sensor state
    bool mDoorIsOpen;
    TaskHandle_t mDoorSensorTaskHandle;
    
    // Thread-safe mechanism for contact sensor state
    static bool sContactSensorStateChanged;
    static bool sContactSensorState;
    static SemaphoreHandle_t sContactSensorMutex;

    static BoltLockManager sLock;
};

inline BoltLockManager & BoltLockMgr()
{
    return BoltLockManager::sLock;
}
