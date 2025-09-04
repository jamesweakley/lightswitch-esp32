/* Temperature & Humidity manager (DHT22) */
#pragma once
#include <stdint.h>
#include <esp_err.h>
#ifdef __cplusplus
extern "C" {
#endif

void temp_manager_start();
void temp_manager_stop();
void temp_manager_force_read();

#ifdef __cplusplus
}
#endif
