/**
 * \file        platform_wifi.h
 * \brief       WiFi HAL for ESP32-S3
 * \details     WiFi initialization and connection management
 * 
 * \author      Kevin Jimenez
 * \version     0.0.1
 * \date        14/06/2025
 */

#ifndef __HAL_WIFI_ESP32S3__
#define __HAL_WIFI_ESP32S3__

#include <stdbool.h>
#include <stdint.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"


/**
 * @brief Initializes the WiFi station mode and connects to the specified access point (AP)
 * @param ssid SSID of the AP
 * @param password Password of the AP
 * @return True if the connection was successful, false otherwise
 */
bool wifi_sta_init(const char *ssid, const char *password);


#endif // __HAL_WIFI_ESP32S3__