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

typedef struct {
    char ssid[32];
    char password[32];
    wifi_mode_t mode;
    bool connected;
} wifi_t;

/**
 * @brief Inicializa el stack WiFi y configura el modo (STA/AP)
 * 
 * @param wifi Estructura de configuración WiFi
 * @param mode Modo WiFi (WIFI_MODE_STA, WIFI_MODE_AP, etc)
 * @return true si la inicialización fue exitosa
 * @return false si falló la inicialización
 */
bool wifi_init(wifi_t *wifi, wifi_mode_t mode);

/**
 * @brief Conecta a una red WiFi (modo estación)
 * 
 * @param wifi Estructura de configuración WiFi (con SSID y password)
 * @return true si la conexión fue exitosa
 * @return false si falló la conexión
 */
bool wifi_connect(wifi_t *wifi);

/**
 * @brief Desconecta de la red WiFi
 * 
 * @param wifi Estructura de configuración WiFi
 * @return true si la desconexión fue exitosa
 * @return false si falló la desconexión
 */
bool wifi_disconnect(wifi_t *wifi);

#endif // __HAL_WIFI_ESP32S3__