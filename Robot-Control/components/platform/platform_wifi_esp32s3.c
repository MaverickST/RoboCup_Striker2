#include "platform_wifi_esp32s3.h"

static const char* TAG_WIFI = "wifi_hal";

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    wifi_t *wifi = (wifi_t*) arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi->connected = false;
        ESP_LOGI(TAG_WIFI, "Desconectado de WiFi");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        wifi->connected = true;
        ESP_LOGI(TAG_WIFI, "Conectado a WiFi");
    }
}

bool wifi_init(wifi_t *wifi, wifi_mode_t mode)
{
    esp_err_t ret = ESP_OK;
    wifi->mode = mode;
    wifi->connected = false;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, wifi, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, wifi, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_mode(mode));
    ESP_ERROR_CHECK(esp_wifi_start());

    return true;
}

bool wifi_connect(wifi_t *wifi)
{
    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid, wifi->ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, wifi->password, sizeof(wifi_config.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_connect());
    return true;
}

bool wifi_disconnect(wifi_t *wifi)
{
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    wifi->connected = false;
    return true;
}