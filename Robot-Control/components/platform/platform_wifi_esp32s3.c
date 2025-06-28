#include "platform_wifi_esp32s3.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include <string.h>
#include "functions.h"
#include <lwip/sockets.h>

#include <lwip/netdb.h>

#define UDP_PORT 12345
#define UDP_RX_BUF_SIZE 128
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define MAX_RETRY          20
#define WIFI_CMD_PORT 3333

static EventGroupHandle_t s_wifi_event_group;
static const char *TAG = "wifi_sta";
static const char *TAG_SCAN = "wifi_scan";
static const char *TAG_UDP_SERVER= "wifi_udp_server";
static int retry_num = 0;
static esp_ip4_addr_t s_ip;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
    ESP_LOGI(TAG, "Event received: base=%s, id=%ld", event_base, event_id);

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disconn = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "Disconnected from AP. Reason: %d", disconn->reason);

        // Mensaje personalizado al desconectarse
        ESP_LOGW(TAG, "Wi-Fi disconnected! Trying to reconnect...");

        if (retry_num < MAX_RETRY) {
            esp_wifi_connect();
            retry_num++;
            ESP_LOGI(TAG, "Retrying to connect to the AP (%d/%d)", retry_num, MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        s_ip = event->ip_info.ip;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&s_ip));
        retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_prepare(void) {
    static bool initialized = false;
    if (initialized) return;

    // NVS init con fallback si es necesario
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    initialized = true;
}


bool wifi_sta_init(const char *ssid, const char *password, esp_ip4_addr_t *out_ip) {
    if (!ssid || strlen(ssid) == 0) {
        ESP_LOGE(TAG, "SSID is null or empty");
        return false;
    }


    s_wifi_event_group = xEventGroupCreate();

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    
    wifi_config.sta.pmf_cfg.capable = false;
    wifi_config.sta.pmf_cfg.required = false;
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi init STA finished. Waiting for connection...");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    bool connected = false;
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to AP: %s", ssid);
        connected = true;
        if (out_ip) *out_ip = s_ip;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to AP: %s", ssid);
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);

    return connected;
}


void wifi_scan_and_print_networks(void) {
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true
    };

    ESP_LOGI(TAG_SCAN, "Starting Wi-Fi scan...");
    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));

    uint16_t ap_num = 0;
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_num));

    wifi_ap_record_t ap_records[20];
    if (ap_num > 20) ap_num = 20; // Limit to 20 records for simplicity
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_num, ap_records));

    ESP_LOGI(TAG_SCAN, "Found networks: %d", ap_num);
    for (int i = 0; i < ap_num; ++i) {
        const char *auth_mode = "UNKNOWN";
        switch (ap_records[i].authmode) {
            case WIFI_AUTH_OPEN: auth_mode = "OPEN"; break;
            case WIFI_AUTH_WEP: auth_mode = "WEP"; break;
            case WIFI_AUTH_WPA_PSK: auth_mode = "WPA"; break;
            case WIFI_AUTH_WPA2_PSK: auth_mode = "WPA2"; break;
            case WIFI_AUTH_WPA_WPA2_PSK: auth_mode = "WPA/WPA2"; break;
            case WIFI_AUTH_WPA3_PSK: auth_mode = "WPA3"; break;
            default: break;
        }

        ESP_LOGI(TAG_SCAN, "[%2d] SSID: %-20s | CH: %2d | RSSI: %3d | Auth: %s",
                 i + 1,
                 (char *)ap_records[i].ssid,
                 ap_records[i].primary,
                 ap_records[i].rssi,
                 auth_mode);
    }
}



void udp_server_task(void *pvParameters) {
    char rx_buffer[UDP_RX_BUF_SIZE];
    struct sockaddr_in6 dest_addr;
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;

    struct sockaddr_in listen_addr = {
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_family = AF_INET,
        .sin_port = htons(UDP_PORT)
    };

    int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
    if (sock < 0) {
        ESP_LOGE(TAG_UDP_SERVER, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG_UDP_SERVER, "Socket created");

    if (bind(sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0) {
        ESP_LOGE(TAG_UDP_SERVER, "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG_UDP_SERVER, "Socket bound, port %d", UDP_PORT);

    while (1) {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                           (struct sockaddr *)&source_addr, &socklen);

        if (len < 0) {
            ESP_LOGE(TAG_UDP_SERVER, "recvfrom failed: errno %d", errno);
            break;
        } else {
            rx_buffer[len] = 0; // Null-terminate
            ESP_LOGI(TAG_UDP_SERVER, "Received %d bytes from %s: '%s'",
                     len, inet_ntoa(source_addr.sin_addr), rx_buffer);

        }
    }

    close(sock);
    vTaskDelete(NULL);
}

