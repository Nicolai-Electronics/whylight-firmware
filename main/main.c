#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_eth.h"
#include "esp_eth_driver.h"
#include "esp_eth_mac_spi.h"
#include "esp_event.h"
#include "esp_ieee802154.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_ota_ops.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "ledstrip.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "sdmmc_cmd.h"
#include "soc/gpio_struct.h"
#include <inttypes.h>
#include <stdio.h>
#include <lwip/netdb.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static const char* TAG = "main";

// Configuration for ledstrip driver
static ledstrip_t ledstrip = {
    .pin = 20,
};

// Configuration for SPI bus
static spi_bus_config_t spi_bus_config = {
    .miso_io_num   = 0,
    .mosi_io_num   = 5,
    .sclk_io_num   = 1,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
};

// Configuration or W5500 ethernet
static spi_device_interface_config_t w5500_spi_device_config = {
    .mode           = 0,
    .clock_speed_hz = 10000000,  // 10MHz
    .queue_size     = 20,
    .spics_io_num   = 8,
};

static eth_w5500_config_t w5500_config = {
    .int_gpio_num      = 4,
    .poll_period_ms    = 0,
    .spi_host_id       = SPI2_HOST,
    .spi_devcfg        = &w5500_spi_device_config,
    .custom_spi_driver = ETH_DEFAULT_SPI,
};

static eth_mac_config_t w5500_mac_config = {
    .sw_reset_timeout_ms = 100,
    .rx_task_stack_size  = 4096,
    .rx_task_prio        = 15,
    .flags               = 0,
};

static eth_phy_config_t w5500_phy_config = {
    .phy_addr            = -1,
    .reset_timeout_ms    = 100,
    .autonego_timeout_ms = 4000,
    .reset_gpio_num      = 23,
};

static esp_eth_config_t w5500_eth_config = {
    .mac                     = NULL,  // Set during initialization
    .phy                     = NULL,  // Set during initialization
    .check_link_period_ms    = 2000,
    .stack_input             = NULL,
    .on_lowlevel_init_done   = NULL,
    .on_lowlevel_deinit_done = NULL,
    .read_phy_reg            = NULL,
    .write_phy_reg           = NULL,
};

// Configuration for Lora radio
static spi_device_interface_config_t lora_spi_device_config = {
    .mode           = 0,
    .clock_speed_hz = 4000000,  // 4MHz
    .queue_size     = 20,
    .spics_io_num   = 21,
};

// Variables
static esp_eth_handle_t w5500_eth_handle             = {0};
static uint8_t          w5500_mac_addr[ETH_ADDR_LEN] = {0};
static uint8_t          led_data[123 * 3]            = {0};

esp_err_t set_all_leds(uint32_t color) {
    for (uint32_t ch = 0; ch < sizeof(led_data) / 3; ch++) {
        led_data[ch * 3 + 0] = (color & 0xFF00) >> 8;     // Green
        led_data[ch * 3 + 1] = (color & 0xFF0000) >> 16;  // Red
        led_data[ch * 3 + 2] = (color & 0xFF);            // Blue
    }
    return ledstrip_send(&ledstrip, led_data, sizeof(led_data));
}

static void eth_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    uint8_t          mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle  = *(esp_eth_handle_t*)event_data;

    switch (event_id) {
        case ETHERNET_EVENT_CONNECTED:
            esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
            ESP_LOGI(TAG, "Ethernet Link Up");
            ESP_LOGI(
                TAG,
                "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                mac_addr[0],
                mac_addr[1],
                mac_addr[2],
                mac_addr[3],
                mac_addr[4],
                mac_addr[5]
            );
            ESP_ERROR_CHECK(set_all_leds(0x000011));  // Set all LEDs to blue
            break;
        case ETHERNET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "Ethernet Link Down");
            ESP_ERROR_CHECK(set_all_leds(0x110000));  // Set all LEDs to red
            break;
        case ETHERNET_EVENT_START: ESP_LOGI(TAG, "Ethernet Started"); break;
        case ETHERNET_EVENT_STOP: ESP_LOGI(TAG, "Ethernet Stopped"); break;
        default: break;
    }
}

#define ARTNET_PORT 6454

const esp_netif_ip_info_t* ip_info;

static void artnet_task(void* pvParameters) {
    char                rx_buffer[1024];
    char                addr_str[128];
    int                 addr_family = (int)pvParameters;
    int                 ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    while (1) {

        if (addr_family == AF_INET) {
            struct sockaddr_in* dest_addr_ip4 = (struct sockaddr_in*)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr    = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family         = AF_INET;
            dest_addr_ip4->sin_port           = htons(ARTNET_PORT);
            ip_protocol                       = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port   = htons(ARTNET_PORT);
            ip_protocol           = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        /*if (addr_family == AF_INET6) {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }*/

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec  = 10;
        timeout.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        int err = bind(sock, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", ARTNET_PORT);

        struct sockaddr_storage source_addr;  // Large enough for both IPv4 or IPv6
        socklen_t               socklen = sizeof(source_addr);

        while (1) {
            // ESP_LOGI(TAG, "Waiting for data");
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr*)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                if (errno == 11) {           // Timeout
                    set_all_leds(0x111111);  // White
                    ESP_LOGI(
                        TAG,
                        "IP Address: " IPSTR ", mask: " IPSTR ", gateway: " IPSTR,
                        IP2STR(&ip_info->ip),
                        IP2STR(&ip_info->netmask),
                        IP2STR(&ip_info->gw)
                    );
                    continue;
                }
                break;
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.ss_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in*)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
                } else if (source_addr.ss_family == PF_INET6) {
                    inet6_ntoa_r(((struct sockaddr_in6*)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
                }

                rx_buffer[len] = 0;  // Null-terminate whatever we received and treat like a string...
                // ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                // ESP_LOGI(TAG, "%s", rx_buffer);

                if (len < 18) {
                    ESP_LOGW(TAG, "Ignored too-short Art-Net packet");
                    continue;
                }

                if (memcmp("Art-Net\0", rx_buffer, 7)) {
                    ESP_LOGW(TAG, "Ignored Art-Net packet with invalid magic");
                    continue;
                }

                uint16_t opcode = (rx_buffer[9] << 8) | rx_buffer[8];

                if (opcode != 0x5000) {
                    ESP_LOGW(TAG, "Ignored non-DMX Art-Net packet");
                    continue;
                }

                uint16_t protocol_version = (rx_buffer[10] << 8) | rx_buffer[11];

                if (protocol_version != 14) {
                    ESP_LOGW(TAG, "Ignored Art-Net packet with invalid version");
                    continue;
                }

                uint8_t  sequence = rx_buffer[12];
                uint8_t  physical = rx_buffer[13];
                uint16_t universe = (rx_buffer[15] << 8) | rx_buffer[14];
                uint16_t length   = (rx_buffer[16] << 8) | rx_buffer[17];

                // printf("Art-Net sequence %u, physical %u, universe %u, length %u\r\n", sequence, physical, universe,
                // length);

                if (length > sizeof(led_data)) {
                    // ESP_LOGW(TAG, "Limited Art-Net data length to %u from %u", sizeof(led_data), length);
                    length = sizeof(led_data);
                }

                memcpy(led_data, &rx_buffer[18], length);
                for (int i = 0; i < sizeof(led_data); i++) {
                    // led_data[i] = led_data[i] / 2;
                }
                for (int i = 0; i < sizeof(led_data) / 3; i++) {
                    uint8_t r           = led_data[i * 3 + 0];
                    uint8_t g           = led_data[i * 3 + 1];
                    led_data[i * 3 + 0] = g;
                    led_data[i * 3 + 1] = r;
                }

                ledstrip_send(&ledstrip, led_data, sizeof(led_data));

                /*int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }*/
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

/** Event handler for IP_EVENT_ETH_GOT_IP */
static void got_ip_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
    ip_info                  = &event->ip_info;

    ESP_LOGI(
        TAG,
        "Ethernet Got IP Address: " IPSTR ", mask: " IPSTR ", gateway: " IPSTR,
        IP2STR(&ip_info->ip),
        IP2STR(&ip_info->netmask),
        IP2STR(&ip_info->gw)
    );

    ESP_ERROR_CHECK(set_all_leds(0x001100));  // Set all LEDs to green
}

void app_main(void) {
    // Welcome message
    const esp_app_desc_t* app_description = esp_app_get_description();
    printf("%s firmware v%s\r\n", app_description->project_name, app_description->version);

    // Initialize GPIO ISR handler
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    // Initialize LED strip
    ESP_ERROR_CHECK(ledstrip_init(&ledstrip));
    ESP_ERROR_CHECK(set_all_leds(0x110000));  // Set all LEDs to red

    // Initialize SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_bus_config, SPI_DMA_CH_AUTO));

    // Initialize W5500 ethernet
    esp_eth_mac_t* w5500_mac = esp_eth_mac_new_w5500(&w5500_config, &w5500_mac_config);
    esp_eth_phy_t* w5500_phy = esp_eth_phy_new_w5500(&w5500_phy_config);

    w5500_eth_config.mac = w5500_mac;
    w5500_eth_config.phy = w5500_phy;
    ESP_ERROR_CHECK(esp_eth_driver_install(&w5500_eth_config, &w5500_eth_handle));

    uint8_t base_mac_addr[ETH_ADDR_LEN];
    ESP_ERROR_CHECK(esp_efuse_mac_get_default(base_mac_addr));
    esp_derive_local_mac(w5500_mac_addr, base_mac_addr);  // Note: this generates a Locally Administered OUI range MAC

    ESP_ERROR_CHECK(esp_eth_ioctl(w5500_eth_handle, ETH_CMD_S_MAC_ADDR, w5500_mac_addr));  // Set MAC address

    ESP_LOGI(
        TAG,
        "W5500 MAC address: %02x:%02x:%02x:%02x:%02x:%02x",
        w5500_mac_addr[0],
        w5500_mac_addr[1],
        w5500_mac_addr[2],
        w5500_mac_addr[3],
        w5500_mac_addr[4],
        w5500_mac_addr[5]
    );

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_config_t          cfg            = ESP_NETIF_DEFAULT_ETH();
    esp_netif_t*                eth_netif      = esp_netif_new(&cfg);
    esp_eth_netif_glue_handle_t eth_netif_glue = esp_eth_new_netif_glue(w5500_eth_handle);
    ESP_ERROR_CHECK(esp_netif_attach(eth_netif, eth_netif_glue));

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

    ESP_ERROR_CHECK(esp_eth_start(w5500_eth_handle));

    // Start Art-Net listener
    xTaskCreate(artnet_task, "Art-Net", 4096, (void*)AF_INET, 5, NULL);


    /*while (1) {
        for (uint32_t ch = 0; ch < sizeof(data) / 3; ch++) {
            data[ch*3] = 10;
        }
        res = ledstrip_send(&ledstrip, data, sizeof(data));
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "LED error");
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }*/

    /*while (1) {
        for (uint32_t ch = 0; ch < sizeof(led_data); ch++) {
            memset(led_data, 0, sizeof(led_data));
            led_data[ch] = 10;
            printf("CH: %" PRIu32 "\r\n", ch);
            res = ledstrip_send(&ledstrip, led_data, sizeof(led_data));
            if (res != ESP_OK) {
                ESP_LOGE(TAG, "LED error");
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }*/
}
