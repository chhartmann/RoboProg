#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <esp_eth.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_err.h>
#include <qemu_eth.h>

#define ETH_CONNECTED_BIT BIT0
#define ETH_FAIL_BIT      BIT1

static const char *TAG = "qemu_eth";
static esp_eth_handle_t s_eth_handle = NULL;
static esp_eth_mac_t *s_mac = NULL;
static esp_eth_phy_t *s_phy = NULL;
static void* s_eth_glue = NULL;
static EventGroupHandle_t s_eth_event_group;

static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));

    xEventGroupSetBits(s_eth_event_group, ETH_CONNECTED_BIT);
}

static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    uint8_t mac_addr[6] = {0};
    /* we can get the ethernet driver handle from event data */
    esp_eth_handle_t eth_handle = *(esp_eth_handle_t *)event_data;

    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        esp_eth_ioctl(eth_handle, ETH_CMD_G_MAC_ADDR, mac_addr);
        ESP_LOGI(TAG, "Ethernet Link Up");
        ESP_LOGI(TAG, "Ethernet HW Addr %02x:%02x:%02x:%02x:%02x:%02x",
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}


bool eth_start(void) {
  s_eth_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  char *desc;
  esp_netif_inherent_config_t esp_netif_config = ESP_NETIF_INHERENT_DEFAULT_ETH();
  asprintf(&desc, "%s: %s", TAG, esp_netif_config.if_desc);
  esp_netif_config.if_desc = desc;
  esp_netif_config.route_prio = 64;
  esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_ETH();

  esp_netif_t *netif = esp_netif_new(&netif_config);
  assert(netif);
  free(desc);

  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();

  phy_config.autonego_timeout_ms = 1000;
  s_mac = esp_eth_mac_new_openeth(&mac_config);
  s_phy = esp_eth_phy_new_dp83848(&phy_config);

  uint8_t mac_addr[] = {0x00, 0x00, 0x12, 0x34, 0x56, 0x78};
  ESP_ERROR_CHECK(esp_base_mac_addr_set(mac_addr));

  esp_eth_config_t config = ETH_DEFAULT_CONFIG(s_mac, s_phy);
  ESP_ERROR_CHECK(esp_eth_driver_install(&config, &s_eth_handle));

  s_eth_glue = esp_eth_new_netif_glue(s_eth_handle);
  esp_netif_attach(netif, s_eth_glue);

    // Register user defined event handers
  ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

  esp_eth_start(s_eth_handle);

  EventBits_t bits = xEventGroupWaitBits(s_eth_event_group,
          ETH_CONNECTED_BIT | ETH_FAIL_BIT,
          pdFALSE,
          pdFALSE,
          10000 / portTICK_PERIOD_MS);

  if (bits & ETH_CONNECTED_BIT) {
      ESP_LOGI(TAG, "Ethernet setup finished");
  } else if (bits & ETH_FAIL_BIT) {
      ESP_LOGI(TAG, "Failed to setup ethernet");
  } else {
      ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }

  return ((bits & ETH_CONNECTED_BIT) != 0);
}
