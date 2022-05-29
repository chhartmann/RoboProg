#include <esp_eth.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_err.h>
#include <qemu_eth.h>

static const char *TAG = "qemu_eth";
static esp_eth_handle_t s_eth_handle = NULL;
static esp_eth_mac_t *s_mac = NULL;
static esp_eth_phy_t *s_phy = NULL;
static esp_eth_netif_glue_handle_t s_eth_glue = NULL;

static void got_ip_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
    const esp_netif_ip_info_t *ip_info = &event->ip_info;

    ESP_LOGI(TAG, "Ethernet Got IP Address");
    ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip_info->ip));
}

void eth_start(void) {
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

  phy_config.autonego_timeout_ms = 100;
  s_mac = esp_eth_mac_new_openeth(&mac_config);
  s_phy = esp_eth_phy_new_dp83848(&phy_config);

  uint8_t mac_addr[] = {0x00, 0x00, 0x12, 0x34, 0x56, 0x78};
  ESP_ERROR_CHECK(esp_base_mac_addr_set(mac_addr));

  esp_eth_config_t config = ETH_DEFAULT_CONFIG(s_mac, s_phy);
  ESP_ERROR_CHECK(esp_eth_driver_install(&config, &s_eth_handle));

  s_eth_glue = esp_eth_new_netif_glue(s_eth_handle);
  esp_netif_attach(netif, s_eth_glue);

    // Register user defined event handers
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

  esp_eth_start(s_eth_handle);
}
