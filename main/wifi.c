/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * ESP deck firmware
 *
 * Copyright (C) 2022 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "wifi.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>
#include "esp_netif.h"

#include "com.h"
#define BLINK_GPIO 4

static esp_routable_packet_t rxp;
static esp_routable_packet_t txp;

#define MAX_SSID_SIZE (50)
#define MAX_PASSWD_SIZE (50)

static char ssid[MAX_SSID_SIZE];
static char key[MAX_SSID_SIZE];

static const int WIFI_CONNECTED_BIT = BIT0;
static const int WIFI_SOCKET_DISCONNECTED = BIT1;
static const int WIFI_PACKET_WAIT_SEND = BIT2;
static const int WIFI_PACKET_SENDING = BIT3;
static EventGroupHandle_t s_wifi_event_group;

static const int START_UP_MAIN_TASK = BIT0;
static const int START_UP_RX_TASK = BIT1;
static const int START_UP_TX_TASK = BIT2;
static const int START_UP_CTRL_TASK = BIT3;
static EventGroupHandle_t startUpEventGroup;


#define NO_CONNECTION -1
#define WIFI_HOST_QUEUE_LENGTH (2)
// The TX queue must absorb a full image frame's burst of CPX chunks (~15-18 for a
// 15 KB JPEG), which the GAP8 pushes over SPI far faster than the socket drains.
// A 2-deep queue dropped mid-frame chunks -> truncated JPEGs on the viewer.
#define WIFI_TX_QUEUE_LENGTH (24)
// How long wifi_transport_send will wait for queue space before dropping a chunk.
// Lossless within a frame for a healthy client; bounded so a dead/stuck client
// can't wedge the router (the socket send() is itself bounded by SO_SNDTIMEO).
#define WIFI_TX_ENQUEUE_TIMEOUT_MS (50)

static xQueueHandle wifiRxQueue;
static xQueueHandle wifiTxQueue;

/* Log printout tag */
static const char *TAG = "WIFI";

/* Socket for receiving WiFi connections */
static int serverSock = -1;
/* Accepted WiFi connection */
static int clientConnection = NO_CONNECTION;

enum {
  WIFI_CTRL_SET_SSID                = 0x10,
  WIFI_CTRL_SET_KEY                 = 0x11,

  WIFI_CTRL_WIFI_CONNECT            = 0x20,

  WIFI_CTRL_STATUS_WIFI_CONNECTED   = 0x31,
  WIFI_CTRL_STATUS_CLIENT_CONNECTED = 0x32,
};

/* WiFi event handler */
static void event_handler(void* handlerArg, esp_event_base_t eventBase, int32_t eventId, void* eventData)
{
  if (eventBase == WIFI_EVENT) {
    switch(eventId) {
      case WIFI_EVENT_STA_START:
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;
      case WIFI_EVENT_STA_DISCONNECTED:
        ESP_ERROR_CHECK(esp_wifi_connect());
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG,"Disconnected from access point");
        break;
      case WIFI_EVENT_AP_STACONNECTED:
        {
          wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*)eventData;
          ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",
                    MAC2STR(event->mac),
                    event->aid);
        }
        break;
      case WIFI_EVENT_AP_STADISCONNECTED:
        {
          wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*)eventData;
          ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d",
                    MAC2STR(event->mac),
                    event->aid);
        }
        break;
      default:
        // Fall through
        break;
    }
  }

  if (eventBase == IP_EVENT) {
    switch (eventId) {
      case IP_EVENT_STA_GOT_IP:
        {
          ip_event_got_ip_t* event = (ip_event_got_ip_t*)eventData;
          ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&event->ip_info.ip));

          wifi_ap_record_t ap_info;
          ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&ap_info));
          ESP_LOGD(TAG, "BSAP MAC is %x:%x:%x:%x:%x:%x",
              ap_info.bssid[0], ap_info.bssid[1], ap_info.bssid[2],
              ap_info.bssid[3], ap_info.bssid[4], ap_info.bssid[5]);
          ESP_LOGI(TAG, "country: %s", ap_info.country.cc);
          ESP_LOGI(TAG, "rssi: %d", ap_info.rssi);
          ESP_LOGI(TAG, "11b: %d, 11g: %d, 11n: %d, lr: %d",
            ap_info.phy_11b, ap_info.phy_11g, ap_info.phy_11n, ap_info.phy_lr);

          cpxInitRoute(CPX_T_ESP32, CPX_T_GAP8, CPX_F_WIFI_CTRL, &txp.route);
          txp.data[0] = WIFI_CTRL_STATUS_WIFI_CONNECTED;
          memcpy(&txp.data[1], &event->ip_info.ip.addr, sizeof(uint32_t));
          txp.dataLength = 1 + sizeof(uint32_t);

          // TODO: We should probably not block here...
          espAppSendToRouterBlocking(&txp);

          txp.route.destination = CPX_T_STM32;
          espAppSendToRouterBlocking(&txp);

          xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
        break;
      default:
        // Fall through
        break;
    }
  }
}

/* Initialize WiFi as AP */
static void wifi_init_softap(const char *ssid, const char* key)
{
  esp_netif_t* ap_netif = esp_netif_create_default_wifi_ap();
  assert(ap_netif);

  wifi_config_t wifi_config = {
      .ap = {
          .ssid_len = strlen(ssid),
          // Allow several associations so a reconnecting client can join a free
          // slot immediately instead of waiting for the previous (stale)
          // station to age out of the AP's table. We still only serve one TCP
          // client at a time at the socket layer.
          .max_connection = 4,
          .authmode = WIFI_AUTH_OPEN},
  };
  strncpy((char *)wifi_config.ap.ssid, ssid, strlen(ssid));
  if (strlen(key) > 0) {
    strncpy((char *)wifi_config.ap.password, key, strlen(key));
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_softap finished");
}

static void wifi_init_sta(const char * ssid, const char * key)
{
  esp_netif_t* sta_netif = esp_netif_create_default_wifi_sta();
  assert(sta_netif);

  wifi_config_t wifi_config;
  memset((void *)&wifi_config, 0, sizeof(wifi_config_t));
  strncpy((char *)wifi_config.sta.ssid, ssid, strlen(ssid));
  ESP_LOGD(TAG, "SSID is %u chars", strlen(ssid));
  strncpy((char *)wifi_config.sta.password, key, strlen(key));
  ESP_LOGD(TAG, "KEY is %u chars", strlen(key));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
  ESP_ERROR_CHECK(esp_wifi_start() );

  ESP_LOGI(TAG, "wifi_init_sta finished.");

}

static void wifi_ctrl(void* _param) {
  xEventGroupSetBits(startUpEventGroup, START_UP_CTRL_TASK);
  while (1) {
    com_receive_wifi_ctrl_blocking(&rxp);

    switch (rxp.data[0]) {
      case WIFI_CTRL_SET_SSID:
        ESP_LOGD("WIFI", "Should set SSID");
        memcpy(ssid, &rxp.data[1], rxp.dataLength - 1);
        ssid[rxp.dataLength - 1 + 1] = 0;
        ESP_LOGD(TAG, "SSID: %s", ssid);
        // Save to NVS?
        break;
      case WIFI_CTRL_SET_KEY:
        ESP_LOGD("WIFI", "Should set password");
        memcpy(key, &rxp.data[1], rxp.dataLength - 1);
        key[rxp.dataLength - 1 + 1] = 0;
        ESP_LOGD(TAG, "KEY: %s", key);
        // Save to NVS?
        break;
      case WIFI_CTRL_WIFI_CONNECT:
        ESP_LOGD("WIFI", "Should connect");

        if (strlen(ssid) > 0) {
          if (rxp.data[1] == 0) {
            wifi_init_sta(ssid, key);
          } else {
            if (0 < strlen(key) && strlen(key) < 8) {
              ESP_LOGW(TAG, "Password too short, cannot initialize AP");
            } else {
              wifi_init_softap(ssid, key);
            }
          }
        } else {
          ESP_LOGW(TAG, "No SSID set, cannot start wifi");
        }

        break;
    }
  }
}

static portMUX_TYPE closeMux = portMUX_INITIALIZER_UNLOCKED;
static void close_client_socket()
{
    // Both the sending and receiving tasks can detect a disconnect, so claim
    // the fd atomically: only the first caller closes it and signals, which
    // also prevents closing a freshly accepted (reused) fd.
    int fd;
    taskENTER_CRITICAL(&closeMux);
    fd = clientConnection;
    clientConnection = NO_CONNECTION;
    taskEXIT_CRITICAL(&closeMux);

    if (fd != NO_CONNECTION) {
        close(fd);
        xEventGroupSetBits(s_wifi_event_group, WIFI_SOCKET_DISCONNECTED);
    }
}

void wifi_bind_socket() {
  char addr_str[128];
  int addr_family;
  int ip_protocol;
  struct sockaddr_in destAddr;
  destAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  destAddr.sin_family = AF_INET;
  destAddr.sin_port = htons(5000);
  addr_family = AF_INET;
  ip_protocol = IPPROTO_IP;
  inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);
    serverSock = socket(addr_family, SOCK_STREAM, ip_protocol);
  if (serverSock < 0) {
    ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
  }
  ESP_LOGD(TAG, "Socket created");

  int err = bind(serverSock, (struct sockaddr *)&destAddr, sizeof(destAddr));
  if (err != 0) {
    ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
  }
  ESP_LOGD(TAG, "Socket binded");

  // Backlog of 2 lets a reconnecting client's TCP handshake complete while the
  // previous (dead) connection is still being torn down, so it is accepted
  // immediately instead of having to retry.
  err = listen(serverSock, 2);
  if (err != 0) {
    ESP_LOGE(TAG, "Error occured during listen: errno %d", errno);
  }
  ESP_LOGD(TAG, "Socket listening");
}

void wifi_wait_for_socket_connected() {
  ESP_LOGI(TAG, "Waiting for connection");
  struct sockaddr sourceAddr;
  uint addrLen = sizeof(sourceAddr);
  clientConnection = accept(serverSock, (struct sockaddr *)&sourceAddr, &addrLen);
  if (clientConnection < 0) {
    ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
  } else {
     // Bound a blocking send() to 200ms so we SHED a stale frame fast when the
     // viewer falls behind, instead of letting 2s of data pile up. For a live
     // stream dropping the latest frame is correct; deep-buffering is what let
     // the connection choke and abort (errno 11 then errno 113 every ~5s). Safe
     // to be this short only because wifi_send_packet no longer closes on
     // EAGAIN -- it just drops the frame and keeps streaming.
     struct timeval snd_to = {.tv_sec = 0, .tv_usec = 200000};
     setsockopt(clientConnection, SOL_SOCKET, SO_SNDTIMEO, &snd_to, sizeof(snd_to));

     // Abortive close (RST) instead of a graceful FIN: when we close a dead
     // client, free the TCP state immediately so it never lingers in TIME_WAIT
     // (60s MSL) and block a reconnect that reuses the same 4-tuple.
     struct linger so_linger = {.l_onoff = 1, .l_linger = 0};
     setsockopt(clientConnection, SOL_SOCKET, SO_LINGER, &so_linger, sizeof(so_linger));

     // Actively probe the peer so a vanished client is detected even when the
     // TCP send buffer has not yet filled. Relaxed from 1s/1s/2 (~3s): that was
     // aggressive enough to abort a healthy connection during a multi-second
     // viewer render-freeze. 5s idle / 2s intvl / 3 probes => ~11s to declare a
     // silently-gone client dead, which is fine here.
     int ka = 1, idle = 5, intvl = 2, cnt = 3;
     setsockopt(clientConnection, SOL_SOCKET,  SO_KEEPALIVE,  &ka,    sizeof(ka));
     setsockopt(clientConnection, IPPROTO_TCP, TCP_KEEPIDLE,  &idle,  sizeof(idle));
     setsockopt(clientConnection, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
     setsockopt(clientConnection, IPPROTO_TCP, TCP_KEEPCNT,   &cnt,   sizeof(cnt));
     ESP_LOGI(TAG, "Connection accepted (fd=%d)", clientConnection);
  }
}

void wifi_wait_for_disconnect() {
  xEventGroupWaitBits(s_wifi_event_group, WIFI_SOCKET_DISCONNECTED, pdTRUE, pdFALSE, portMAX_DELAY);
}

static void wifi_task(void *pvParameters) {

  s_wifi_event_group = xEventGroupCreate();

  esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL, NULL);
  esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, NULL, NULL);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  uint8_t mac[6];
  ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_AP, mac));
  ESP_LOGD(TAG, "AP MAC is %x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  ESP_ERROR_CHECK(esp_wifi_get_mac(ESP_IF_WIFI_STA, mac));
  ESP_LOGD(TAG, "STA MAC is %x:%x:%x:%x:%x:%x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  wifi_bind_socket();

  xEventGroupSetBits(startUpEventGroup, START_UP_MAIN_TASK);
  while (1) {
    //blink_period_ms = 500;
    wifi_wait_for_socket_connected();
    ESP_LOGI(TAG, "Client connected");

    //blink_period_ms = 100;

    // Not thread safe!
    cpxInitRoute(CPX_T_ESP32, CPX_T_GAP8, CPX_F_WIFI_CTRL, &txp.route);
    txp.data[0] = WIFI_CTRL_STATUS_CLIENT_CONNECTED;
    txp.data[1] = 1;    // connected
    txp.dataLength = 2;
    espAppSendToRouterBlocking(&txp);

    txp.route.destination = CPX_T_STM32;
    espAppSendToRouterBlocking(&txp);

    // Probably not the best, should be handled in some other way?
    wifi_wait_for_disconnect();
    ESP_LOGI(TAG, "Client disconnected");

    // Not thread safe!
    cpxInitRoute(CPX_T_ESP32, CPX_T_GAP8, CPX_F_WIFI_CTRL, &txp.route);
    txp.data[0] = WIFI_CTRL_STATUS_CLIENT_CONNECTED;
    txp.data[1] = 0;    // disconnected
    txp.dataLength = 2;
    espAppSendToRouterBlocking(&txp);

    txp.route.destination = CPX_T_STM32;
    espAppSendToRouterBlocking(&txp);
  }
}

void wifi_led_task(void *pvParameters)
{
  int ledstate = 0;
  while(1) {
    if(clientConnection == NO_CONNECTION){
      gpio_set_level(BLINK_GPIO, !ledstate);
      ledstate = !ledstate;
      vTaskDelay(pdMS_TO_TICKS(500));
    } else {
      EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_PACKET_SENDING | WIFI_PACKET_WAIT_SEND  , pdFALSE,pdFALSE,portMAX_DELAY);
      if (bits & WIFI_PACKET_SENDING) {
        gpio_set_level(BLINK_GPIO,1);
        xEventGroupClearBits(s_wifi_event_group, WIFI_PACKET_SENDING);
      }
      if (bits & WIFI_PACKET_WAIT_SEND ) {
        gpio_set_level(BLINK_GPIO,0);
        xEventGroupClearBits(s_wifi_event_group, WIFI_PACKET_WAIT_SEND);
      }
    }
  }
}

void wifi_send_packet(const char * buffer, size_t size) {
  if (clientConnection != NO_CONNECTION) {
    ESP_LOGD(TAG, "Sending WiFi packet of size %u", size);
    xEventGroupSetBits(s_wifi_event_group, WIFI_PACKET_SENDING);
    int err = send(clientConnection, buffer, size, 0);
    if (err < 0) {
      // EAGAIN/EWOULDBLOCK == the SO_SNDTIMEO fired: the client is just slow
      // (e.g. a laggy viewer that stalled on rendering), not gone. Drop this
      // frame and keep streaming -- tearing down the socket only causes a
      // reconnect freeze. A genuinely dead client still gets closed: either a
      // hard send error here (ECONNRESET/EPIPE/EHOSTUNREACH) or TCP keepalive
      // (~3s) catches it.
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        ESP_LOGW(TAG, "send() timed out (errno %d); dropping frame, keeping connection", errno);
      } else {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        close_client_socket();
      }
    }
    xEventGroupSetBits(s_wifi_event_group, WIFI_PACKET_WAIT_SEND);
  }
}

static void wifi_sending_task(void *pvParameters) {
  static WifiTransportPacket_t txp_wifi;
  static CPXRoutablePacket_t qPacket;

  xEventGroupSetBits(startUpEventGroup, START_UP_TX_TASK);
  while (1) {
    xQueueReceive(wifiTxQueue, &qPacket, portMAX_DELAY);

    txp_wifi.payloadLength = qPacket.dataLength + CPX_ROUTING_PACKED_SIZE;

    cpxRouteToPacked(&qPacket.route, &txp_wifi.routablePayload.route);

    memcpy(txp_wifi.routablePayload.data, qPacket.data, qPacket.dataLength);

    wifi_send_packet((const char *)&txp_wifi, txp_wifi.payloadLength + 2);
  }
}

static void wifi_receiving_task(void *pvParameters) {
  static WifiTransportPacket_t rxp_wifi;
  int len;

  xEventGroupSetBits(startUpEventGroup, START_UP_RX_TASK);
  while (1) {
    len = recv(clientConnection, &rxp_wifi, 2, 0);
    if (len > 0) {
      ESP_LOGD(TAG, "Wire data length %i", rxp_wifi.payloadLength);
      int totalRxLen = 0;
      do {
        len = recv(clientConnection, &rxp_wifi.payload[totalRxLen], rxp_wifi.payloadLength - totalRxLen, 0);
        ESP_LOGD(TAG, "Read %i bytes", len);
        totalRxLen += len;
      } while (totalRxLen < rxp_wifi.payloadLength);
      ESP_LOG_BUFFER_HEX_LEVEL(TAG, &rxp_wifi, 10, ESP_LOG_DEBUG);
      xQueueSend(wifiRxQueue, &rxp_wifi, portMAX_DELAY);
    } else if (len == 0) {
      close_client_socket();  //Reading 0 bytes most often means the client has disconnected.
    } else {
      vTaskDelay(10);
    }
  }
}

void wifi_transport_send(const CPXRoutablePacket_t* packet) {
  assert(packet->dataLength <= WIFI_TRANSPORT_MTU - CPX_ROUTING_PACKED_SIZE);
  if (xQueueSend(wifiTxQueue, packet, pdMS_TO_TICKS(WIFI_TX_ENQUEUE_TIMEOUT_MS)) != pdTRUE) {
		// client gone or stuck past the bounded wait: discard this chunk rather
		// than wedge the router. SO_SNDTIMEO bounds the downstream send(), so the
		// queue always drains and this can't block the SPI path indefinitely.
  }
}

void wifi_transport_receive(CPXRoutablePacket_t* packet) {
  // Not reentrant safe. Assuming only one task is popping the queue
  static WifiTransportPacket_t qPacket;
  xQueueReceive(wifiRxQueue, &qPacket, portMAX_DELAY);

  packet->dataLength = qPacket.payloadLength - CPX_ROUTING_PACKED_SIZE;

  cpxPackedToRoute(&qPacket.routablePayload.route, &packet->route);

  memcpy(packet->data, qPacket.routablePayload.data, packet->dataLength);
}

void wifi_init() {
  esp_netif_init();

  s_wifi_event_group = xEventGroupCreate();

  wifiRxQueue = xQueueCreate(WIFI_HOST_QUEUE_LENGTH, sizeof(WifiTransportPacket_t));
  wifiTxQueue = xQueueCreate(WIFI_TX_QUEUE_LENGTH, sizeof(CPXRoutablePacket_t));

  startUpEventGroup = xEventGroupCreate();
  xEventGroupClearBits(startUpEventGroup, START_UP_MAIN_TASK | START_UP_RX_TASK | START_UP_TX_TASK | START_UP_CTRL_TASK);
  xTaskCreate(wifi_task, "WiFi TASK", 5000, NULL, 1, NULL);
  xTaskCreate(wifi_sending_task, "WiFi TX", 5000, NULL, 1, NULL);
  xTaskCreate(wifi_receiving_task, "WiFi RX", 5000, NULL, 1, NULL);
  xTaskCreate(wifi_led_task, "WiFi LED", 5000, NULL, 1, NULL);
  ESP_LOGI(TAG, "Waiting for main, RX and TX tasks to start");
  xEventGroupWaitBits(startUpEventGroup,
                      START_UP_MAIN_TASK | START_UP_RX_TASK | START_UP_TX_TASK,
                      pdTRUE, // Clear bits before returning
                      pdTRUE, // Wait for all bits
                      portMAX_DELAY);

  xTaskCreate(wifi_ctrl, "WiFi CTRL", 5000, NULL, 1, NULL);
  ESP_LOGI(TAG, "Waiting for CTRL task to start");
  xEventGroupWaitBits(startUpEventGroup,
                      START_UP_CTRL_TASK,
                      pdTRUE, // Clear bits before returning
                      pdTRUE, // Wait for all bits
                      portMAX_DELAY);

  ESP_LOGI("WIFI", "Wifi initialized");
}
