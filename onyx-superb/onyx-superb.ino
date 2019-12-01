// Onyx firmware for Macchina SuperB

#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <PacketSerial.h>
#include "config.h"

#if WANT_LOGGING == 1
  #define LOG_NLN(t) Serial.print(t);Serial.print(" ")
  #define LOG(t) Serial.println(t)
  #define LOG2(t1, t2) Serial.print(t1);Serial.print(" ");Serial.println(t2)
  #define LOG3(t1, t2, t3) Serial.print(t1);Serial.print(" ");Serial.print(t2);Serial.print(" ");Serial.println(t3)
#else
  #define LOG_NLN(t)
  #define LOG(t)
  #define LOG2(t1, t2)
  #define LOG3(t1, t2, t3)
#endif

// M2 interface
// This allows the SuperB to relay commands from apps and send notifications to the M2
// The format is <u8:id, u8:len, u8[len]:data>.
#define M2_COMMAND 0x01
#define M2_NOTIFICATION 0x02

// Notification interface
// This allows the SuperB to send notifications in realtime to the M2
#define NOTIFY_WIFI_UP 0x01
#define NOTIFY_WIFI_DOWN 0x02
#define NOTIFY_WS_UP 0x03
#define NOTIFY_WS_DOWN 0x04
#define NOTIFY_WS_LATENCY 0x05

// Wifi SSID types
#define WIFI_SSID_HOME_TYPE 0
#define WIFI_SSID_MOBILE_TYPE 1

// Maximum length of a command that can be received from the websocket
#define MAX_CMD_LENGTH 20
#define MAX_M2_MESSAGE_LENGTH 20

#define WS_CHECK_INTERVAL 2000

bool wsAlive = true;
uint32_t wsCheckMillis = 0;
uint32_t wsLatencyMillis = 0;

bool wifiHome = false;
bool wifiConnected = false;
bool wsConnected = false;

using namespace websockets;

WiFiMulti wifiMulti;
WebsocketsClient WS;
PacketSerial M2;

void sendM2(uint8_t type, uint8_t* data, uint8_t length) {
  uint8_t buf[MAX_M2_MESSAGE_LENGTH];
  uint8_t* p = buf;
  *p++ = type;
  for (int i = length; i > 0; i--) {
    *p++ = *data++;
  }
  M2.send(buf, length + 1);
}

void sendM2(uint8_t type, uint8_t id) {
  uint8_t buf[3];
  buf[0] = type;
  buf[1] = id;
  buf[2] = 0;
  #if WANT_LOGGING == 1
    LOG_NLN("M2 <-");
    for (int i = 0; i < 3; i++) {
      LOG_NLN(buf[i]);
    }
    LOG("");
  #else
    M2.send(buf, 3);
  #endif

}

void sendM2(uint8_t type, uint8_t id, uint8_t length, uint8_t d0 = 0, uint8_t d1 = 0, uint8_t d2 = 0) {
  uint8_t buf[MAX_M2_MESSAGE_LENGTH];
  uint8_t* p = buf;
  *p++ = type;
  *p++ = id;
  *p++ = length;
  if (length > 0) {
    *p++ = d0;
  }
  if (length > 1) {
    *p++ = d1;
  }
  if (length > 2) {
    *p++ = d2;
  }
  #if WANT_LOGGING == 1
    LOG_NLN("M2 <-");
    for (int i = 0; i < length + 3; i++) {
      LOG_NLN(buf[i]);
    }
    LOG("");
  #else
    M2.send(buf, length + 3);
  #endif
}

void onWSMessage(WebsocketsMessage message) {
  LOG3("WS message", message.isBinary(), message.length());
  int length = message.length();
  if (message.isBinary() && length < MAX_CMD_LENGTH) {
    uint8_t* data = (uint8_t*) message.c_str();
    #if WANT_LOGGING == 1
      LOG_NLN("M2 <-");
      for (int i = 0; i < length; i++) {
        LOG_NLN(data[i]);
      }
      LOG("");
    #else
      sendM2(M2_COMMAND, data, length);
    #endif
  }
}

void onWSEvent(WebsocketsEvent event, WSInterfaceString data) {
  switch (event) {

    case WebsocketsEvent::ConnectionOpened:
      LOG("WS UP");
      sendM2(M2_NOTIFICATION, NOTIFY_WS_UP);
      wsConnected = true;
      wsAlive = true;
      break;

    case WebsocketsEvent::ConnectionClosed:
      wsConnected = false;
      LOG("WS DOWN");
      sendM2(M2_NOTIFICATION, NOTIFY_WS_DOWN);
      break;

    case WebsocketsEvent::GotPing:
      LOG("WS got ping");
      break;

    case WebsocketsEvent::GotPong:
      wsLatencyMillis = millis() - wsCheckMillis;
      wsAlive = true;
      LOG2("WS got pong", wsLatencyMillis);
      sendM2(M2_NOTIFICATION, NOTIFY_WS_LATENCY, 3,
        (uint8_t) wsLatencyMillis & 0xFF, (uint8_t) wsLatencyMillis >> 8, WiFi.RSSI());
      break;
  }
}

void onM2(const uint8_t* buffer, size_t size) {
  WS.sendBinary((const char*)buffer, size);
}

void setup() {
  Serial.begin(115200);
  delay(10);
  M2.setStream(&Serial);
  M2.setPacketHandler(&onM2);

  if (WIFI_SSID_HOME) {
    wifiMulti.addAP(WIFI_SSID_HOME, WIFI_PASSWORD_HOME);
  }

  if (WIFI_SSID_MOBILE) {
    wifiMulti.addAP(WIFI_SSID_MOBILE, WIFI_PASSWORD_MOBILE);
  }

  WS.onMessage(&onWSMessage);
  WS.onEvent(&onWSEvent);
}

void loop() {

  // check for wifi needing to be connected
  wifiConnected = WiFi.status() == WL_CONNECTED;
  if (!wifiConnected) {
    LOG("Wifi DOWN, connecting");
    sendM2(M2_NOTIFICATION, NOTIFY_WIFI_DOWN);
    wifiConnected = wifiMulti.run() == WL_CONNECTED;//wifiConnect();
    if (wifiConnected) {
      LOG3("Wifi UP", WiFi.SSID(), WiFi.RSSI());
      uint8_t ssidType = WIFI_SSID_MOBILE_TYPE;
      if (WiFi.SSID() == WIFI_SSID_HOME) {
        ssidType = WIFI_SSID_HOME_TYPE;
      }
      sendM2(M2_NOTIFICATION, NOTIFY_WIFI_UP, 2, ssidType, WiFi.RSSI());
    }
    else {
      delay(1000);
    }
  }

  // check for connection to ws server
  if (wifiConnected && !wsConnected) {
    LOG("Connecting to ws server");
    if (!WS.connect(WEBSOCKET_URL)) {
      LOG("Connection failed, retring in 1s");
      delay(1000);
    }
  }

  // main pumps for ws and m2 (will trigger event callbacks if appropriate)
  WS.poll();
  M2.update();

  // if the ws is connected, check the state of the connection
  if (wsConnected) {
    uint32_t now = millis();
    if (now - wsCheckMillis > WS_CHECK_INTERVAL) {
      LOG2("WS alive check:", wsAlive);
      if (wsAlive) {
        wsAlive = false;
        wsCheckMillis = now;
        WS.ping();
      }
      else {
        WS.close();
      }
    }
  }
}
