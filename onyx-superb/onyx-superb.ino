// Onyx firmware for Macchina SuperB

#include <ArduinoWebsockets.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <PacketSerial.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "config.h"

#if COMM_INTF == COMM_WIFI
  #define WANT_WIFI 1
#elif COMM_INTF == COMM_BLE
  #define WANT_BLE 1
#endif

#if WANT_SUPERB_LOGGING == 1
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

// BLE interface
#define BLE_SERVICE_NAME "Onyx M2"
#define BLE_SERVICE_UUID "e9377e45-d4d2-4fdc-9e1c-448d8b4e05d5"
#define BLE_RELAY_CHARACTERISTIC_UUID "8e9e4115-30a8-4ce6-9362-5afec3315d7d"
#define BLE_COMMAND_CHARACTERISTIC_UUID "25b9cc8b-9741-4beb-81fc-a0df9b155f8d"
#define BLE_MESSAGE_CHARACTERISTIC_UUID "7d363f56-9154-4168-8ee8-034a216edfb4"

// M2 interface
// This allows the SuperB to relay commands from apps and send notifications to the M2.
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
#define NOTIFY_BLE_CONNECTED 0x06
#define NOTIFY_BLE_DISCONNECTED 0x07

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
bool bleConnected = false;

using namespace websockets;

#ifdef WANT_WIFI

WiFiMulti wifiMulti;
WebsocketsClient WS;

#endif // WANT_WIFI

PacketSerial M2;

BLECharacteristic* pBleRelayCharacteristic = NULL;
BLECharacteristic* pBleCommandCharacteristic = NULL;
BLECharacteristic* pBleMessageCharacteristic = NULL;

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

#ifdef WANT_WIFI

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

#endif // WANT_WIFI

// M2 message callback
// The M2 will push messages here. Always attempt to send on to the web socket. If that
// fails, and there's a ble device currently connected, notify it of the new message.
void onM2(const uint8_t* buffer, size_t size) {
  bool sent = false;
  #if WANT_WIFI == 1
  sent = WS.sendBinary((const char*)buffer, size);
  #endif
  //if (bleConnected && !sent) {
    pBleMessageCharacteristic->setValue(const_cast<uint8_t*>(buffer), size);
    pBleMessageCharacteristic->notify(true);
  //}
}

class BLECallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    LOG("BLE CONNECTED");
    bleConnected = true;
    sendM2(M2_NOTIFICATION, NOTIFY_BLE_CONNECTED);
  };
  void onDisconnect(BLEServer* pServer) {
    LOG("BLE DISCONNECTED");
    bleConnected = false;
    sendM2(M2_NOTIFICATION, NOTIFY_BLE_DISCONNECTED);
    BLEDevice::startAdvertising();
  }
};

class BLERelayCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    uint8_t* data = (uint8_t*) value.data();
    if (data[0] == 1) {
      // todo: enrich the characteristic to indicate mobile vs home based on
      // the phone being on wifi or lte
      sendM2(M2_NOTIFICATION, NOTIFY_WS_UP);
    }
    else {
      sendM2(M2_NOTIFICATION, NOTIFY_WS_DOWN);
    }
  }
};

class BLECommandCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    uint8_t* data = (uint8_t*) value.data();
    int length = value.length();
    if (length < MAX_CMD_LENGTH) {
      sendM2(M2_COMMAND, data, length);
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(10);
  M2.setStream(&Serial);
  M2.setPacketHandler(&onM2);

  #ifdef WANT_WIFI

  if (WIFI_SSID_HOME) {
    wifiMulti.addAP(WIFI_SSID_HOME, WIFI_PASSWORD_HOME);
  }

  if (WIFI_SSID_MOBILE) {
    wifiMulti.addAP(WIFI_SSID_MOBILE, WIFI_PASSWORD_MOBILE);
  }

  WS.onMessage(&onWSMessage);
  WS.onEvent(&onWSEvent);

  #endif

  BLEDevice::init(BLE_SERVICE_NAME);

  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BLECallbacks());
  BLEService* pService = pServer->createService(BLE_SERVICE_UUID);

  pBleRelayCharacteristic = pService->createCharacteristic(
    BLE_RELAY_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE
  );
  pBleRelayCharacteristic->setCallbacks(new BLERelayCallbacks());

  pBleCommandCharacteristic = pService->createCharacteristic(
    BLE_COMMAND_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE
  );
  pBleCommandCharacteristic->setCallbacks(new BLECommandCallbacks());

  pBleMessageCharacteristic = pService->createCharacteristic(
    BLE_MESSAGE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pBleMessageCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
}

void loop() {

    uint32_t now = millis();
    uint32_t wifiLastAttempt = 0;
    uint32_t wsLastAttempt = 0;

    M2.update();

    //delay(2000);

/// TODO >>>> BLE fallback in loops // less aggressive wifi reconnects? // async retries // disconnect wifi on bad socket

  #ifdef WANT_WIFI

  // check for wifi needing to be connected
  wifiConnected = WiFi.status() == WL_CONNECTED;
  if (!wifiConnected && (now - wifiLastAttempt > 10000)) {
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
      wifiLastAttempt = now;
    }
  }

  // check for connection to ws server
  if (wifiConnected && !wsConnected && (now - wsLastAttempt > 1000)) {
    LOG("Connecting to ws server");
    wsLastAttempt = now;
    if (!WS.connect(WEBSOCKET_URL)) {
      LOG("Connection failed, retring in 1s");
    }
  }

  // main pumps for ws and m2 (will trigger event callbacks if appropriate)
  WS.poll();
  M2.update();

  // if the ws is connected, check the state of the connection
  if (wsConnected) {
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

  #endif // WANT_WIFI
}
