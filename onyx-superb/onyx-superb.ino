//   ___  _  ___   ____  __  __  __ ___
//  / _ \| \| \ \ / /\ \/ / |  \/  |_  )
// | (_) | .` |\ V /  >  <  | |\/| |/ /
//  \___/|_|\_| |_|  /_/\_\ |_|  |_/___|
//
// Firmware for Macchina SuperB
// https://github.com/onyx-m2/onyx-m2-firmware

#include <ArduinoWebsockets.h>
#include <WString.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <PacketSerial.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <nvs_flash.h>
#include <nvs.h>

// Define the communication interfaces you want to use. If you choose both, they
// will work together, giving priority to the wifi connection (i.e. if both are
// connected, messages flow over wifi, not ble). BLE is now pretty much necessary
// for configuration at this point though.
#define WANT_WIFI 1
#define WANT_BLE 1

// Logging is now controlled by the built in "core debug level", and means you
// can control it from the Arduino menu (or vscode board config panel). If you don't
// see anything other than "None", add the relevant entries in boards.txt for superb.
#define PRINT(...)
#define LOG_E(...)
#define LOG_W(...)
#define LOG_I(...)
#define LOG_D(...)
#if CORE_DEBUG_LEVEL > 0
  char __log_buffer[256];
  #define PRINT(...) sprintf(__log_buffer, __VA_ARGS__);Serial.println(__log_buffer);
  #define OUTPUT_LOG(level, ...) sprintf(__log_buffer, __VA_ARGS__);Serial.print("[");Serial.print(level);Serial.print("][superb:");Serial.print(__LINE__);Serial.print("] ");Serial.println(__log_buffer);
  #define LOG_E(...) OUTPUT_LOG("E", __VA_ARGS__)
#endif
#if CORE_DEBUG_LEVEL > 1
  #define LOG_W(...) OUTPUT_LOG("W", __VA_ARGS__)
#endif
#if CORE_DEBUG_LEVEL > 2
  #define LOG_I(...) OUTPUT_LOG("I", __VA_ARGS__)
#endif
#if CORE_DEBUG_LEVEL > 3
  #define LOG_D(...) OUTPUT_LOG("D", __VA_ARGS__)
#endif

// BLE interface
#define BLE_SERVICE_NAME "Onyx M2"
#define BLE_SERVICE_UUID "e9377e45-d4d2-4fdc-9e1c-448d8b4e05d5"
#define BLE_CONFIG_CHARACTERISTIC_UUID "3c1a503d-06bd-4153-874c-c03e4866f19b"
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

// Interval (ms) to check for "aliveness" (ping/pong) of web socket
#define WS_CHECK_INTERVAL 2000

// Interval (ms) to reconnect to web socket while wifi is up
#define WS_RECONNECT_INTERVAL 1000

// Interval (ms) to reconnect to wifi ap when disconnected
#define WIFI_RECONNECT_INTERVAL 10000

char wsUrl[64];
bool wsConnected = false;
bool wsAlive = true;
uint32_t wsCheckMillis = 0;
uint32_t wsLatencyMillis = 0;
uint32_t wsLastAttempt = 0;

using namespace websockets;

bool wifiEnabled = false;
bool wifiConnected = false;
uint32_t wifiLastAttempt = 0;

bool bleConnected = false;

#if WANT_WIFI == 1

WiFiMulti wifiMulti;
WebsocketsClient WS;

#endif // WANT_WIFI

PacketSerial M2;

BLECharacteristic* pBleMessageCharacteristic = NULL;

class NVSClass {
  public:
    NVSClass();
    void begin(const String& ns);
    String read(String key);
    void write(const String& key, const String& value);

  private:
    const int MAX_NV_VALUE = 16;
    nvs_handle nvs;
};
NVSClass NVS;

NVSClass::NVSClass() : nvs(0) {
}

void NVSClass::begin(const String& ns) {
  LOG_D("Initializing nvs");
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    LOG_W("NVS partition was truncated and needs to be erased");
    nvs_flash_erase();
    nvs_flash_init();
  }
  LOG_D("Opening nvs, namespace: %s", ns.c_str());
  err = nvs_open(ns.c_str(), NVS_READWRITE, &nvs);
  if (err != ESP_OK) {
    LOG_E("Unable to open nvs, err: %x", err);
  }
}

String NVSClass::read(String key) {
  size_t len = MAX_NV_VALUE;
  char value[MAX_NV_VALUE];
  esp_err_t err = nvs_get_str(nvs, key.c_str(), value, &len);
  if (err != ESP_OK) {
    LOG_E("Unable to get string, err: %x", err);
    return String();
  }
  LOG_I("NVS read, %s=%s", key.c_str(), value);
  return String(value);
}

void NVSClass::write(const String& key, const String& value) {
  esp_err_t err = nvs_set_str(nvs, key.c_str(), value.c_str());
  if (err != ESP_OK) {
    LOG_E("Unable to set string, err: %x", err);
    return;
  }
  err = nvs_commit(nvs);
  if (err != ESP_OK) {
    LOG_E("Unable to commit data to nvs, err: %x", err);
    return;
  }
  LOG_I("NVS write, %s=%s", key.c_str(), value.c_str());
}

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
  LOG_D("Send m2, type: %d, id: %d", type, id);
  // we don't currently have a way to support both logging and send data to m2 (both use
  // the serial interface)
  #if CORE_DEBUG_LEVEL == 0
    uint8_t buf[3];
    buf[0] = type;
    buf[1] = id;
    buf[2] = 0;
    M2.send(buf, 3);
  #endif

}

void sendM2(uint8_t type, uint8_t id, uint8_t length, uint8_t d0 = 0, uint8_t d1 = 0, uint8_t d2 = 0) {
  LOG_D("Send m2, type: %d, id: %d, length: %d, d0: %x, d1: %x, d2: %x", type, id, length, d0, d1, d2);
  // we don't currently have a way to support both logging and send data to m2 (both use
  // the serial interface)
  #if CORE_DEBUG_LEVEL == 0
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
    M2.send(buf, length + 3);
  #endif
}

#if WANT_WIFI == 1

void onWSMessage(WebsocketsMessage message) {
  LOG_D("WS message, binary: %B, length: %d", message.isBinary(), message.length());
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
      LOG_D("Web sockets connection opened");
      sendM2(M2_NOTIFICATION, NOTIFY_WS_UP);
      wsConnected = true;
      wsAlive = true;
      break;

    case WebsocketsEvent::ConnectionClosed:
      LOG_D("Web socket connection closed");
      wsConnected = false;
      sendM2(M2_NOTIFICATION, NOTIFY_WS_DOWN);
      break;

    case WebsocketsEvent::GotPing:
      LOG_D("Web socket got ping");
      break;

    case WebsocketsEvent::GotPong:
      wsLatencyMillis = millis() - wsCheckMillis;
      wsAlive = true;
      LOG_D("Websocket got pong, latency: %d, rssi: %d", wsLatencyMillis, WiFi.RSSI());
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
  LOG_D("M2 message callback called, size: %d", size);
  bool sent = false;
  #if WANT_WIFI == 1
  sent = WS.sendBinary((const char*)buffer, size);
  #endif
  if (bleConnected && !sent) {
    pBleMessageCharacteristic->setValue(const_cast<uint8_t*>(buffer), size);
    pBleMessageCharacteristic->notify(true);
  }
}

class BLECallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    LOG_I("BLE server's client connected");
    bleConnected = true;
    sendM2(M2_NOTIFICATION, NOTIFY_BLE_CONNECTED);

    // restart advertising as soon as a device connects to support multiple
    // clients connecting (the library turns off advertising on connect by
    // default but otherwise supports multiple simultaneous clients)
    BLEDevice::startAdvertising();
  };
  void onDisconnect(BLEServer* pServer) {
    LOG_I("BLE server's client disconnected");
    bleConnected = false;
    sendM2(M2_NOTIFICATION, NOTIFY_BLE_DISCONNECTED);
  }
};

// Configuration
// This characteristic allows BLE clients to configure the M2.
// The format is <section><setting>=<value>.
// For example, to set the server hostname, you could send 'SH=onyx.net'.
class BLEConfigCallbacks: public BLECharacteristicCallbacks {

  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string data = pCharacteristic->getValue();
    if (data == "RESET") {
      LOG_I("Configuration write, reset requested, restarting")
      esp_restart();
    }

    std::string key = data.substr(0, 2);
    std::string value = data.substr(3);
    LOG_I("Configuration write, key: %s, value: %s", key.c_str(), value.c_str());
    NVS.write(key.c_str(), value.c_str());
  }
};

class BLERelayCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    uint8_t* data = (uint8_t*) value.data();
    if (data[0] == 1) {
      // todo: enrich the characteristic to indicate mobile vs home based on
      // the phone being on wifi or lte
      LOG_I("Relay notifying its web socket connected")
      sendM2(M2_NOTIFICATION, NOTIFY_WS_UP);
    }
    else {
      LOG_I("Relay notifying its web socket disconnected")
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
      LOG_I("Relay sent a command for m2");
      sendM2(M2_COMMAND, data, length);
    }
    else {
      LOG_W("Relay sent an invalid command, length: %s", length);
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(10);
  M2.setStream(&Serial);
  M2.setPacketHandler(&onM2);
  NVS.begin("onyx-m2");

  PRINT(" ---------------------------------------------");
  PRINT("| O N Y X  M 2 - S U P E R B                  ");
  PRINT("|                                             ");
  PRINT("| https://github.com/onyx-m2/onyx-m2-firmware ");
  PRINT("| Revision 8095668                            ");
  PRINT("| %s", __DATE__);
  #if WANT_BLE == 1
  PRINT("| Bluetooth support enabled                   ");
  #endif
  #if WANT_WIFI == 1
  PRINT("| Wifi support enabled                        ");
  #endif
  PRINT(" ---------------------------------------------");

  String hostname = NVS.read("SH");
  String pin = NVS.read("SP");
  sprintf(wsUrl, "ws://%s/m2?pin=%s", hostname.c_str(), pin.c_str());
  LOG_D("Web socket url is %s", wsUrl);

  #if WANT_WIFI == 1

  String homeWifiEnabled = NVS.read("HE");
  if (homeWifiEnabled[0] == '1') {
    wifiMulti.addAP(NVS.read("HS").c_str(), NVS.read("HP").c_str());
    wifiEnabled = true;
  }

  String mobileWifiEnabled = NVS.read("ME");
  if (mobileWifiEnabled[0] == '1') {
    wifiMulti.addAP(NVS.read("MS").c_str(), NVS.read("MP").c_str());
    wifiEnabled = true;
  }

  // undocumented requirement: wifiMulti.run() must be called first in setup() because
  // the first call is synchronous

  WS.onMessage(&onWSMessage);
  WS.onEvent(&onWSEvent);

  #endif

  LOG_D("Initializing BLE support");
  BLEDevice::init(BLE_SERVICE_NAME);

  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new BLECallbacks());
  BLEService* pService = pServer->createService(BLE_SERVICE_UUID);

  BLECharacteristic* pConfig = pService->createCharacteristic(
    BLE_CONFIG_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE
  );
  pConfig->setCallbacks(new BLEConfigCallbacks());

  BLECharacteristic* pRelay = pService->createCharacteristic(
    BLE_RELAY_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE
  );
  pRelay->setCallbacks(new BLERelayCallbacks());

  BLECharacteristic* pCommand = pService->createCharacteristic(
    BLE_COMMAND_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE
  );
  pCommand->setCallbacks(new BLECommandCallbacks());

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

  LOG_D("Starting BLE advertising");
  BLEDevice::startAdvertising();
}

void loop() {

  // main pumps for ws and m2 (will trigger event callbacks if appropriate)
  bool m2Activity = false;
  bool wsActivity = false;
  if (Serial.available() > 0) {
    M2.update();
    m2Activity = true;
  }
  #if WANT_WIFI == 1
  wsActivity = WS.poll();
  #endif

  // if no recent activity, introduce a short delay to mitigate chip overheating
  if (!m2Activity && !wsActivity) {
    delay(10);
  }

  #if WANT_WIFI == 1

  // if wifi isn't enabled because there are no configured APs, skip the wifi and
  // web socket connection logic below
  if (!wifiEnabled) {
    return;
  }

  uint32_t now = millis();

  // check for wifi needing to be connected
  wifiConnected = WiFi.status() == WL_CONNECTED;
  if (!wifiConnected && (now - wifiLastAttempt > WIFI_RECONNECT_INTERVAL)) {
    LOG_I("Wifi is down, now:%d, last attempt: %d, re-connecting", now, wifiLastAttempt);
    sendM2(M2_NOTIFICATION, NOTIFY_WIFI_DOWN);

    wifiConnected = wifiMulti.run() == WL_CONNECTED;
    if (wifiConnected) {
      LOG_I("Wifi is now up, ssid: %s, rssi: %s", WiFi.SSID(), WiFi.RSSI());
      // TODO: re-implement the Notification of which ssid this is
      // uint8_t ssidType = WIFI_SSID_MOBILE_TYPE;
      // if (WiFi.SSID() == WIFI_SSID_HOME) {
      //   ssidType = WIFI_SSID_HOME_TYPE;
      // }
      sendM2(M2_NOTIFICATION, NOTIFY_WIFI_UP, 2, WIFI_SSID_HOME_TYPE, WiFi.RSSI());
    }
    else {
      wifiLastAttempt = now;
    }
  }

  // check for connection to ws server
  // TODO: if the server isn't connecting because of bad wifi (which happens), it'll
  // stay stuck here
  if (wifiConnected && !wsConnected && (now - wsLastAttempt > WS_RECONNECT_INTERVAL)) {
    LOG_I("Web socket is down, now:%d, last attempt: %d, re-connecting", now, wsLastAttempt);
    wsLastAttempt = now;
    if (!WS.connect(wsUrl)) {
      LOG_W("Web socket connect failed, will retring in %d ms", WS_RECONNECT_INTERVAL);
    }
  }

  // if the ws is connected, check the state of the connection
  if (wsConnected) {
    if (now - wsCheckMillis > WS_CHECK_INTERVAL) {
      LOG_D("Web socket alive check: %d, now: %d, previous check: %d", wsAlive, now, wsCheckMillis);
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
