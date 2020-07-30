//   ___  _  ___   ____  __  __  __ ___
//  / _ \| \| \ \ / /\ \/ / |  \/  |_  )
// | (_) | .` |\ V /  >  <  | |\/| |/ /
//  \___/|_|\_| |_|  /_/\_\ |_|  |_/___|
//
// Firmware for Macchina M2
// https://github.com/onyx-m2/onyx-m2-firmware

#include <variant.h>
#include <due_can.h>
#include <PacketSerial.h>

// Logging is now controlled by the built in "core debug level", and means you
// can control it from the Arduino menu (or vscode board config panel). If you don't
// see anything other than "None", add the relevant entries in boards.txt for superb.
#define PRINT(...)
#define LOG_E(...)
#define LOG_W(...)
#define LOG_I(...)
#define LOG_D(...)
#ifndef CORE_DEBUG_LEVEL
  #error "Update your boards.txt file to include CORE_DEBUG_LEVEL as per the README"
#endif
#if CORE_DEBUG_LEVEL > 0
  char __log_buffer[256];
  #define PRINT(...) sprintf(__log_buffer, __VA_ARGS__);SerialUSB.println(__log_buffer);
  #define OUTPUT_LOG(level, ...) sprintf(__log_buffer, __VA_ARGS__);SerialUSB.print("[");SerialUSB.print(level);SerialUSB.print("][m2:");SerialUSB.print(__LINE__);SerialUSB.print("] ");SerialUSB.println(__log_buffer);
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

// Serial maps to the BLE link xbee
// BLE allows you transfer maximum of 20 bytes, so we'll keep everything under that
// limit to allow atomic transmission of CAN messages (which works out well as CAN
// messages max out at 16 bytes).
// The format of the transmitted message is [timestamp | bus | id | length | data]:
// uint32_t timestamp (little endian milliseconds)
// uint8_t bus (the source bus, vehicle:0, chassis:1)
// uint16_t id (little endian message id)
// uint8_t length (number of bytes of data)
// uint8_t data[1..8] (signal data, up to 8 bytes worth)
#define Xbee Serial
PacketSerial SuperB;

// Tesla Model 3 Can buses
// Can0 is connected to the vehicle bus, under the center console at the back.
// Can1 is connected to the chassis bus, under the passenger seat.
#define VehicleCan Can0
#define VEHICLE_BUS 0
#define ChassisCan Can1
#define CHASSIS_BUS 1

// Mock is a mock device that is enabled when something is connected to the usb port.
// This can be used to inject can message directly into the M2 for testing. It is
// designed to be used with the 'bin/onyx-serial-replay' script.
PacketSerial Mock;

// Mode constants
// The firmware has 2 completely separate modes of operation
// In RUN mode, the firmware runs normally
// In SUPERB mode, the firmware provides a direct link to the SuperB, allowing it to
// flashed and debugged
#define MODE_RUN 1
#define MODE_SUPERB 2

// State constants
// The firmware can be in one of 3 states: idle (no can messages), passive (can messages
// received but not transmitted), and active.
#define STATE_IDLE 0
#define STATE_PASSIVE 1
#define STATE_ACTIVE 2

// Message size related constants
#define CAN_MSG_SIZE 8        // Size of fixed part of can message
#define CAN_MSG_MAX_LENGTH 8  // Maximum frame length
#define CAN_BUS_COUNT 2       // Number of can buses
#define CAN_MSG_COUNT 0x800   // Number of message ids
#define CAN_MSG_MAX_SIZE 15   // Maximum size of a can message
#define CAN_MSG_TS_OFFSET 0   // Offset of timestamp
#define CAN_MSG_ID_OFFSET 4   // Offset of message id
#define CAN_MSG_LEN_OFFSET 6  // Offset of message payload length

// Rate limiting constant
// This limits any one message id to the specified period in milliseconds
#define CAN_MSG_RATE_LIMIT 250

// Message flags
// Each message can have up to 8 flags associated with it
// These flags are set remotely by using the command interface

// Flags this message for realtime OTA transmission
#define CAN_MSG_FLAG_TRANSMIT 0x01

// Flags this message for real time OTA transmission even if its value is unmodified
#define CAN_MSG_FLAG_TRANSMIT_UNMODIFIED 0x02

// SuperB interface
// This allows the SuperB to relay commands from apps and send notifications to the M2
// The format is <u8:id, u8:len, u8[len]:data>.
#define SUPERB_COMMAND 0x01
#define SUPERB_NOTIFICATION 0x02

// Command interface
// This allows apps to send commands on realtime to the M2
// Commands have no specific format, other than the first byte being the command id
// and the second byte being the length of the command
#define CMDID_SET_ALL_MSG_FLAGS 0x01
#define CMDID_SET_MSG_FLAGS 0x02
#define CMDID_GET_MSG_LAST_VALUE 0x03

// Notification interface
// This allows the SuperB to send notifications in realtime to the M2
#define NOTIFY_WIFI_UP 0x01
#define NOTIFY_WIFI_DOWN 0x02
#define NOTIFY_WS_UP 0x03
#define NOTIFY_WS_DOWN 0x04
#define NOTIFY_LATENCY 0x05
#define NOTIFY_BLE_CONNECTED 0x06
#define NOTIFY_BLE_DISCONNECTED 0x07

// Commands related constants
#define SUPERB_TYPE_OFFSET 0
#define SUPERB_ID_OFFSET 1
#define SUPERB_LENGTH_OFFSET 2
#define SUPERB_DATA_OFFSET 3

// Message ids
#define MSGID_UNIX_TIME 1320

// LED colours
// When powered, the red led is, flashing the version number at startup.
// When CAN traffic is detected, the colour changes to blue.
// When packets are actually sent to the SuperB for transmission to the server,
// the colour is green.
#define LED_IDLE RGB_RED
#define LED_PASSIVE RGB_BLUE
#define LED_ACTIVE RGB_GREEN

// The connection status of the SuperB is indicated using surface mount leds (not
// visible in the enclosure). Yellow means wifi is up, green means ws is up.
#define LED_WIFI_UP DS5
#define LED_WS_UP DS6
#define LED_BLE_CONNECTED DS4

// The button activated SuperB programming mode is indicated with another set of
// surface mount leds
#define LED_SUPERB_YELLOW DS3
#define LED_SUPERB_RED DS2

// How fast can an led change colour
#define LEDCFG_CHANGE_PERIOD 1000

// Firmware buffers
// The message flags buffer hold the current flag values for all messages.
// The message buffer holds all incoming frames from the Tesla. This ends up being
// very large, but the memory is there, so might as well use it. Having this allows
// us to do away with ring buffers means we get write without blocking ever (mostly).
uint8_t canMsgFlags[CAN_BUS_COUNT][CAN_MSG_COUNT];
uint8_t canMsgBuffer[CAN_BUS_COUNT][CAN_MSG_COUNT][CAN_MSG_MAX_SIZE];

// Global state variables
uint8_t mode = MODE_RUN;
uint32_t lastActiveMillis = 0;
uint32_t lastPassiveMillis = 0;
uint32_t lastLedChangeMillis = 0;

// Clear all message buffers.
void clearAllMsgBuffers() {
  uint8_t* pb = &canMsgBuffer[0][0][0];
  for (int i = CAN_BUS_COUNT * CAN_MSG_COUNT * CAN_MSG_MAX_SIZE; i > 0; i--) {
    *pb++ = 0;
  }
}

// Set the specified flags to all messages.
void setAllMsgFlags(uint8_t flags) {
  uint8_t* pb = &canMsgFlags[0][0];
  for (int i = CAN_BUS_COUNT * CAN_MSG_COUNT; i > 0; i--) {
    *pb++ = flags;
  }
}

// Set the specified flags of the specified message id.
void setMsgFlags(uint8_t bus, uint16_t id, uint8_t flags) {
  canMsgFlags[bus][id] = flags;
}

// Verify if the specified flag is set for the specified message id.
bool isMsgFlagSet(uint8_t bus, uint16_t id, uint8_t flag) {
  return (canMsgFlags[bus][id] & flag) == flag;
}

// Extract a uint32 value from a buffer (little endian).
uint32_t getUint32(const uint8_t* pmsg) {
  uint32_t v = pmsg[0]
    | ((uint32_t) pmsg[1]) << 8
    | ((uint32_t) pmsg[2]) << 16
    | ((uint32_t) pmsg[3]) << 24;
  return v;
}

// Extract a uint16 value from a buffer (little endian).
uint32_t getUint16(const uint8_t* pmsg) {
  uint16_t v = pmsg[0]
    | ((uint16_t) pmsg[1]) << 8;
  return v;
}

// Get the last timestamp of the specified message id.
uint32_t getMsgTimestamp(uint8_t bus, uint16_t id) {
  return getUint32(&canMsgBuffer[bus][id][CAN_MSG_TS_OFFSET]);
}

// Transmit the message of the specified id, if it has a value (timestamp is not zero).
void transmitMsg(uint8_t bus, uint16_t id) {
  if (getMsgTimestamp(bus, id) != 0) {
    uint8_t* pmsg = canMsgBuffer[bus][id];
    uint8_t size = CAN_MSG_SIZE + pmsg[CAN_MSG_LEN_OFFSET];
    LOG_D("SB <- %d, size: %d", (int) pmsg, size);
    transmitBuffer(pmsg, size);
  }
}

// Transmit the message buffer, but only if the message flag says so.
// The state is active if the message was sent, and passive if the
// message was ignored.
void transmitBuffer(const uint8_t* buf, uint8_t size) {
  SuperB.send(buf, size);
}

// Command processing. Each command has its own function, so this function only figures
// what the command it and dispatches it.
void onSuperB(const uint8_t* buf, size_t size) {
  LOG_D("SuperB msg, size: %d", size);
  uint8_t type = buf[SUPERB_TYPE_OFFSET];
  uint8_t id = buf[SUPERB_ID_OFFSET];
  uint8_t len = buf[SUPERB_LENGTH_OFFSET];
  if (len != size - SUPERB_DATA_OFFSET) {
    LOG_W("Invalid superb message, length mismatch, type: %d, id: %d, len: %d, size: %d", type, id, len, size);
    return;
  }
  const uint8_t* data = &buf[SUPERB_DATA_OFFSET];
  if (type == SUPERB_COMMAND) {
    onSuperBCommand(id, data);
  }
  else if (type == SUPERB_NOTIFICATION) {
    onSuperBNotification(id, data);
  }
}

void onSuperBCommand(uint8_t id, const uint8_t* data) {
  switch (id) {
    case CMDID_SET_ALL_MSG_FLAGS: {
      uint8_t flags = data[0];
      LOG_D("CMDID_SET_ALL_MSG_FLAGS, flags: %d", flags);
      setAllMsgFlags(flags);
      break;
    }

    case CMDID_SET_MSG_FLAGS: {
      uint16_t bus = data[0];
      uint16_t id = getUint16(&data[1]);
      uint8_t flags = data[3];
      LOG_D("CMDID_SET_MSG_FLAGS, bus: %d, id: %d, flags: %d", bus, id, flags);
      setMsgFlags(bus, id, flags);
      break;
    }

    case CMDID_GET_MSG_LAST_VALUE: {
      uint16_t bus = data[0];
      uint16_t id = getUint16(&data[1]);
      LOG_D("CMDID_GET_MSG_LAST_VALUE, bus: %d, id: %d", bus, id);
      transmitMsg(bus, id);
      break;
    }

    default:
      LOG_W("Unknown superb command, id: %d", id);
  }
}

void onSuperBNotification(uint8_t id, const uint8_t* data) {
  switch (id) {

    case NOTIFY_WIFI_UP:
      LOG_I("WIFI UP");
      digitalWrite(LED_WIFI_UP, LOW);
      break;

    case NOTIFY_WIFI_DOWN:
      LOG_I("WIFI DOWN");
      digitalWrite(LED_WS_UP, HIGH);
      digitalWrite(LED_WIFI_UP, HIGH);
      break;

    case NOTIFY_WS_UP:
      LOG_I("WS UP");
      digitalWrite(LED_WS_UP, LOW);
      break;

    case NOTIFY_WS_DOWN:
      LOG_I("WS DOWN");
      digitalWrite(LED_WS_UP, HIGH);
      break;

    case NOTIFY_LATENCY: {
      uint16_t latency = getUint16(&data[0]);
      int8_t rssi = (int8_t) data[2];
      LOG_D("WS, latency: %d, RSSI: %d", latency, rssi);
      break;
    }

    case NOTIFY_BLE_CONNECTED:
      LOG_I("BLE CONNECTED");
      digitalWrite(LED_BLE_CONNECTED, LOW);
      break;

    case NOTIFY_BLE_DISCONNECTED:
      LOG_I("BLE DISCONNECTED");
      digitalWrite(LED_WS_UP, HIGH);
      digitalWrite(LED_BLE_CONNECTED, HIGH);
      break;

    default:
      LOG_W("Unknown superb notification: %d", id);
  }
}

// Mock interface processing.
void onMock(const uint8_t* buf, size_t size) {
  uint16_t id = getUint16(&buf[CAN_MSG_ID_OFFSET]);
  transmitBuffer(buf, size);
}

// Message processing. If the can bus has a message available, we grab it and figure
// out what to do with it.
uint8_t pollCanBus(CANRaw& can, uint8_t bus, uint32_t now) {
  bool hasMessages = (can.available() > 0);
  if (!hasMessages) {
    return STATE_IDLE;
  }

  CAN_FRAME frame;
  can.read(frame);
  uint16_t id = (uint16_t) frame.id;
  uint8_t length = frame.length;
  uint8_t* data = frame.data.bytes;
  bool modified = false;

  // verify that the frame is valid from our point of view
  // (we only support standard frames, and length is clamped)
  if (id < CAN_MSG_COUNT && length <= CAN_MSG_MAX_LENGTH) {
    uint8_t* pmsg = canMsgBuffer[bus][id];
    uint32_t prevts = getMsgTimestamp(bus, id);

    // apply per message rate limiting, ignoring any that are arriving too quickly
    if (now - prevts > CAN_MSG_RATE_LIMIT) {
      LOG_D("M2 -> %d %d", id, (int) pmsg);

      *pmsg++ = (uint8_t)(now & 0xff);
      *pmsg++ = (uint8_t)(now >> 8);
      *pmsg++ = (uint8_t)(now >> 16);
      *pmsg++ = (uint8_t)(now >> 24);
      *pmsg++ = bus;
      *pmsg++ = (uint8_t)(frame.id & 0xff);
      *pmsg++ = (uint8_t)(frame.id >> 8);
      *pmsg++ = length;

      // super annoying bug with unix time being big endian while everything else
      // is little endian; let's not slow down regular processing and just have
      // explicit copy statements for unix time
      if (id != MSGID_UNIX_TIME) {
        while (length--) {
          *pmsg++ = *data++;
          // uint8_t src = *data++;
          // uint8_t dst = *pmsg;
          // if (src != dst) {
          //   modified = true;
          //   *pmsg = src;
          // }
          // pmsg++;
        }
      }
      else {
        pmsg[0] = data[3];
        pmsg[1] = data[2];
        pmsg[2] = data[1];
        pmsg[3] = data[0];
      }

      // only transmit if this msg if its transmit flag is set
      if (isMsgFlagSet(bus, id, CAN_MSG_FLAG_TRANSMIT)) {
      // if ((isMsgFlagSet(id, CAN_MSG_FLAG_TRANSMIT) && modified) ||
      //     isMsgFlagSet(id, CAN_MSG_FLAG_TRANSMIT_UNMODIFIED)) {
        transmitBuffer(canMsgBuffer[bus][id], CAN_MSG_SIZE + frame.length);
        return STATE_ACTIVE;
      }
    }
  }
  return STATE_PASSIVE;
}

// Set the status using the led colours.
void setStateLedStatus(uint32_t led) {
  digitalWrite(LED_IDLE, HIGH);
  digitalWrite(LED_PASSIVE, HIGH);
  digitalWrite(LED_ACTIVE, HIGH);
  if (led != 0) {
    digitalWrite(led, LOW);
  }
}

// Main controller setup. Initialize the led we'll be using for activity,
// the BleLink and Tesla interfaces, and our internal buffers.
void setup() {
  // LEDs used for RUN mode
  pinMode(LED_IDLE, OUTPUT);
  pinMode(LED_PASSIVE, OUTPUT);
  pinMode(LED_ACTIVE, OUTPUT);
  pinMode(LED_WIFI_UP, OUTPUT);
  pinMode(LED_WS_UP, OUTPUT);
  pinMode(LED_BLE_CONNECTED, OUTPUT);
  digitalWrite(LED_WIFI_UP, HIGH);
  digitalWrite(LED_WS_UP, HIGH);
  digitalWrite(LED_BLE_CONNECTED, HIGH);

  // Mode switching buttons and leds
  pinMode(LED_SUPERB_RED, OUTPUT);
  pinMode(LED_SUPERB_YELLOW, OUTPUT);
  pinMode(Button1, INPUT);
  pinMode(Button2, INPUT);
  digitalWrite(LED_SUPERB_RED, HIGH);
  digitalWrite(LED_SUPERB_YELLOW, HIGH);

  // The USB port is used either as a mock interface that allows direct injection of
  // CAN message on the M2, or a channel to flash the SuperB.
  SerialUSB.begin(115200);
  Mock.setStream(&SerialUSB);
  Mock.setPacketHandler(&onMock);
  LOG_D("USB setup done");

  PRINT(" ---------------------------------------------");
  PRINT("| O N Y X  M 2                                ");
  PRINT("|                                             ");
  PRINT("| https://github.com/onyx-m2/onyx-m2-firmware ");
  PRINT("| Revision 8095668                            ");
  PRINT("| %s", __DATE__);
  PRINT(" ---------------------------------------------");

  // SuperB is reached through the Xbee serial interface, which uses COBS encoding
  Xbee.begin(115200);
  SuperB.setStream(&Xbee);
  SuperB.setPacketHandler(&onSuperB);
  LOG_D("SuperB setup done");

  // We connect to the Tesla Model 3 chassis and vehicle buses, read-only, and we'll
  // initialize all 7 receive mailboxes to accept any standard frame (TM3 doesn't
  // appear to use extended frames)
  VehicleCan.begin(CAN_BPS_500K);
  ChassisCan.begin(CAN_BPS_500K);
  for (int i = 0; i < 7; i++) {
    VehicleCan.setRXFilter(i, 0, 0, false);
    ChassisCan.setRXFilter(i, 0, 0, false);
  }
  LOG_D("CAN bus setup done");

  // initialize our internal buffers
  clearAllMsgBuffers();
  setAllMsgFlags(0);
  LOG_D("Buffer initialization done");

  // Turn on the red idle light to indicate we're good to go
  setStateLedStatus(LED_IDLE);
}

// Main controller loop. The buttons are checked (any press causes the mode to switch
// to MODE_SUPERB), and the appropriate mode handler is dispatched.
void loop() {
  if (mode == MODE_RUN) {
    runModeLoop();
    if (digitalRead(Button1) == LOW || digitalRead(Button2) == LOW) {
      mode = MODE_SUPERB;
      PRINT("Entering SuperB mode");
      PRINT("Hold BTN2 while pressing then releasing BTN1 to enter programming mode");
      pinMode(XBEE_RST, OUTPUT);
      pinMode(XBEE_MULT4, OUTPUT);
      digitalWrite(LED_IDLE, HIGH);
      digitalWrite(LED_BLE_CONNECTED, HIGH);
      digitalWrite(LED_WS_UP, HIGH);
      digitalWrite(LED_WIFI_UP, HIGH);
    }
  }
  else {
    superbModeLoop();
  }
}

// Main MODE_RUN loop. SuperB is updated to check for commands and notifications,
// and the leds are adjusted based on the state.
void runModeLoop() {
  SuperB.update();
  uint32_t now = millis();
  uint8_t vehicleState = pollCanBus(VehicleCan, VEHICLE_BUS, now);
  uint8_t chassisState = pollCanBus(ChassisCan, CHASSIS_BUS, now);

  // update mock interface if usb is connected and no real can messages were received
  if (vehicleState == STATE_IDLE && chassisState == STATE_IDLE) {
    Mock.update();
  }

  // store the timing of the last active and passive states
  if (vehicleState == STATE_ACTIVE || chassisState == STATE_ACTIVE) {
    lastActiveMillis = now;
  }
  if (vehicleState == STATE_PASSIVE || chassisState == STATE_PASSIVE) {
    lastPassiveMillis = now;
  }

  // if the time interval since the last led change has expired, update the led
  // with the current state; if there were any active states during the interval,
  // the active led is show, if not passive states are checked, and if neither
  // of those occurred, the idle led is shown
  if (now - lastLedChangeMillis > LEDCFG_CHANGE_PERIOD) {
    if (now - lastActiveMillis < LEDCFG_CHANGE_PERIOD) {
      setStateLedStatus(LED_ACTIVE);
    }
    else if (now - lastPassiveMillis < LEDCFG_CHANGE_PERIOD) {
      setStateLedStatus(LED_PASSIVE);
    }
    else {
      setStateLedStatus(LED_IDLE);
    }
    lastLedChangeMillis = now;
  }
}

// Main MODE_SUPERB loop. In this mode, the USB port is wired to the SuperB directly so
// that the latter can be programmed directly if needed. This allows the SuperB to be
// debugged by allowing its logging messages to get to the USB serial monitor.
bool idleLedState = HIGH;
void superbModeLoop() {
  if (SerialUSB.available() > 0) {
    Serial.write(SerialUSB.read());
  }
  if (Serial.available() > 0) {
    SerialUSB.write(Serial.read());
  }

  int btn1 = digitalRead(Button1);
  if (btn1 == LOW) {
    digitalWrite(LED_SUPERB_RED, LOW);
    digitalWrite(XBEE_RST, LOW);
  } else {
    digitalWrite(LED_SUPERB_RED, HIGH);
    digitalWrite(XBEE_RST, HIGH);
  }

  int bnt2 = digitalRead(Button2);
  if (bnt2 == LOW) {
    digitalWrite(LED_SUPERB_YELLOW, LOW);
    digitalWrite(XBEE_MULT4, LOW);
  } else {
    digitalWrite(LED_SUPERB_YELLOW, HIGH);
    digitalWrite(XBEE_MULT4, HIGH);
  }

  uint32_t now = millis();
  if (now - lastLedChangeMillis > 1000) {
    lastLedChangeMillis = now;
    idleLedState = (idleLedState == HIGH) ? LOW : HIGH;
    digitalWrite(LED_IDLE, idleLedState);
  }
}
