# Onyx M2/SuperB Firmware

This project has 2 flashable Arduino firmwares for a Macchina M2 and SuperB device that implements a CANBUS access system for Tesla Model 3s. See details on the hardware here:
   - https://docs.macchina.cc/m2-docs/m2
   - https://docs.macchina.cc/superb-docs/superb

The firmware was developed in conjunction with
[onyx-m2-server](https://github.com/onyx-m2/onyx-m2-server), which implements the server that relays the data from the M2 to WebSocket clients and broadcasts over BLE.

NOTE: This documentation is a work in progress, and by this I mean it sucks and I need to improve it. So, I'm sorry. :)

# Flashing

The device must be flashed in 2 steps (because it is in fact 2 separate devices connected through the Xbee socket). Step zero is to install the Arduino libraries required, using the library manager.  Currently these are:
  - ArduinoWebsockets
  - PacketSerial

## Flashing the M2

### Step 1
Follow the instructions from Macchina to setup your Arduino environment if that's not done yet. See https://docs.macchina.cc/m2-docs/arduino for details.

### Step 2
The firmware uses extra settings from the Arduino environment that aren't in the Macchina instructions, in
particular, for logging. You can set the logging levels desired in the board config menu (much like with the
esp32 does out of the box). To set this up, add the following lines to the `boards.txt` file for the `sam`
hardware, just after the entries for `m2`:

```
menu.DebugLevel=Core Debug Level
m2.menu.DebugLevel.none=None
m2.menu.DebugLevel.none.build.code_debug=0
m2.menu.DebugLevel.error=Error
m2.menu.DebugLevel.error.build.code_debug=1
m2.menu.DebugLevel.warn=Warn
m2.menu.DebugLevel.warn.build.code_debug=2
m2.menu.DebugLevel.info=Info
m2.menu.DebugLevel.info.build.code_debug=3
m2.menu.DebugLevel.debug=Debug
m2.menu.DebugLevel.debug.build.code_debug=4
m2.menu.DebugLevel.verbose=Verbose
m2.menu.DebugLevel.verbose.build.code_debug=5
```
Next find the line in the same file that starts with `m2.build.extra_flags`, and add:
```
-DCORE_DEBUG_LEVEL={build.code_debug}
```
As of this writing, the entire line should read:
```
m2.build.extra_flags=-D__SAM3X8E__ -mthumb {build.usb_flags} -DCORE_DEBUG_LEVEL={build.code_debug}
```

### Step 3
Flash the M2 sketch [onyx-m2.ino](onyx-m2/onyx-m2.ino) onto to the M2.

The device should restart on its own. At this point, the M2 is in `run` mode, but the SuperB is not operational yet.

## Flashing the SuperB

### Step 1
Follow the instructions from Macchina to setup your Arduino environment for the SuperB (see the software section, skip the hardware section, https://docs.macchina.cc/superb-docs/flashing-superb#step-2-software)

### Step 2
The firmware is now too large for the default partition scheme, and unfortunately Macchina's
instructions have issues. I explain the issues [on the forums here](https://forum.macchina.cc/t/superb-esp32-ble-wifi-crash-due-to-partitions/1274/4). The gist is:

- Change `--flash_size` from `detect` to `{build.flash_size}` in `platform.txt`
- Change the superb entries in `boards.txt` as shown below

The superb section of `boards.txt`:
```
superb.name=SuperB on M2

superb.upload.tool=esptool_py
superb.upload.speed=115200
superb.upload.maximum_data_size=327680
superb.upload.wait_for_upload_port=true

superb.serial.disableDTR=true
superb.serial.disableRTS=true

superb.build.mcu=esp32
superb.build.core=esp32
superb.build.variant=esp32
superb.build.board=esp32_DEV

superb.build.f_cpu=240000000L
superb.build.flash_size=4MB
superb.build.flash_freq=40m
superb.build.flash_mode=dio
superb.build.boot=dio
superb.build.partitions=min_spiffs
superb.build.defines=

superb.menu.PartitionScheme.min_spiffs=Default OTA App (Max 1.9MB APP) with 190KB SPIFFS
superb.menu.PartitionScheme.min_spiffs.build.partitions=min_spiffs
superb.menu.PartitionScheme.min_spiffs.upload.maximum_size=1966080
superb.menu.PartitionScheme.huge_app=Large App (Max 3MB) without OTA and a 1MB SPIFFS)
superb.menu.PartitionScheme.huge_app.build.partitions=huge_app
superb.menu.PartitionScheme.huge_app.upload.maximum_size=3145728

superb.menu.DebugLevel.none=None
superb.menu.DebugLevel.none.build.code_debug=0
superb.menu.DebugLevel.error=Error
superb.menu.DebugLevel.error.build.code_debug=1
superb.menu.DebugLevel.warn=Warn
superb.menu.DebugLevel.warn.build.code_debug=2
superb.menu.DebugLevel.info=Info
superb.menu.DebugLevel.info.build.code_debug=3
superb.menu.DebugLevel.debug=Debug
superb.menu.DebugLevel.debug.build.code_debug=4
superb.menu.DebugLevel.verbose=Verbose
superb.menu.DebugLevel.verbose.build.code_debug=5
```

### Step 3
Begin the upload process to flash the SuperB sketch [onyx-superb.ino](onyx-superb/onyx-superb.ino)
to the device. Once the build start to try to connect to the device:

1. Hit `BTN1` to drop M2 into superb mode, and reset the superb
2. Perform the hold `BTN2` while pressing and releasing `BTN1` and then releasing `BTN2`
to initiate the download.

Note: this step can be kind of finicky, and you might need a few attempts before it takes.

### Step 4
The device will *not* cycle on its own, you need to power down the M2 by pulling the USB cable and reconnecting.

# Debugging

If all went well (it won't have), the M2 should connect to the server immediately upon being powered up, either on the workbench through the USB cable, or in the car, using the powered CANBUS tap.

To obtain logs, either firmware can be configured with `WANT_LOGGING` (but not both at the same time!). I suggest you start with the SuperB. Any connection issues should be logged to the serial monitor using the SuperB Arduino setup in this mode.

If the connection works, the M2 can be setup to log also, but note that the M2 will not run properly while the SuperB is logging.

# Operations

## Main Status LED

The firmware uses the externally visible LED (`DS7`) to indicate active operational
status (as indicated by activity within the last second).

The LED that is visible outside of the enclosure is the main status indicator. When
first powered on, the LED will flash `RED` indicating the version number of the
firmware.

The LED will settle on solid `RED` after initialization, and indicates an idle status.
As soon as CAN traffic is detected, the LED changes to a solid `BLUE`. When CAN packets
make it through the filter and are sent to the SuperB for transmission to the server, a
solid `GREEN` is shown.

## Surface Mount LEDs

These LEDs are not visible if using an opaque enclosure, but can be useful for debugging
prior to final installation in the enclosure in the car.

Starting from the back of the M2 (nearest to the `BTN1` button, opposite from the USB connector)

- `DS6` is solid `GREEN` if the server's web socket is currently connected
- `DS5` is solid `YELLOW` if the wifi is up (connected to the access point specified
   in `config.h`)
- `DS4` (middle LED) is solid `YELLOW` if a BLE device is connected
- `DS3` and `DS2` are used to indicate the state of the SuperB mode. Pressing any
  button (`BTN1` or `BTN2`) will enter SuperB mode

## Entering SuperB Mode

Press in either `BTN1` or `BTN2` to enter SuperB mode, which routes all serial port traffic to the
SuperB directly. This allows programming of the SuperB, or debugging if logging is turned on.

To initiate programming of the SuperB, hold `BTN2` while pressing then releasing `BTN1`.

`BTN1` can be used by itself to power cycle the SuperB.

# Message Protocol

All of the above is just the preamble to what really matters, the protocol and what it
allows to actually do with the M2.

Communication is by directional, the M2 accepting commands that affect the flow of
message data out of it. The default state has the M2 ignore all messages on the bus
until it is asked by a client to `ENABLE` some or all messages.

## CAN Messages

The firmware strives to be as efficient and compact as possible in its encoding of
message, both to reduce latency and bandwidth requirements. Additionally, an effort
was made to keep each message under 20 bytes to allow each to be send in a single BLE
(Bluetooth Low Energy) packet.

All messages are binary coded as `timestamp` |`bus` | `id` | `length` | `data` tuples, where
  - `timestamp` is `4` bytes, representing a 32 bit little endian encoded milliseconds
    timestamp, zeroed at device startup
  - `id` is `2` bytes, representing a 16 bit little endian CAN message id (this
    corresponds to the message identifiers in dbc files)
  - `length` is a byte that indicates the number of bytes of message data in the
    following `data` part
  - `data` is `0` to `8` bytes of CAN signal data

There is currently rate limiting, on a per message basis, set at 250ms.

## M2 Commands

The firmware contains some basic commands that allow client to tailor the M2 to their
requirements. This is pretty basic for now, but much more is planned "soon".

The format of commands is simply `command` | `length` | `data`, where
  - `command` is a byte, indicating what command is being called (see below)
  - `length` is a byte that indicates the number of bytes of command data in the
    following `data` part
  - `data` is `0` to `16` bytes of command data

The current `command` ids are:
- `0x1` Set all message flags, where `length` is always `1` and `data` is a byte of flag
  values
- `0x2` Set a specific message's flags, where `length` is always `3` and `data` is 2
  bytes for the CAN message id (little endian) followed by a byte of flag values
- `0x3` Get the latest value of a specific message, which will be returned in a
  different message, and where `length` is always `2` and `data` is 2
  bytes for the CAN message id (little endian)

The only flag is `transmit(0x1)`, which indicates whether a message should be transmitted
if found on the bus. Clearing this flag will effectively _disable_ the message.

# BLE

The firmware sets up a BLE server named *Onyx M2* (UUID `e9377e45-d4d2-4fdc-9e1c-448d8b4e05d5`),
which will broadcast message data in real time, and accept commands.

There are 2 characteristics:

- *M2 Command* (`25b9cc8b-9741-4beb-81fc-a0df9b155f8d`) can be written to using the
  format described above, and will cause the command to be executed
- *CAN Message* (`7d363f56-9154-4168-8ee8-034a216edfb4`) can be read from, and will
  contain a CAN message encoded as described above

The intent of the BLE interface to use a smartphone with BLE as a fast, low latency,
and low power consumption relay.

A bit of (short) history: the first version on this used BLE with the help of a DRROBOT
XBee module and a smartphone relay. It was replaced by the current SuperB module because
it allows Wifi, which is great for syncing when at home. But it does require you to turn
on your phone's Wifi Hotspot to act as a relay. This drains the phone's battery very
quickly, and is annoying to have to toggle on and off. Thus, this new BLE interface
is the best of both worlds: keep both Wifi AP plus relay through BLE!

Eventually, I may also use the BLE interface for security. It's not great that the car
is blasting its telemetry to the public Internet using a simple pin as security.

# Future Development

Developing this firmware was a lot of fun and a great learning experience, but also
quite tedious and time consuming. Part of that was me trying to "do it right" and
"future proof it". Part of it is just the nature of the beast.

The following sections give some details about things I'd like to get to. If you are
reading this and have other ideas, or want to help implement some of these, please
open an issue so we can discuss it!

## More Commands

As mentioned above, there are more commands I'd like to add eventually. A few
possibilities might be:
  - Setting the rate limiting per message, or at least allowing to be changed
    remotely
  - Combining and/or filtering signals from multiple messages into a single packet
  - Being able to set certain messages as "only send if its data changes" would be
    super useful for alert type things, or the "sun up" signal
  - Being able to ask for a message, get its value, but not leave it broadcasting
    continuously

## Data Logging

From the very beginning of this project, I was planning on using the M2 to log
everything on the bus to the sd card. This would be great to grab CAN bus data
easily for study. I was planning on syncing that data with a server when the car
returns to the garage and the home wifi kicks in.

It would also be possible to make some pretty neat "trip reports" and collate
historical data on everything. Lots and lots of possibilities with this basic
feature!

It would be cool to also log superb related data here, including connection
hickups, latency, bandwidth usage, etc.

## Quality Of Life Improvements

One of the big ones here is having to take the enclosure off to flash the superb. It
would be great to be able to this through software, possibly by connecting to the usb
port.

DONE - EVERYTHING IS CONFIGURABLE USING BLE
<s>It would also be nice to be able to change the wifi parameters without flashing! The M2
has an onboard EEPROM, so it should be possible.</s>

DONE
<s>I'm also not pleased with the battery life of my phone when the hotspot is enabled.
Even if no devices are connected, it slams the battery, and so I've been toggling it
on and off - which is annoying. I'd love to revive my BLE relay and experience with
that instead of using the hotspot when on the road.</s>

SOLVED, BUT NOT EXACTLY LIKE THIS, NOW HANDLED CLEANLY BY THE SERVER
<s>Another "big one" would be to allow sessions to be managed on the device. Currently,
the M2 is 100% dependent on clients enabling and disabling messages, which is kind of
hard to manage in an environment with so many failure points between clients and the M2.
A much better system would be for the M2 to know what clients are still listening and
what messages they are interested in.</s>