# Onyx M2/SuperB Firmware

This project has 2 flashable Arduino firmwares for a Macchina M2 and SuperB device that implements a CANBUS access system for Tesla Model 3s. See details on the hardware here:
   - https://docs.macchina.cc/m2-docs/m2
   - https://docs.macchina.cc/superb-docs/superb

The firmware was developed in conjunction with
[tesla-onyx-m2-server](https://github.com/johnmccalla/tesla-onyx-m2-server), which implements the server that relays the data from the M2 to WebSocket clients.

NOTE: This documentation is a work in progress, and by this I mean it sucks and I need to improve it. So, I'm sorry. :)

## Flashing

The device must be flashed in 2 steps (because it is in fact 2 separate devices connected through the Xbee socket).

### Flashing the M2

1. Follow the instructions from Macchina to setup your Arduino environment if that's not done yet. See https://docs.macchina.cc/m2-docs/arduino for details.

2. Flash the M2 sketch [onyx-m2.ino](onyx-m2/onyx-m2.ino) onto to the M2.

3. The device should restart on its own. At this point, the M2 is in `run` mode, but the SuperB is not operational yet.

### Flashing the SuperB

1. Follow the instructions from Macchina to setup your Arduino environment for the SuperB (see the software section, skip the hardware section, https://docs.macchina.cc/superb-docs/flashing-superb#step-2-software)

2. Onyx M2 firmware is setup so that if you do the button combo, it'll drop into its `superb` mode, and allow the SuperB to be flashed as per the instructions from Macchina.

3. Before actually flashing, open [config.h](onyx-superb/config.h), and setup your home and mobile wifi information.

4. Also configure the websocket url and pin where your M2 server will be listening.

4. Flash the SuperB sketch [onyx-superb.ino](onyx-superb/onyx-superb.ino) to the device.

5. The device will *not* cycle on its own, you need to power down the M2 by pulling the USB cable and reconnecting.

## Debugging

If all went well (it won't have), the M2 should connect to the server immediatly upon being powered up, either on the workbench through the USB cable, or in the car, using the powered CANBUS tap.

To obtain logs, either firmware can be configured with `WANT_LOGGING` (but not both at the same time!). I suggest you start with the SuperB. Any connection issues should be logged to the serial monitor using the SuperB Arduino setup in this mode.

If the connection works, the M2 can be setup to log also, but note that the M2 will not run properly while the SuperB is logging.