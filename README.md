# worc-remote-controller-esp32

Firmware for the WORC Remote Controller based on the Espressif ESP32 MCU and ESP-NOW protocol.

For more information about the project see:<br/>
https://www.wallieonline.nl/blogs/esp-now-remote-control-mini-robots.html

![WORC Remote Controller](https://img.youtube.com/vi/kkPaScOB8mg/hqdefault.jpg)

## Connecting the remote-controller to the robot-controller (ESP-NOW version)

No code changes or MAC addresses needed — the controllers find each other automatically.

1. Turn on the **robot** first. Its LED starts blinking: it is searching for a remote.
2. Within 30 seconds, turn on the **remote**. Its LED also blinks while searching.
3. When they find each other, the LEDs stay on — done! The remote now controls the robot.

Want to use a different remote? Turn both off and on again. Only turn on one remote at a time while connecting.

## Control inputs

The remote supports two input methods, detected automatically:

- **Analog joystick** — a two-axis joystick on the ESP32 ADC inputs (see wiring below).
- **CPPM input** — a CPPM sum signal on GPIO 12. If a CPPM signal is present, it takes priority over the joystick.

If neither input is detected, the remote keeps the link alive by broadcasting failsafe values (sticks neutral, throttle low).

## Wiring the ESP32 remote-controller for analog joystick usage

| ESP32        | Analog joystick |
| ------------ | --------------- |
| 3.3V         | 5V              |
| GND          | GND             |
| S-VP GPIO 36 | VRY             |
| S-VN GPIO 39 | VRX             |

## Programming the ESP32 with the Arduino IDE

- Install the Arduino IDE software from the Arduino website.
- After starting the software go to Preferences.
- Paste the "Additional Boards Manager URLs" for ESP32: https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
- Open the "Boards Manager" and install "esp32 by Espressif Systems version 2.0.14".
- Use version 2.0.14 — other versions may not work!
- Use a USB data transfer cable! Some cables are charge only!
- Put your ESP32 board in serial bootloader mode by keeping the Flash/BOOT button pressed when powering up.
- Install the driver for your ESP32 board if needed.
- Select the correct Board, Settings and port for your ESP32 board and click upload.

## License and attribution

This program is free software licensed under GPLv3 only (GPL-3.0-only, see [LICENSE](LICENSE)), with additional terms under Section 7 of the GPL.
The full text of these terms is in [NOTICE](NOTICE).
In short:

1. **Attribution:** if you distribute this software or a modified version, the credit "WORC Remote Controller — © 2026 wallieonline" with a link to https://github.com/wallieonline must be visibly included in the user interface or, if there is none, in the documentation and the source code headers.
   Do not remove or obscure the original copyright notices.
2. **Naming:** modified versions must be clearly marked as modified and use a distinct product name.
   The name "WORC" may not be used for modified versions.

These terms apply in addition to all GPLv3 conditions, including the requirement to publish your corresponding source code.