# worc-remote-controller-esp32
Firmware for the WORC Remote Controller based on the Espressif ESP32 MCU and ESP-NOW protocol.
For more information about the project see: (https://www.wallieonline.nl/blogs/esp-now-remote-control-mini-robots.html)

![WORC Robot Controller](https://img.youtube.com/vi/kkPaScOB8mg/hqdefault.jpg)

# Binding the remote-controller to the robot-controller
To bind the robot to your remote you need to change one line of code.
- Use readmac.ino on the ESP32-C3 Robot Controller to retrieve the MAC address.
- Save the MAC address.
- Open worc-esp32.ino in Arduino IDE and change the MAC adress in line 14 to the one you saved.

# Programming the ESP32 remote-controller
- Install the Arduino IDE software from the Arduino website.
- After starting the software go to preferences.
- Past the "Additional Boards Manager URLs" for ESP32. (https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json)
- Open the "Boards Manager" and install "esp32 by Espressive Systems version 2.0.14"
- Use version 2.0.14 other versions may not work!
- Use a USB data transfer kabel! Some cables are charge only!
- Install the driver for your ESP32 board if needed.
- Select the correct Board, Settings and port for your ESP32 board and click upload.