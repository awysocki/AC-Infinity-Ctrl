# AC Infinity Controller (ESP32-S2 Rev TFT)

This project controls an AC Infinity fan using an ESP32-S2 board and your external BS170 transistor circuit.

Current verified pin map (your existing build):

- `GPIO12` -> BS170 gate -> controls fan PWM line (`D+`)
- `GPIO13` <- fan FG/tach line (`D-`)

For tach conditioning, use your external resistor network on the tach line (no board rewiring required).

I used this as inspiration
https://www.reddit.com/r/homeassistant/comments/1lhveyu/ac_infinity_fan_esphome_control_with_tachometer/


## Files

- `WireDiagram.txt`: your hardware mapping
- `ACInfinityController/ACInfinityController.ino`: Arduino firmware

## 1) Install Arduino IDE + ESP32 support

1. Download and install Arduino IDE 2.x.
2. Open Arduino IDE.
3. Go to **File -> Preferences**.
4. In **Additional boards manager URLs**, add:
   - `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
5. Click **OK**.
6. Go to **Tools -> Board -> Boards Manager**.
7. Search for `esp32` and install **esp32 by Espressif Systems**.

## 1.5) Install display libraries

In Arduino IDE, go to **Tools -> Manage Libraries...** and install:

- `Adafruit GFX Library`
- `Adafruit ST7735 and ST7789 Library`

## 2) Connect board and select settings

1. Connect your ESP32-S2 Rev TFT to USB.
2. In Arduino IDE, open `ACInfinityController/ACInfinityController.ino`.
3. Select board:
   - **Tools -> Board -> ESP32 Arduino -> Adafruit Feather ESP32-S2**
   - If that exact option is not present, choose a close ESP32-S2 board profile from Adafruit.
4. Select COM port:
   - **Tools -> Port -> COMx** (the one that appears when you plug in the board).

## 3) Upload firmware

1. Click **Verify** (checkmark icon).
2. Click **Upload** (right-arrow icon).
3. If upload fails, hold **BOOT** while clicking upload, then tap **RESET**, then release BOOT when "Connecting..." appears.

## 4) Test fan control

1. The TFT should boot to a dashboard showing:
   - `Speed: xx%`
   - `RPM: xxxx`
   - On-screen `-10` and `+10` button boxes (visual in Phase 1)
2. Open **Serial Monitor**.
2. Set baud to **115200**.
3. Type commands and press Enter:
   - `help`
   - `status`
   - `+` (increase by 10%)
   - `-` (decrease by 10%)
   - `inc` (increase by 10%)
   - `dec` (decrease by 10%)
   - `speed 20`
   - `speed 50`
   - `speed 100`

The firmware reports measured RPM every 5 seconds.

The tach math uses a 5-second sample window and assumes 3 tach pulses per revolution, so the code can use the shortcut:

- `RPM = pulses in 5 seconds x 4`

Phase 1 note: on-screen `-10` and `+10` boxes are display indicators only. Tap input is not enabled yet on this hardware profile.

## 5) Physical button control (on-board)

The firmware now supports your board buttons:

- `D0`: decrease speed by 10%
- `D1`: jump to 50%
- `D2`: increase speed by 10%

Buttons use internal pull-up or pull-down mode based on the polarity settings in the sketch.

If only one button works on your board revision, adjust these booleans in the sketch:

- `BTN_MINUS_ACTIVE_LOW`
- `BTN_PRESET_ACTIVE_LOW`
- `BTN_PLUS_ACTIVE_LOW`

`true` means pressed reads `LOW`, `false` means pressed reads `HIGH`.

## Important safety note

Your `WireDiagram.txt` shows tach (`D-`) directly connected to GPIO13. If that tach line is higher than 3.3V, add a level shifter or resistor divider before the ESP32 pin.

## Tuning tips

- If speed control seems backwards, edit this line in the sketch and re-upload:
  - `PWM_ACTIVE_LOW_AT_FAN_LINE = true;`
  - change to `false` and test again.
- If RPM looks wrong, adjust:
  - `TACH_PULSES_PER_REV` (often 2)
- If fan does not react well, try different `PWM_FREQ_HZ` values (for example 1000, 5000, 25000).
