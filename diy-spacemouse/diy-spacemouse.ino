/*
 * Copyright (c) 2025, lraton 
 * All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include <Adafruit_TinyUSB.h>
#include "TLx493D_inc.hpp"
#include <SimpleKalmanFilter.h>
#include <OneButton.h>

//--------------------------------------------------------------------+
// MSC RAM Disk Config
//--------------------------------------------------------------------+

Adafruit_USBD_MSC usb_msc;

// HID Report ID for keyboard and mouse
enum {
  RID_KEYBOARD = 1,
  RID_MOUSE,
};

// USB HID report descriptor
uint8_t const desc_hid_report[] = {
  TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(RID_KEYBOARD)),
  TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(RID_MOUSE))
};

// USB HID object
Adafruit_USBD_HID usb_hid;

using namespace ifx::tlx493d;

// Definition of the power pin and sensor objects for Kit2Go XMC1100 boards.
const uint8_t POWER_PIN = 15;  // XMC1100 : LED2

// Magnetic sensor initialization
TLx493D_A1B6 mag(Wire, TLx493D_IIC_ADDR_A0_e);

// Kalman Filters for noise reduction in x, y, z axes
SimpleKalmanFilter xFilter(1, 1, 0.2), yFilter(1, 1, 0.2), zFilter(1, 1, 0.2);

// Setup buttons
OneButton button1(2, true);
OneButton button2(3, true);

// Offset variables for calibration
float xOffset = 0, yOffset = 0, zOffset = 0;
float xCurrent = 0, yCurrent = 0, zCurrent = 0;

// Calibration and movement sensitivity settings
int calSamples = 50;
int sensivity = 8;
int magRange = 3;
int outRange = 127;       // Max allowed in HID report
float xyThreshold = 0.4;  // Center threshold

int inRange = magRange * sensivity;
float zThreshold = xyThreshold * 2;

bool isOrbit = false;
bool middle = false;

// HID key definitions
uint8_t key_none[6] = { HID_KEY_NONE };
uint8_t key_h[6] = { HID_KEY_H };

/** Calibrate the magnetic sensor by averaging multiple readings. */
void calibrate() {
  double x, y, z;
  Wire.begin();
  mag.setPowerPin(POWER_PIN, OUTPUT, INPUT, HIGH, LOW, 0, 250000);
  mag.begin();

  for (int i = 1; i <= calSamples; i++) {
    mag.getMagneticField(&x, &y, &z);
    xOffset += x;
    yOffset += y;
    zOffset += z;
  }

  xOffset /= calSamples;
  yOffset /= calSamples;
  zOffset /= calSamples;
}

/** Initialize USB HID and buttons. */
void setup() {
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  usb_msc.begin();
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.setBootProtocol(HID_ITF_PROTOCOL_NONE);
  usb_hid.setPollInterval(2);
  usb_hid.begin();

  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }

  Serial.begin(115200);
  calibrate();

  button1.attachClick(btn1);
  button2.attachClick(btn2);
}

/** Process HID mouse movement. */
void process_hid(int x, int y) {
  if (usb_hid.ready()) {
    usb_hid.mouseButtonPress(RID_MOUSE, MOUSE_BUTTON_MIDDLE);
    middle = true;
    delay(20);
    usb_hid.mouseReport(RID_MOUSE, MOUSE_BUTTON_MIDDLE, x, y, 0, 0);
  }
}

/** Read magnetic field and apply filtering. */
void getMagnet() {
  double x, y, z;
  mag.getMagneticField(&x, &y, &z);

  xCurrent = xFilter.updateEstimate(x - xOffset);
  yCurrent = yFilter.updateEstimate(y - yOffset);
  zCurrent = zFilter.updateEstimate(z - zOffset);

  static uint32_t ms = 0;
  static uint32_t ms2 = 0;

  if (abs(xCurrent) > xyThreshold || abs(yCurrent) > xyThreshold) {
    int xMove = map(xCurrent, -inRange, inRange, -outRange, outRange);
    int yMove = map(yCurrent, -inRange, inRange, -outRange, outRange);

    if (abs(zCurrent) > zThreshold) {
      usb_hid.keyboardReport(RID_KEYBOARD, KEYBOARD_MODIFIER_LEFTSHIFT, key_none);
      isOrbit = true;
      ms2 = millis();
    }

    ms = millis();
    process_hid(xMove, -yMove);
  } else {
    if (millis() - ms > 200 && middle) {
      usb_hid.mouseButtonRelease(RID_MOUSE);
      middle = false;
      delay(10);
    }
    if (millis() - ms2 > 200 && isOrbit) {
      usb_hid.keyboardRelease(RID_KEYBOARD);
      isOrbit = false;
      delay(10);
    }
  }

  Serial.print(xCurrent);
  Serial.print(",");
  Serial.print(yCurrent);
  Serial.print(",");
  Serial.print(zCurrent);
  Serial.println();
}

/** Main loop: watches buttons and processes HID inputs. */
void loop() {
  button1.tick();
  button2.tick();

#ifdef TINYUSB_NEED_POLLING_TASK
  TinyUSBDevice.task();
#endif

  if (!TinyUSBDevice.mounted()) {
    return;
  }
  getMagnet();
}

/** Button 1 action: Simulate Shift + GUI + H keypress. */
void btn1() {
  usb_hid.keyboardReport(RID_KEYBOARD, KEYBOARD_MODIFIER_LEFTSHIFT + KEYBOARD_MODIFIER_LEFTGUI, key_h);
  delay(10);
  usb_hid.keyboardRelease(RID_KEYBOARD);
}

/** Button 2 action: Simulate double middle mouse click. */
void btn2() {
  usb_hid.mouseButtonPress(RID_MOUSE, MOUSE_BUTTON_MIDDLE);
  delay(10);
  usb_hid.mouseButtonRelease(RID_MOUSE);
  delay(10);
  usb_hid.mouseButtonPress(RID_MOUSE, MOUSE_BUTTON_MIDDLE);
  delay(10);
  usb_hid.mouseButtonRelease(RID_MOUSE);
}
