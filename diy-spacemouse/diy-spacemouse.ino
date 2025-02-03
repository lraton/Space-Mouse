#include <TinyUSB_Mouse_and_Keyboard.h>
#include <OneButton.h>
#include <Wire.h>
#include <SimpleKalmanFilter.h>

// Setup buttons
OneButton button1(2, true);
OneButton button2(3, true);

float xOffset = 0, yOffset = 0, zOffset = 0;
float xCurrent = 0, yCurrent = 0, zCurrent = 0;

int calSamples = 300;
int sensivity = 8;
int magRange = 3;
int outRange = 127;      // Max allowed in HID report
float xyThreshold = 0.4; // Center threshold

int inRange = magRange * sensivity;
float zThreshold = xyThreshold * 1.5;

bool isOrbit = false;

#define TLV493D_ADDR 0x5E  // Indirizzo del TLV493D
SimpleKalmanFilter xFilter(1, 1, 0.2), yFilter(1, 1, 0.2), zFilter(1, 1, 0.2);

void setup() {
  button1.attachClick(goHome);
  button1.attachLongPressStop(goHome);

  button2.attachClick(fitToScreen);
  button2.attachLongPressStop(fitToScreen);

  // mouse and keyboard init
  Mouse.begin();
  Keyboard.begin();

  Serial.begin(115200);
  Wire.begin();

  Serial.println("Attivazione TLV493D...");

  // Attivazione sensore
  Wire.beginTransmission(TLV493D_ADDR);
  Wire.write(0x00); // Registro di controllo
  Wire.write(0x01); // Attiviamo il sensore
  Wire.endTransmission();

  delay(100);

  // crude offset calibration on first boot
  for (int i = 1; i <= calSamples; i++) {
    delay(10);  // Short delay for measurement time
    
    // Reading raw data
    Wire.beginTransmission(TLV493D_ADDR);
    Wire.write(0x00);  // Registro dati
    Wire.endTransmission();
    
    Wire.requestFrom(TLV493D_ADDR, 6);
    if (Wire.available() == 6) {
      uint8_t data[6];
      for (int i = 0; i < 6; i++) {
        data[i] = Wire.read();
      }

      int16_t x = ((data[0] << 4) | (data[4] >> 4));
      int16_t y = ((data[1] << 4) | (data[4] & 0x0F));
      int16_t z = ((data[2] << 4) | (data[5] & 0x0F));

      if (x > 2047) x -= 4096;
      if (y > 2047) y -= 4096;
      if (z > 2047) z -= 4096;

      xOffset += x;
      yOffset += y;
      zOffset += z;

      Serial.print(".");
    } else {
      Serial.println("Errore lettura dati!");
    }
  }

  xOffset = xOffset / calSamples;
  yOffset = yOffset / calSamples;
  zOffset = zOffset / calSamples;

  Serial.println();
  Serial.println(xOffset);
  Serial.println(yOffset);
  Serial.println(zOffset);
}

void loop() {
  // keep watching the push buttons
  button1.tick();
  button2.tick();

  // Read raw data
  delay(10);  // Wait for measurement to complete
  
  Wire.beginTransmission(TLV493D_ADDR);
  Wire.write(0x00);  // Registro dati
  Wire.endTransmission();
  
  Wire.requestFrom(TLV493D_ADDR, 6);
  if (Wire.available() == 6) {
    uint8_t data[6];
    for (int i = 0; i < 6; i++) {
      data[i] = Wire.read();
    }

    int16_t x = ((data[0] << 4) | (data[4] >> 4));
    int16_t y = ((data[1] << 4) | (data[4] & 0x0F));
    int16_t z = ((data[2] << 4) | (data[5] & 0x0F));

    if (x > 2047) x -= 4096;
    if (y > 2047) y -= 4096;
    if (z > 2047) z -= 4096;

    // Update the filters with the raw data minus the offset
    xCurrent = xFilter.updateEstimate(x - xOffset);
    yCurrent = yFilter.updateEstimate(y - yOffset);
    zCurrent = zFilter.updateEstimate(z - zOffset);

    // Check the center threshold
    if (abs(xCurrent) > xyThreshold || abs(yCurrent) > xyThreshold) {
      int xMove = 0;
      int yMove = 0;

      // Map the magnetometer x, y to the allowed 127 range for HID reports
      xMove = map(xCurrent, -inRange, inRange, -outRange, outRange);
      yMove = map(yCurrent, -inRange, inRange, -outRange, outRange);

      // Press shift to orbit in Fusion 360 if the pan threshold is not crossed (z-axis)
      if (abs(zCurrent) < zThreshold && !isOrbit) {
        Keyboard.press(KEY_LEFT_SHIFT);
        isOrbit = true;
      }

      // Pan or orbit by holding the middle mouse button and moving proportional to the xy axis
      Mouse.press(MOUSE_MIDDLE);
      Mouse.move(yMove, xMove, 0);
    } else {
      // Release the mouse and keyboard if within the center threshold
      Mouse.release(MOUSE_MIDDLE);
      Keyboard.releaseAll();
      isOrbit = false;
    }

    // Print the filtered data
    Serial.print(xCurrent);
    Serial.print(",");
    Serial.print(yCurrent);
    Serial.print(",");
    Serial.print(zCurrent);
    Serial.println();
  } else {
    Serial.println("Errore lettura dati!");
  }
}

// Go to home view in Fusion 360 by pressing (CMD + SHIFT + H) shortcut assigned to the custom Add-in command
void goHome() {
  Keyboard.press(KEY_LEFT_GUI);
  Keyboard.press(KEY_LEFT_SHIFT);
  Keyboard.write('h');

  delay(10);
  Keyboard.releaseAll();
  Serial.println("Pressed home");
}

// Fit to screen by pressing the middle mouse button twice
void fitToScreen() {
  Mouse.press(MOUSE_MIDDLE);
  Mouse.release(MOUSE_MIDDLE);
  Mouse.press(MOUSE_MIDDLE);
  Mouse.release(MOUSE_MIDDLE);

  Serial.println("Pressed fit");
}
