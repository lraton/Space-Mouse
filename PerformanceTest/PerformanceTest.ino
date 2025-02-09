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

#include <Wire.h>

#define TLV493D_ADDR 0x5E  // Indirizzo del TLV493D

void setup() {
    Serial.begin(115200);
    Wire.begin();

    Serial.println("Attivazione TLV493D...");

    // Scriviamo nei registri per impostare la modalità "Measuring Mode"
    Wire.beginTransmission(TLV493D_ADDR);
    Wire.write(0x00); // Registro di controllo
    Wire.write(0x01); // Attiviamo il sensore
    Wire.endTransmission();

    delay(100);
}

void loop() {
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

        Serial.print("X: "); Serial.print(x);
        Serial.print(" Y: "); Serial.print(y);
        Serial.print(" Z: "); Serial.println(z);
    } else {
        Serial.println("Errore lettura dati!");
    }
    delay(500);
}
