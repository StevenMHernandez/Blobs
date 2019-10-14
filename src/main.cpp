#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>
#include <MPU6050.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SharpMem.h>

#define RST_PIN         5
#define SS_PIN          53

MFRC522 rfidReader(SS_PIN, RST_PIN);

MPU6050 accelGyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    SPI.begin();

    accelGyro.initialize();

    rfidReader.PCD_Init();
    // Delay (potentially) required for MFRC522
    delay(100);

    rfidReader.PCD_DumpVersionToSerial();
}

void loop() {
    if (rfidReader.PICC_IsNewCardPresent() and rfidReader.PICC_ReadCardSerial()) {
        rfidReader.PICC_DumpToSerial(&(rfidReader.uid));
    }

    accelGyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);
}