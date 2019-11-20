#define ARDUINO_MAIN

#include <Box2D/Box2D.h>
#include <stdio.h>
#include <math.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <MPU6050.h>
#include "Arduino.h"
#include "SdFat.h"
#include "sdios.h"

// application includes
#include "core.h"
#include "renderer.h"
#include "physics.h"
#include "rfid.h"

#define RENDER_TO_SERIAL_MONITOR false
#define MAIN_CIRCLE_INDEX 3
#define NUM_CIRCLES 12
#define SD_SS SDCARD_SS_PIN

SdFat sd;
MPU6050 accelGyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
int t = 0;
char msg[100];
bool message_exists = false;

void setup() {
    Serial.begin(921600);
    Wire.begin();
    SPI.begin();

    accelGyro.initialize();
    setup_renderer();
    setup_physics(NUM_CIRCLES, MAIN_CIRCLE_INDEX);
    setup_rfid();

    if (!sd.begin(SD_SS, SD_SCK_MHZ(50))) {
        sd.initErrorHalt();
    }
}

void loop() {
    rfid_update(t, msg, &message_exists);
    accelGyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    physics_update(ay, ax);
    render_all(ground, is_upside_down, NUM_CIRCLES, MAIN_CIRCLE_INDEX, the_bodies, message_exists, msg, RENDER_TO_SERIAL_MONITOR);
    t++;
}