#define ARDUINO_MAIN

#include <Box2D/Box2D.h>
#include <stdio.h>
#include <math.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <MPU6050.h>
#include "Arduino.h"
#include <Adafruit_PN532.h>
#include "SdFat.h"
#include "sdios.h"
#include "core.h"
#include "renderer.h"



#define SD_SS SDCARD_SS_PIN
SdFat sd;

#define PN532_IRQ   (4)
#define PN532_SCK  (13)
#define PN532_MOSI (11)
#define PN532_SS   (9)
#define PN532_MISO (12)

Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

MPU6050 accelGyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define DEG2RAD(deg) (0.0174533 * deg)

#define MAIN_CIRCLE_INDEX 3
#define NUM_CIRCLES 12

float32 timeStep = 10.0f / 60.0f;
int32 velocityIterations = 8;
int32 positionIterations = 3;

b2Vec2 gravity(0.0f, -10.0f);
b2World world(gravity);
b2BodyDef border_def;
b2BodyDef ground_def;
b2Body *border;
b2Body *ground;
b2EdgeShape ground_edge;
b2ChainShape chain;
b2Body **the_bodies;

bool was_upside_down = false;
bool is_upside_down = false;

void initialize_box2d_objects() {
    border_def.type = b2_staticBody;
    ground_def.type = b2_kinematicBody;
    ground_def.position = b2Vec2(8.0, 6.0);
    border = world.CreateBody(&border_def);
    ground = world.CreateBody(&ground_def);

    // (invisible) borders
    b2Vec2 vs[4];
    vs[0].Set(0.0f, 0.0f);
    vs[1].Set(16.0f, 0.0f);
    vs[2].Set(16.0f, 12.0f);
    vs[3].Set(0.0f, 12.0f);
    chain.CreateLoop(vs, 4);
    border->CreateFixture(&chain, 0.0f);

    // Ground
    b2Vec2 v1(-16.0f, 0.0f);
    b2Vec2 v2(16.0f, 0.0f);
    ground_edge.Set(v1, v2);
    ground->CreateFixture(&ground_edge, 0.0f);

    // circles
    the_bodies = static_cast<b2Body **>(malloc(sizeof(b2Body) * NUM_CIRCLES));
    for (int i = 0; i < NUM_CIRCLES; ++i) {
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.allowSleep = true;
        bodyDef.awake = true;
        bodyDef.position.Set((float32) ((i + 1) % 16), 6.0f);
        the_bodies[i] = world.CreateBody(&bodyDef);
        b2CircleShape circle;
        circle.m_radius = i == MAIN_CIRCLE_INDEX ? 1.1f : 0.45f;

        b2FixtureDef fixtureDef;
        fixtureDef.shape = &circle;
        fixtureDef.density = 1.0f / (i + 1);
        fixtureDef.friction = 0.3f;
        fixtureDef.restitution = 0.75;
        the_bodies[i]->CreateFixture(&fixtureDef);
    }
}

bool rfid_tag_present = false;
void handleInterruptFalling() {
    rfid_tag_present = true;
}

void setup() {
    Serial.begin(921600);

    Wire.begin();
    SPI.begin();

    accelGyro.initialize();

    setup_renderer();

    initialize_box2d_objects();

    ground->SetAngularVelocity(0.0075);
    ground->SetTransform(b2Vec2(8, 4.0f), 0);

    pinMode(PN532_IRQ, INPUT_PULLUP);
    nfc.startPassiveTargetIDDetection(PN532_MIFARE_ISO14443A);
    attachInterrupt(digitalPinToInterrupt(PN532_IRQ), handleInterruptFalling, FALLING);
    nfc.begin();
    nfc.SAMConfig();

    if (!sd.begin(SD_SS, SD_SCK_MHZ(50))) {
        sd.initErrorHalt();
    }
}

int t = 0;
char line[100];
bool message_exists = false;
void loop() {
    uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0};
    uint8_t uidLength;

    if (t % 10 == 0 && rfid_tag_present) {
        nfc.readDetectedPassiveTargetID(uid, &uidLength);

        char filename[50];
        sprintf(filename, "0x%02X/0x%02X/0x%02X/0x%02X/0x%02X/0x%02X/0x%02X/msg.txt", uid[0], uid[1], uid[2], uid[3], uid[4], uid[5], uid[6]);

        Serial.print("opening: ");
        Serial.println(filename);

        SdFile rdfile(filename, O_RDONLY);
        if (!rdfile.isOpen()) {
            Serial.println("can't open file. Does it exist?");
        }
        rdfile.fgets(line, sizeof(line));
        message_exists = true;
        Serial.println(line);

        nfc.startPassiveTargetIDDetection(PN532_MIFARE_ISO14443A);
        rfid_tag_present = false;
    }

    accelGyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    int angle = (int) (-ay > 0 ? (90.0 * (min(-ay, 16000.0) / 16000.0))
                               : (90.0 * (max(-ay, -16000.0) / 16000.0)));

    // if upsidedown
    angle = ax < -1000 ? 180 : angle;

    bool angle_is_negative = angle < 0;
    angle = abs(angle);
    angle = (int) angle % 360;

    if ((!is_upside_down && angle > 135 && 225 > angle) || (is_upside_down && (angle > 305 || 45 > angle))) {
        is_upside_down = !is_upside_down;
    }

    /*
     * Determine goal-angle of the ground
     */
    float goal_a = angle * (is_upside_down ? -1 : 1);
    if (goal_a > 20 && goal_a < 180) {
        goal_a = DEG2RAD(20);
    } else if (goal_a > 180 && goal_a < 340) {
        goal_a = DEG2RAD(340);
    } else {
        goal_a = DEG2RAD(goal_a);
    }
    if (goal_a > 90 && goal_a < 180) {
        goal_a = DEG2RAD(180);
    }
    if (goal_a > 180 && goal_a < 270) {
        goal_a = DEG2RAD(270);
    }

    /*
     * Update the position of the ground
     */
    if (was_upside_down != is_upside_down) {
        world.SetGravity(b2Vec2(0, is_upside_down ? 10 : -10));
        if (is_upside_down) {
            ground->SetTransform(b2Vec2(8, 9.0f), 0);
            ground->SetAngularVelocity(0);
        } else {
            ground->SetTransform(b2Vec2(8, 4.0f), goal_a * (angle_is_negative ? -1 : 1));
        }
        was_upside_down = is_upside_down;
    }


    if (!is_upside_down) {
        float current_a = ground->GetAngle() * (angle_is_negative ? -1 : 1);
        float option_a = goal_a - current_a;
        float option_b = goal_a - current_a + (2 * b2_pi);
        if (abs(option_a) < abs(option_b)) {
            ground->SetAngularVelocity(option_a * (angle_is_negative ? -1 : 1));
        } else {
            ground->SetAngularVelocity(option_b * (angle_is_negative ? -1 : 1));
        }
    }

    world.Step(timeStep, velocityIterations, positionIterations);

    /*
     * Render
     */
    render_all(ground, is_upside_down, NUM_CIRCLES, MAIN_CIRCLE_INDEX, the_bodies, message_exists, line);

    print_bitmap();
    Serial.println("==========");

    message_exists = false;
}