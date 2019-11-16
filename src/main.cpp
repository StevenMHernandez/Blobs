#define ARDUINO_MAIN

#include <Box2D/Box2D.h>
#include <stdio.h>
#include <math.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <MPU6050.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>
#include "Arduino.h"
#include <Adafruit_PN532.h>
#include "SdFat.h"
#include "sdios.h"

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));

void initVariant() {}

// Initialize C library
extern "C" void __libc_init_array(void);

/*
 * \brief Main entry point of Arduino application
 */
int main(void) {
    init();

    __libc_init_array();

    initVariant();

    delay(1);

#if defined(USE_TINYUSB)
    Adafruit_TinyUSB_Core_init();
#elif defined(USBCON)
    USBDevice.init();
    USBDevice.attach();
#endif

    setup();

    for (;;) {
        loop();
        yield(); // yield run usb background task

        if (serialEventRun) serialEventRun();
    }

    return 0;
}



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
b2ChainShape chain;
b2EdgeShape ground_edge;
b2Body **the_bodies;

bool was_upside_down = false;
bool is_upside_down = false;

#define SHARP_SCK  52
#define SHARP_MOSI 51
#define SHARP_SS   10

Adafruit_SharpMem display(SHARP_SCK, SHARP_MOSI, SHARP_SS, 320, 240);

/*
 * Print a simple bitmap representation to the serial monitor for initial debugging
 */
void print_bitmap() {
    int print_every_n = 8;
    for (uint16_t i = 239; i > 10; i -= print_every_n) {
        for (uint16_t j = 0; j < 310; j += print_every_n) {
            Serial.print(display.getPixel(j, i) ? "@" : "`");
        }
        Serial.print("\n");
    }
    Serial.flush();
}

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

void render_ground() {
    b2Fixture *f = ground->GetFixtureList();
    ground_edge = *((b2EdgeShape *) f->GetShape());
    ground_edge.m_vertex1 = b2Mul(ground->GetTransform(), ground_edge.m_vertex1);
    ground_edge.m_vertex2 = b2Mul(ground->GetTransform(), ground_edge.m_vertex2);

    display.drawLine((int) (ground_edge.m_vertex1.x * 20), (int) (ground_edge.m_vertex1.y * 20),
                     (int) (ground_edge.m_vertex2.x * 20), (int) (ground_edge.m_vertex2.y * 20), 1);

    int min_y_ground_vertex = is_upside_down ? max((int) (ground_edge.m_vertex1.y * 20),
                                                   (int) (ground_edge.m_vertex2.y * 20)) : min(
                                      (int) (ground_edge.m_vertex1.y * 20), (int) (ground_edge.m_vertex2.y * 20));
    if (!is_upside_down || min_y_ground_vertex < 240) {
        display.fillRect(0, is_upside_down ? min_y_ground_vertex : 0, 320,
                             is_upside_down ? 240 - min_y_ground_vertex : min_y_ground_vertex, 1);
    }
    display.fillTriangle((int) (ground_edge.m_vertex1.x * 20), (int) (ground_edge.m_vertex1.y * 20),
                         (int) (ground_edge.m_vertex2.x * 20), (int) (ground_edge.m_vertex2.y * 20),
                         0, min_y_ground_vertex, 1);
    display.fillTriangle((int) (ground_edge.m_vertex1.x * 20), (int) (ground_edge.m_vertex1.y * 20),
                         (int) (ground_edge.m_vertex2.x * 20), (int) (ground_edge.m_vertex2.y * 20),
                         320, min_y_ground_vertex, 1);
}

void render_circles() {
    for (int i = 0; i < NUM_CIRCLES; ++i) {
        b2Vec2 position = the_bodies[i]->GetPosition();
        display.fillCircle((int) (position.x * 20.0f), (int) (position.y * 20.0f),
                           (int) (i == MAIN_CIRCLE_INDEX ? 22 : 9), 1);
    }
}

void render_face() {
    b2Vec2 position = the_bodies[MAIN_CIRCLE_INDEX]->GetPosition();
    int multiplier = is_upside_down ? -1 : 1;

    // mouth
    display.drawCircle((int) (position.x * 20.0f) + (multiplier * 5),
                           (int) (position.y * 20.0f) + (multiplier * 5), 5, 0);
    display.fillRect((int) (position.x * 20.0f) + (multiplier * 5) - 5,
                         (int) (position.y * 20.0f) + (multiplier * 5) - 3, 15, 9, 1);

    display.fillCircle((int) (position.x * 20.0f) + (multiplier * 14),
                           (int) (position.y * 20.0f) + (multiplier * 15), 8, 0);
    display.drawCircle((int) (position.x * 20.0f) + (multiplier * 14),
                           (int) (position.y * 20.0f) + (multiplier * 15), 8, 1);
    display.fillCircle((int) (position.x * 20.0f) + (multiplier * -1),
                           (int) (position.y * 20.0f) + (multiplier * 17), 8, 0);
    display.drawCircle((int) (position.x * 20.0f) + (multiplier * -1),
                           (int) (position.y * 20.0f) + (multiplier * 17), 8, 1);

    // pupils
    bool is_looking_up = false;
    bool is_looking_down = false;
    bool is_looking_backwards = false;

    int left_movement = is_looking_backwards ? -2 : 2;
    int up_down_movement = 0;
    if (is_looking_up) {
        up_down_movement += 2;
    }
    if (is_looking_down) {
        up_down_movement -= 2;
    }
    display.fillCircle((int) (position.x * 20.0f) + (multiplier * 14) + left_movement,
                           (int) (position.y * 20.0f) + (multiplier * 15) + up_down_movement, 1, 1);
    display.fillCircle((int) (position.x * 20.0f) + (multiplier * -1) + left_movement,
                           (int) (position.y * 20.0f) + (multiplier * 17) + up_down_movement, 1, 1);
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

    display.begin();

    initialize_box2d_objects();

    ground->SetAngularVelocity(0.0075);

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
    display.fillScreen(0);
    render_ground();
    render_circles();
    render_face();
    if (message_exists && !is_upside_down) {
        display.setCursor(9, 9);
        display.setTextColor(1, 0);
        display.println(line);
    }
    display.refresh();

    message_exists = false;
}