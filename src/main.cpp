#include <stdio.h>
#include <math.h>
#include <HardwareSerial.h>
//#include <Arduino.h>
//#include <Wire.h>
//#include <SPI.h>
//#include <MFRC522.h>
//#include <MPU6050.h>
#include <Box2D/Box2D.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>

//#define RST_PIN         5
//#define SS_PIN          53
//
//MFRC522 rfidReader(SS_PIN, RST_PIN);
//
//MPU6050 accelGyro;
//int16_t ax, ay, az;
//int16_t gx, gy, gz;

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

#define SHARP_SCK  13
#define SHARP_MOSI 11
#define SHARP_SS   10

Adafruit_SharpMem display_raw(SHARP_SCK, SHARP_MOSI, SHARP_SS, 320, 240); // used to compute gaussian blur
Adafruit_SharpMem display_final(SHARP_SCK, SHARP_MOSI, SHARP_SS, 320, 240); // used to render on screen

float one_d_kernel[] = {0.132429, 0.125337, 0.106259, 0.080693, 0.054891, 0.033446, 0.018255, 0.008925, 0.003908,
                        0.001533, 0.000539};

float image_sums[240][320];

float gaussian_blur_i(int x, int y, bool is_horizontal) {
    int d = is_horizontal ? x : y;
    float out = 0;

    out = 100.0f * image_sums[x][y] * one_d_kernel[0];

    for (int i = 1; i < 11; i++) {
        if (d - i > 0) {
            int add_to_x = is_horizontal ? -i : 0;
            int add_to_y = !is_horizontal ? -i : 0;
            out += 100.0f * image_sums[x + add_to_x][y + add_to_y] * one_d_kernel[i];
        }
        if (d - i < (is_horizontal ? 320 : 240)) {
            int add_to_x = is_horizontal ? i : 0;
            int add_to_y = !is_horizontal ? i : 0;
            out += 100.0f * image_sums[x + add_to_x][y + add_to_y] * one_d_kernel[i];
        }
    }

    return out;
}

void gaussian_blur_all() {
    // Setup Initial values
    for (uint16_t i = 0; i < 240; i++) {
        for (uint16_t j = 0; j < 320; j++) {
            image_sums[i][j] = 100.0f * (float) display_raw.getPixel(i, j);
        }
    }
    // Apply gaussian blur horizontally
    for (uint16_t i = 0; i < 240; i++) {
        for (uint16_t j = 0; j < 320; j++) {
            image_sums[i][j] = gaussian_blur_i(i, j, true);
        }
    }
    // Apply gaussian blur vertically and apply to display output buffer
    for (uint16_t i = 0; i < 240; i++) {
        for (uint16_t j = 0; j < 320; j++) {
            float out = gaussian_blur_i(i, j, false);
            display_final.drawPixel(i, j, out > 20);
        }
    }
}

/*
 * Print a simple bitmap representation to the serial monitor for initial debugging
 */
void print_bitmap() {
    int print_every_n = 8;
    for (uint16_t i = 239; i > 10; i -= print_every_n) {
        for (uint16_t j = 0; j < 310; j += print_every_n) {
            Serial.print(display_raw.getPixel(j, i) ? "@" : "`");
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
        bodyDef.position.Set((float32) i + 2, 6.0f);
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

    display_raw.drawLine((int) (ground_edge.m_vertex1.x * 20), (int) (ground_edge.m_vertex1.y * 20),
                         (int) (ground_edge.m_vertex2.x * 20), (int) (ground_edge.m_vertex2.y * 20), 1);

    int min_y_ground_vertex = is_upside_down ? max((int) (ground_edge.m_vertex1.y * 20), (int) (ground_edge.m_vertex2.y * 20)) : min((int) (ground_edge.m_vertex1.y * 20), (int) (ground_edge.m_vertex2.y * 20));
    if (!is_upside_down || min_y_ground_vertex < 240) {
        display_raw.fillRect(0, is_upside_down ? min_y_ground_vertex : 0, 320,is_upside_down ? 240 - min_y_ground_vertex : min_y_ground_vertex, 1);
    }
    display_raw.fillTriangle((int) (ground_edge.m_vertex1.x * 20), (int) (ground_edge.m_vertex1.y * 20),
                             (int) (ground_edge.m_vertex2.x * 20), (int) (ground_edge.m_vertex2.y * 20),
                             0, min_y_ground_vertex, 1);
    display_raw.fillTriangle((int) (ground_edge.m_vertex1.x * 20), (int) (ground_edge.m_vertex1.y * 20),
                             (int) (ground_edge.m_vertex2.x * 20), (int) (ground_edge.m_vertex2.y * 20),
                             320, min_y_ground_vertex, 1);
}

void render_circles() {
    for (int i = 0; i < NUM_CIRCLES; ++i) {
        b2Vec2 position = the_bodies[i]->GetPosition();
        display_raw.fillCircle((int) (position.x * 20.0f), (int) (position.y * 20.0f),
                               (int) (i == MAIN_CIRCLE_INDEX ? 22 : 9), 1);
    }
}

void render_face() {
    b2Vec2 position = the_bodies[MAIN_CIRCLE_INDEX]->GetPosition();
    int multiplier = is_upside_down ? -1 : 1;
    display_raw.fillCircle((int) (position.x * 20.0f) + (multiplier * 14),
                           (int) (position.y * 20.0f) + (multiplier * 15), 8, 0);
    display_raw.drawCircle((int) (position.x * 20.0f) + (multiplier * 14),
                           (int) (position.y * 20.0f) + (multiplier * 15), 8, 1);
    display_raw.fillCircle((int) (position.x * 20.0f) + (multiplier * -1),
                           (int) (position.y * 20.0f) + (multiplier * 17), 8, 0);
    display_raw.drawCircle((int) (position.x * 20.0f) + (multiplier * -1),
                           (int) (position.y * 20.0f) + (multiplier * 17), 8, 1);
}

void setup() {
    Serial.begin(921600);

//    Wire.begin();
//    SPI.begin();
//
//    accelGyro.initialize();
//
//    rfidReader.PCD_Init();
//    // Delay (potentially) required for MFRC522
//    delay(100);
//
//    rfidReader.PCD_DumpVersionToSerial();

    display_raw.begin();
    display_final.begin();

    initialize_box2d_objects();

    ground->SetAngularVelocity(0.0075);
}

int t = 0;

void loop() {
//    if (rfidReader.PICC_IsNewCardPresent() and rfidReader.PICC_ReadCardSerial()) {
//        rfidReader.PICC_DumpToSerial(&(rfidReader.uid));
//    }
//
//    accelGyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//
//    Serial.print("a/g:\t");
//    Serial.print(ax); Serial.print("\t");
//    Serial.print(ay); Serial.print("\t");
//    Serial.print(az); Serial.print("\t");
//    Serial.print(gx); Serial.print("\t");
//    Serial.print(gy); Serial.print("\t");
//    Serial.println(gz);

    int angle = t;
    bool angle_is_negative = angle < 0;
    angle = abs(angle);
    angle = angle % 360;

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

    display_raw.fillScreen(0);
    render_ground();
    render_circles();
//    gaussian_blur_all();
    render_face();

    /*
     * Final
     */
    print_bitmap();
    Serial.println("======");

    t++;
//    t--;
}