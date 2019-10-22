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


#define MAIN_CIRCLE_INDEX 3
#define NUM_CIRCLES 12

float32 timeStep = 25.0f / 60.0f;
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

#define SHARP_SCK  13
#define SHARP_MOSI 11
#define SHARP_SS   10

Adafruit_SharpMem display_raw(SHARP_SCK, SHARP_MOSI, SHARP_SS, 320, 240); // used to compute gaussian blur
Adafruit_SharpMem display_final(SHARP_SCK, SHARP_MOSI, SHARP_SS, 320, 240); // used to render on screen

float one_d_kernel[] = {0.132429, 0.125337, 0.106259, 0.080693, 0.054891, 0.033446, 0.018255, 0.008925, 0.003908,
                        0.001533, 0.000539};

uint8_t gaussian_blur_i(int x, int y) {
    if (display_raw.getPixel(x, y)) {
        return 1;
    }

    float sum = 0;
    for (int i = 0; i < 8; ++i) {
        int lr_s[] = {-i, 0, i};
        int ud_s[] = {-i, 0, i};
        for (int lr = 0; lr <= 2; lr++) {
            int left_right = lr_s[lr];
            for (int ud = 0; ud <= 2; ud++) {
                int up_down = ud_s[ud];
                if (x + left_right > 0 && x + left_right < 320 && x + left_right > 0 && y + up_down < 240 &&
                    display_raw.getPixel(x + left_right, y + up_down)) {
                    sum += one_d_kernel[i - 1];
                }
            }
        }

        if (sum > 2.4) {
            return 1;
        }
    }

    return 0;
}

void gaussian_blur_all() {
    for (uint16_t i = 0; i < 240; i++) {
        for (uint16_t j = 0; j < 320; j++) {
            display_raw.drawPixel(i, j, gaussian_blur_i(i, j));
            display_raw.drawPixel(i, j, gaussian_blur_i(i, j));
            display_final.drawPixel(i, j, gaussian_blur_i(i, j));
        }
    }
}

/*
 * Print a simple bitmap representation to the serial monitor for initial debugging
 */
void print_bitmap() {
    int print_every_n = 4;
    for (uint16_t i = 239; i > 10; i -= print_every_n) {
        for (uint16_t j = 319; j > 10; j -= print_every_n) {
            Serial.print(display_raw.getPixel(j, i) ? "@" : "`");
        }
        Serial.print("\n");
    }
}

void initialize_box2d_objects() {
    border_def.type = b2_staticBody;
    ground_def.type = b2_kinematicBody;
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
    b2Vec2 v1(-10.0f, 4.0f);
    b2Vec2 v2(25.0f, 3.0f);
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

    int min_y_ground_vertex = min((int) (ground_edge.m_vertex1.y * 20), (int) (ground_edge.m_vertex2.y * 20));
    display_raw.fillRect(0, 0, 320, min_y_ground_vertex,
                         1);
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
    b2Vec2 position = the_bodies[3]->GetPosition();
    display_raw.fillCircle((int) (position.x * 20.0f) + 14, (int) (position.y * 20.0f) + 15, 8, 0);
    display_raw.drawCircle((int) (position.x * 20.0f) + 14, (int) (position.y * 20.0f) + 15, 8, 1);
    display_raw.fillCircle((int) (position.x * 20.0f) + -1, (int) (position.y * 20.0f) + 17, 8, 0);
    display_raw.drawCircle((int) (position.x * 20.0f) + -1, (int) (position.y * 20.0f) + 17, 8, 1);
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

    ground->SetAngularVelocity(0.075);
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


    Serial.println(t);

    if (t > 0 and t % 5 == 0) {
        ground->SetAngularVelocity(t % 10 == 0 ? 0.1 : -0.1);
//            ground->SetLinearVelocity(b2Vec2(0.0, 0.1));
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
    delay(250);

    t++;
}