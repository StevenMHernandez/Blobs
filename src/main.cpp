#include <stdio.h>
#include <math.h>
#include <HardwareSerial.h>
//#include <Arduino.h>
//#include <Wire.h>
//#include <SPI.h>
//#include <MFRC522.h>
//#include <MPU6050.h>
#include <Box2D/Box2D.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SharpMem.h>

//#define RST_PIN         5
//#define SS_PIN          53
//
//MFRC522 rfidReader(SS_PIN, RST_PIN);
//
//MPU6050 accelGyro;
//int16_t ax, ay, az;
//int16_t gx, gy, gz;

b2Vec2 gravity(0.0f, -10.0f);
b2World world(gravity);
b2BodyDef groundBodyDef;
b2PolygonShape groundBox;

void setup() {
    Serial.begin(9600);
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


    b2BodyDef border_def;
    b2BodyDef ground_def;
    border_def.type = b2_staticBody;
    ground_def.type = b2_kinematicBody;
    b2Body* border = world.CreateBody(&border_def);
    b2Body* ground = world.CreateBody(&ground_def);

    // borders
    b2Vec2 vs[4];
    vs[0].Set(0.0f, 0.0f);
    vs[1].Set(16.0f, 0.0f);
    vs[2].Set(16.0f, 12.0f);
    vs[3].Set(0.0f, 12.0f);
    b2ChainShape chain;
    chain.CreateLoop(vs, 4);
    border->CreateFixture(&chain, 0.0f);

    // Update the gravity
    world.SetGravity(b2Vec2(0.0f, -9.0f));

    // Ground
    b2Vec2 v1(-10.0f, 4.0f);
    b2Vec2 v2(25.0f, 3.0f);
    b2EdgeShape edge;
    edge.Set(v1, v2);
    ground->CreateFixture(&edge, 0.0f);

    int num_circles = 8;
    b2Body **the_bodies = static_cast<b2Body **>(malloc(sizeof(b2Body) * num_circles));


    for (int i = 0; i < num_circles; ++i) {
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.allowSleep = true;
        bodyDef.awake = true;
        bodyDef.position.Set((float32) i + 2, 6.0f);
        the_bodies[i] = world.CreateBody(&bodyDef);
        b2CircleShape circle;
//        circle.m_p.Set((float32) i + 2, 4.4f);
        circle.m_radius = i == 0 ? 2.5f : 0.45f;

        b2FixtureDef fixtureDef;
        fixtureDef.shape = &circle;
        fixtureDef.density = 1.0f / (i+1);
        fixtureDef.friction = 0.3f;
        the_bodies[i]->CreateFixture(&fixtureDef);
    }

    float32 timeStep = 1.0f / 60.0f;
    int32 velocityIterations = 8;
    int32 positionIterations = 3;

    for (int32 t = 0; t < 2000; ++t) {
        if (t == 75) {
//            ground->SetAngularVelocity( 0);
            ground->SetTransform(ground->GetPosition(), -1);
        }
//        if (t == 60) {
//            edge.m_vertex1.y = 0.0f;
//        }
//
//        if (t == 120) {
//            edge.m_vertex0.y = 10.0f;
//        }
        world.Step(timeStep, velocityIterations, positionIterations);

        for (int i = 0; i < num_circles; ++i) {
            b2Vec2 position = the_bodies[i]->GetPosition();
            float32 angle = the_bodies[i]->GetAngle();
            printf("%i %4.2f %4.2f %4.2f %i\n", i, position.x, position.y, angle, t);
        }
    }

}

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
}