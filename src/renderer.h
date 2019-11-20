#ifndef BLOBS_RENDERER_H
#define BLOBS_RENDERER_H

#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>
#include <Box2D/Box2D.h>

#define SHARP_SCK  52
#define SHARP_MOSI 51
#define SHARP_SS   10

Adafruit_SharpMem display(SHARP_SCK, SHARP_MOSI, SHARP_SS, 320, 240);

void setup_renderer() {
    display.begin();
}

/*
 * Print a simple bitmap representation to the serial monitor for initial debugging
 */
void print_bitmap() {
    int print_every_n = 4;
    for (uint16_t i = 0; i < 240; i += print_every_n) {
        for (uint16_t j = 0; j < 310; j += print_every_n) {
            Serial.print(display.getPixel(j, i) ? "@" : "`");
        }
        Serial.print("\n");
    }
    Serial.flush();
}

void render_ground(b2Body *ground, boolean is_upside_down) {
    b2Fixture *f = ground->GetFixtureList();
    b2EdgeShape ground_edge = *((b2EdgeShape *) f->GetShape());
    ground_edge.m_vertex1 = b2Mul(ground->GetTransform(), ground_edge.m_vertex1);
    ground_edge.m_vertex2 = b2Mul(ground->GetTransform(), ground_edge.m_vertex2);

    display.drawLine(320 - (int) (ground_edge.m_vertex1.x * 20), 240 - (int) (ground_edge.m_vertex1.y * 20),
                     320 - (int) (ground_edge.m_vertex2.x * 20), 240 - (int) (ground_edge.m_vertex2.y * 20), 1);

    int min_y_ground_vertex = min((int) (ground_edge.m_vertex1.y * 20),
                                  (int) (ground_edge.m_vertex2.y * 20));
    if (is_upside_down) {
        display.fillRect(0, 0, 320, 240 - min_y_ground_vertex, 1);
    }
    if (!is_upside_down && min_y_ground_vertex > 0) {
        display.fillRect(0, 240 - min_y_ground_vertex, 320, 240 - min_y_ground_vertex, 1);
    }
    if (!is_upside_down) {
        display.fillTriangle(320 - (int) (ground_edge.m_vertex1.x * 20), 240 - (int) (ground_edge.m_vertex1.y * 20),
                             320 - (int) (ground_edge.m_vertex2.x * 20), 240 - (int) (ground_edge.m_vertex2.y * 20),
                             (int) (ground_edge.m_vertex1.x * 20), 240 - min_y_ground_vertex, 1);
        display.fillTriangle(320 - (int) (ground_edge.m_vertex1.x * 20), 240 - (int) (ground_edge.m_vertex1.y * 20),
                             320 - (int) (ground_edge.m_vertex2.x * 20), 240 - (int) (ground_edge.m_vertex2.y * 20),
                             (int) (ground_edge.m_vertex2.x * 20), 240 - min_y_ground_vertex, 1);
    }
}

void render_circles(int number_of_circles, int main_circle_index, b2Body **circle_bodies) {
    for (int i = 0; i < number_of_circles; ++i) {
        b2Vec2 position = circle_bodies[i]->GetPosition();
        display.fillCircle(320 - (int) (position.x * 20.0f), 240 - (int) (position.y * 20.0f),
                           (int) (i == main_circle_index ? 22 : 9), 1);
    }
}

void render_face(int main_circle_index, b2Body **circle_bodies, boolean is_upside_down) {
    b2Vec2 position = circle_bodies[main_circle_index]->GetPosition();
    int multiplier = is_upside_down ? 1 : -1;

    // mouth
    display.drawCircle(320 - (int) (position.x * 20.0f) + (multiplier * 5),
                       240 - (int) (position.y * 20.0f) + (multiplier * 5), 5, 0);
    display.fillRect(320 - (int) (position.x * 20.0f) + (multiplier * 5) - 5,
                     240 - (int) (position.y * 20.0f) + (multiplier * 5) - (is_upside_down ? 3 : 6), 15, 9, 1);

    display.fillCircle(320 - (int) (position.x * 20.0f) + (multiplier * 14),
                       240 - (int) (position.y * 20.0f) + (multiplier * 15), 8, 0);
    display.drawCircle(320 - (int) (position.x * 20.0f) + (multiplier * 14),
                       240 - (int) (position.y * 20.0f) + (multiplier * 15), 8, 1);
    display.fillCircle(320 - (int) (position.x * 20.0f) + (multiplier * -1),
                       240 - (int) (position.y * 20.0f) + (multiplier * 17), 8, 0);
    display.drawCircle(320 - (int) (position.x * 20.0f) + (multiplier * -1),
                       240 - (int) (position.y * 20.0f) + (multiplier * 17), 8, 1);

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
    display.fillCircle(320 - (int) (position.x * 20.0f) + (multiplier * 14) + left_movement,
                       240 - (int) (position.y * 20.0f) + (multiplier * 15) + up_down_movement, 1, 1);
    display.fillCircle(320 - (int) (position.x * 20.0f) + (multiplier * -1) + left_movement,
                       240 - (int) (position.y * 20.0f) + (multiplier * 17) + up_down_movement, 1, 1);
}

void render_all(b2Body *ground, boolean is_upside_down, int number_of_circles, int main_circle_index, b2Body **circle_bodies, boolean message_exists, char *line, boolean render_to_serial_monitor) {
    display.fillScreen(0);
    render_ground(ground, is_upside_down);
    render_circles(number_of_circles, main_circle_index, circle_bodies);
    render_face(main_circle_index, circle_bodies, is_upside_down);
    if (message_exists && !is_upside_down) {
        display.setCursor(15, 20);
        display.setTextColor(1, 0);
        display.println(line);
    }
    display.refresh();

    if (render_to_serial_monitor) {
        print_bitmap();
        Serial.println("==========");
    }
}

#endif //BLOBS_RENDERER_H
