#include "graphics.h"

#include <math.h>
#include <raylib.h>
#include "rlgl.h"

void open_window(void) {
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "2d physics");
}

void close_window(void) {
    CloseWindow();
}

void clear_screen(uint32_t color) {
    ClearBackground(GetColor(color));
}

void begin_frame(void) {
    BeginDrawing();
}

void end_frame(void) {
    EndDrawing();
}

void draw_line(int x0, int y0, int x1, int y1, uint32_t color) {
    DrawLine(x0, y0, x1, y1, GetColor(color));
}

void draw_line_thick(int x0, int y0, int x1, int y1, uint32_t color, int thickness) {
    DrawLineEx((Vector2){x0, y0}, (Vector2){x1, y1}, thickness, GetColor(color));
}

void draw_circle(int x, int y, int radius, uint32_t color) {
    Color _color = GetColor(color);
    DrawCircleLines(x, y, radius, _color);
}

void draw_fill_circle(int x, int y, int radius, uint32_t color) {
    DrawCircle(x, y, radius, GetColor(color));
}

void draw_circle_line(int x, int y, int radius, float angle, uint32_t color) {
    Color _color = GetColor(color);
    DrawCircleLines(x, y, radius, _color);
    DrawLine(x, y, x + cosf(angle) * radius, y + sinf(angle) * radius, _color);
}

void draw_rect(int x, int y, int width, int height, uint32_t color) {
    DrawRectangleLines(x, y, width, height, GetColor(color));
}

void draw_fill_rect(int x, int y, int width, int height, uint32_t color) {
    DrawRectangle(x, y, width, height, GetColor(color));
}

void draw_polygon(int x, int y, Vec2Array vertices, uint32_t color) {
    for (uint32_t i = 0; i < vertices.count; i++) {
        int curr_index = i;
        int next_index = (i + 1) % vertices.count;
        DrawLine(
            vertices.items[curr_index].x,
            vertices.items[curr_index].y,
            vertices.items[next_index].x,
            vertices.items[next_index].y,
            GetColor(color)
        );
    }
    DrawCircle(x, y, 1, GetColor(color));
}

// NOTE: only works with convex polygons
void draw_fill_polygon(int x, int y, Vec2Array vertices, uint32_t color) {
    Color tint = GetColor(color);
    rlBegin(RL_TRIANGLES);
        rlColor4ub(tint.r, tint.g, tint.b, tint.a);

        // iterate in reverse order because of backface culling
        for (int i = (int)vertices.count - 1; i >= 0; i--)
        {
            int next_index = i > 0 ? (i - 1) : ((int)vertices.count - 1);
            rlVertex2f(x, y); // center
            rlVertex2f(vertices.items[i].x, vertices.items[i].y); // cur vertex
            rlVertex2f(vertices.items[next_index].x, vertices.items[next_index].y); // next vertex
        }
    rlEnd();
}

void draw_texture(int x, int y, int width, int height, float rotation, Texture2D* texture) {
    float rotation_deg = rotation * 57.2958;
    Rectangle source_rect = {0, 0, texture->width, texture->height};
    Rectangle dest_rect = {x, y, width, height}; // centered around the origin
    Vector2 origin = {width / 2.0f, height / 2.0f}; // rotation origin is relative to dest rect
    DrawTexturePro(*texture, source_rect, dest_rect, origin, rotation_deg, WHITE);
}

