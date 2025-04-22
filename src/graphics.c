#include "graphics.h"

#include <math.h>
#include <raylib.h>
#include "physics/utils.h"
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

void draw_line_pixels(int x0, int y0, int x1, int y1, uint32_t color) {
    DrawLine(x0, y0, x1, y1, GetColor(color));
}

void draw_line_thick_pixels(int x0, int y0, int x1, int y1, uint32_t color, int thickness) {
    DrawLineEx((Vector2){x0, y0}, (Vector2){x1, y1}, thickness, GetColor(color));
}

void draw_circle_pixels(int x, int y, int radius, uint32_t color) {
    Color _color = GetColor(color);
    DrawCircleLines(x, y, radius, _color);
}

void draw_fill_circle_pixels(int x, int y, int radius, uint32_t color) {
    DrawCircle(x, y, radius, GetColor(color));
}

void draw_circle_line_pixels(int x, int y, int radius, float angle, uint32_t color) {
    Color _color = GetColor(color);
    DrawCircleLines(x, y, radius, _color);
    DrawLine(x, y, x + cosf(angle) * radius, y + sinf(angle) * radius, _color);
}

void draw_rect_pixels(int x, int y, int width, int height, uint32_t color) {
    DrawRectangleLines(x, y, width, height, GetColor(color));
}

void draw_fill_rect_pixels(int x, int y, int width, int height, uint32_t color) {
    DrawRectangle(x, y, width, height, GetColor(color));
}

void draw_polygon_pixels(int x, int y, Vec2Array vertices, uint32_t color) {
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
void draw_fill_polygon_pixels(int x, int y, Vec2Array vertices, uint32_t color) {
    Color tint = GetColor(color);
    rlBegin(RL_TRIANGLES);
        rlColor4ub(tint.r, tint.g, tint.b, tint.a);

        // iterate in reverse order because of backface culling
        for (int i = (int)vertices.count - 1; i >= 0; i--) {
            int next_index = i > 0 ? (i - 1) : ((int)vertices.count - 1);
            rlVertex2f(x, y); // center
            rlVertex2f(vertices.items[i].x, vertices.items[i].y); // cur vertex
            rlVertex2f(vertices.items[next_index].x, vertices.items[next_index].y); // next vertex
        }
    rlEnd();
}

void draw_line_meters(float x0, float y0, float x1, float y1, uint32_t color) {
    DrawLine(meters_to_pixels(x0), meters_to_pixels(y0), meters_to_pixels(x1), meters_to_pixels(y1), GetColor(color));
}

void draw_line_thick_meters(float x0, float y0, float x1, float y1, uint32_t color, int thickness) {
    DrawLineEx((Vector2){meters_to_pixels(x0), meters_to_pixels(y0)}, (Vector2){meters_to_pixels(x1), meters_to_pixels(y1)}, thickness, GetColor(color));
}

void draw_circle_meters(float x, float y, float radius, uint32_t color) {
    Color _color = GetColor(color);
    DrawCircleLines(meters_to_pixels(x), meters_to_pixels(y), meters_to_pixels(radius), _color);
}

void draw_fill_circle_meters(float x, float y, float radius, uint32_t color) {
    DrawCircle(meters_to_pixels(x), meters_to_pixels(y), radius, GetColor(color));
}

void draw_circle_line_meters(float x, float y, float radius, float angle, uint32_t color) {
    Color _color = GetColor(color);
    float pixel_radius = meters_to_pixels(radius);
    float pixel_x = meters_to_pixels(x);
    float pixel_y = meters_to_pixels(y);
    DrawCircleLines(pixel_x, pixel_y, pixel_radius, _color);
    DrawLine(pixel_x, pixel_y, pixel_x + cosf(angle) * pixel_radius, pixel_y + sinf(angle) * pixel_radius, _color);
}

void draw_rect_meters(float x, float y, float width, float height, uint32_t color) {
    DrawRectangleLines(meters_to_pixels(x), meters_to_pixels(y), meters_to_pixels(width), meters_to_pixels(height), GetColor(color));
}

void draw_fill_rect_meters(float x, float y, float width, float height, uint32_t color) {
    DrawRectangle(meters_to_pixels(x), meters_to_pixels(y), meters_to_pixels(width), meters_to_pixels(height), GetColor(color));
}

void draw_polygon_meters(float x, float y, Vec2Array cur_vertices, Vec2Array prev_vertices, float alpha, uint32_t color) {
    for (uint32_t i = 0; i < cur_vertices.count; i++) {
        int curr_index = i;
        int next_index = (i + 1) % cur_vertices.count;
        DrawLine(
            meters_to_pixels(cur_vertices.items[curr_index].x * alpha + prev_vertices.items[curr_index].x * (1 - alpha)),
            meters_to_pixels(cur_vertices.items[curr_index].y * alpha + prev_vertices.items[curr_index].y * (1 - alpha)),
            meters_to_pixels(cur_vertices.items[next_index].x * alpha + prev_vertices.items[next_index].x * (1 - alpha)),
            meters_to_pixels(cur_vertices.items[next_index].y * alpha + prev_vertices.items[next_index].y * (1 - alpha)),
            GetColor(color)
        );
    }
    /*DrawCircle(meters_to_pixels(x), meters_to_pixels(y), 1, GetColor(color));*/
}

// NOTE: only works with convex polygons
void draw_fill_polygon_meters(float x, float y, Vec2Array cur_vertices, Vec2Array prev_vertices, float alpha, uint32_t color) {
    Color tint = GetColor(color);
    rlBegin(RL_TRIANGLES);
        rlColor4ub(tint.r, tint.g, tint.b, tint.a);

        // iterate in reverse order because of backface culling
        for (int i = (int)cur_vertices.count - 1; i >= 0; i--)
        {
            int next_index = i > 0 ? (i - 1) : ((int)cur_vertices.count - 1);
            rlVertex2f(meters_to_pixels(x), meters_to_pixels(y)); // center
            rlVertex2f(meters_to_pixels(cur_vertices.items[i].x * alpha + prev_vertices.items[i].x * (1 - alpha)),
                    meters_to_pixels(cur_vertices.items[i].y * alpha + prev_vertices.items[i].y * (1 - alpha))); // cur vertex
            rlVertex2f(meters_to_pixels(cur_vertices.items[next_index].x * alpha + prev_vertices.items[next_index].x * (1 - alpha)),
                    meters_to_pixels(cur_vertices.items[next_index].y * alpha + prev_vertices.items[next_index].y * (1 - alpha))); // next vertex
        }
    rlEnd();
}


void draw_texture(int x, int y, int width, int height, float rotation, Texture2D* texture) {
    float rotation_deg = rotation * 57.2958f;
    Rectangle source_rect = {0, 0, texture->width, texture->height};
    Rectangle dest_rect = {x, y, width, height}; // centered around the origin
    Vector2 origin = {width / 2.0f, height / 2.0f}; // rotation origin is relative to dest rect
    DrawTexturePro(*texture, source_rect, dest_rect, origin, rotation_deg, WHITE);
}

