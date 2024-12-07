#include "graphics.h"

#include <math.h>

void open_window() {
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "2d physics");
}

void close_window() {
    CloseWindow();
}

void clear_screen(uint32_t color) {
    ClearBackground(GetColor(color));
}

void begin_frame() {
    BeginDrawing();
}

void end_frame() {
    EndDrawing();
}

void draw_line(int x0, int y0, int x1, int y1, uint32_t color) {
    DrawLine(x0, y0, x1, y1, GetColor(color));
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

void draw_polygon(int x, int y, Vec2Array* vertices, uint32_t color) {
    for (int i = 0; i < vertices->count; i++) {
        int curr_index = i;
        int next_index = (i + 1) % vertices->count;
        DrawLine(
            vertices->items[curr_index].x,
            vertices->items[curr_index].y,
            vertices->items[next_index].x,
            vertices->items[next_index].y,
            GetColor(color)
        );
    }
    DrawCircle(x, y, 1, GetColor(color));
}


// TODO: figure out
// https://www.raylib.com/examples.html
void draw_fill_polygon(int x, int y, Vec2Array* vertices, uint32_t color) {
    /*std::vector<short> vx;*/
    /*std::vector<short> vy;*/
    /*for (int i = 0; i < vertices.size(); i++) {*/
    /*    vx.push_back(static_cast<int>(vertices[i].x));*/
    /*}*/
    /*for (int i = 0; i < vertices.size(); i++) {*/
    /*    vy.push_back(static_cast<int>(vertices[i].y));*/
    /*}*/
    /*filledPolygonColor(renderer, &vx[0], &vy[0], vertices.size(), color);*/
    /*filledCircleColor(renderer, x, y, 1, 0xFF000000);*/
}


// TODO: figure out
void draw_texture(int x, int y, int width, int height, float rotation, Texture* texture) {
    /*SDL_Rect dstRect = {x - (width / 2), y - (height / 2), width, height};*/
    /*float rotationDeg = rotation * 57.2958;*/
    /*SDL_RenderCopyEx(renderer, texture, NULL, &dstRect, rotationDeg, NULL, SDL_FLIP_NONE);*/
}

