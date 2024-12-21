#ifndef GRAPHICS_H
#define GRAPHICS_H

#include <stdint.h>
#include "raylib.h"

// ugly but what should I do
#include "physics/vec2.h"

#define WINDOW_HEIGHT 1080
#define WINDOW_WIDTH 1920

void open_window();
void close_window();
void clear_screen(uint32_t color);
void begin_frame();
void end_frame();
void draw_line(int x0, int y0, int x1, int y1, uint32_t color);
void draw_circle(int x, int y, int radius, uint32_t color);
void draw_fill_circle(int x, int y, int radius, uint32_t color);
void draw_circle_line(int x, int y, int radius, float angle, uint32_t color);
void draw_rect(int x, int y, int width, int height, uint32_t color);
void draw_fill_rect(int x, int y, int width, int height, uint32_t color);
void draw_polygon(int x, int y, Vec2Array* vertices, uint32_t color);
void draw_fill_polygon(int x, int y, Vec2Array* vertices, uint32_t color);
void draw_texture(int x, int y, int width, int height, float rotation, Texture* texture);

#endif // GRAPHICS_H

