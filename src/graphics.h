#ifndef GRAPHICS_H
#define GRAPHICS_H

#include <stdint.h>
#include "raylib.h"
#include "physics/vec2.h"

#define WINDOW_HEIGHT 1080
#define WINDOW_WIDTH 1920

void open_window(void);
void close_window(void);
void clear_screen(uint32_t color);
void begin_frame(void);
void end_frame(void);

void draw_line_pixels(int x0, int y0, int x1, int y1, uint32_t color);
void draw_line_thick_pixels(int x0, int y0, int x1, int y1, uint32_t color, int thickness);
void draw_circle_pixels(int x, int y, int radius, uint32_t color);
void draw_fill_circle_pixels(int x, int y, int radius, uint32_t color);
void draw_circle_line_pixels(int x, int y, int radius, float angle, uint32_t color);
void draw_rect_pixels(int x, int y, int width, int height, uint32_t color);
void draw_fill_rect_pixels(int x, int y, int width, int height, uint32_t color);
void draw_polygon_pixels(int x, int y, Vec2Array vertices, uint32_t color);
void draw_fill_polygon_pixels(int x, int y, Vec2Array vertices, uint32_t color);

void draw_line_meters(float x0, float y0, float x1, float y1, uint32_t color);
void draw_line_thick_meters(float x0, float y0, float x1, float y1, uint32_t color, int thickness);
void draw_circle_meters(float x, float y, float radius, uint32_t color);
void draw_fill_circle_meters(float x, float y, float radius, uint32_t color);
void draw_circle_line_meters(float x, float y, float radius, float angle, uint32_t color);
void draw_rect_meters(float x, float y, float width, float height, uint32_t color);
void draw_fill_rect_meters(float x, float y, float width, float height, uint32_t color);
void draw_polygon_meters(float x, float y, Vec2Array cur_vertices, Vec2Array prev_vertices, float alpha, uint32_t color);
void draw_fill_polygon_meters(float x, float y, Vec2Array cur_vertices, Vec2Array prev_vertices, float alpha, uint32_t color);

void draw_texture(int x, int y, int width, int height, float rotation, Texture2D* texture);

#endif // GRAPHICS_H

