#ifndef GRAPHICS_H
#define GRAPHICS_H

#include <stdint.h>
#include "raylib.h"

// TODO: remove
#include "physics/vec2.h"

#define WINDOW_HEIGHT 1080
#define WINDOW_WIDTH 1920

void open_window(void);
void close_window(void);
void clear_screen(uint32_t color);
void begin_frame(void);
void end_frame(void);
void draw_line(int x0, int y0, int x1, int y1, uint32_t color);
void draw_line_thick(int x0, int y0, int x1, int y1, uint32_t color, int thickness);
void draw_circle(int x, int y, int radius, uint32_t color);
void draw_fill_circle(int x, int y, int radius, uint32_t color);
void draw_circle_line(int x, int y, int radius, float angle, uint32_t color);
void draw_rect(int x, int y, int width, int height, uint32_t color);
void draw_fill_rect(int x, int y, int width, int height, uint32_t color);
void draw_polygon(int x, int y, Vec2Array vertices, uint32_t color);
void draw_fill_polygon(int x, int y, Vec2Array vertices, uint32_t color);
// TODO
/*void draw_polygon(int x, int y, Vector2 vertices[], int count, uint32_t color);*/
/*void draw_fill_polygon(int x, int y, Vector2 vertices[], int count, uint32_t color);*/
void draw_texture(int x, int y, int width, int height, float rotation, Texture2D* texture);

#endif // GRAPHICS_H

