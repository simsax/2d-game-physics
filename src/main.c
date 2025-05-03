#define _POSIX_C_SOURCE 199309L // required for clock_gettime under c99
#include <raylib.h>

#include "graphics.h"
#include "physics/body.h"
#include "physics/shape.h"
#include "physics/utils.h"
#include "physics/vec2.h"
#include "physics/world.h"
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#define SHOW_FPS 1

// palette (https://lospec.com/palette-list/grayscale-16)
#define COLOR_BACKGROUND 0x181818FF
#define COLOR_CIRCLE 0x9B9B9BFF
#define COLOR_BOX 0x9B9B9BFF

#define COLOR_GUI 0x000000FF
#define COLOR_GUI_BUTTON 0x8C8C8CFF
#define COLOR_GUI_BUTTON_HOVER 0xD1D1D1FF
#define COLOR_GUI_TEXT 0xD1D1D1FF

// globals
static bool running;
static bool paused = false;
static bool debug = true;
static bool warm_start = true;
static World world;
static Vec2 mouse_coord = {0, 0};
static bool gui_hovering = false;
// gui
static Vector2 text_demos_size;
static Vector2 text_num_size;
static int gui_width = 200;
static int font_size = 40;
static int num_demos = 9;
static int current_demo = 2;

/*static Vec2Array make_regular_polygon(int num_vertices, int radius) {*/
/*    Vec2Array vertices = DA_NULL;*/
/**/
/*    float angle_offset = 2.0f * PI / num_vertices;*/
/*    for (int i = 0; i < num_vertices; i++) {*/
/*        float angle = i * angle_offset;*/
/*        DA_APPEND(&vertices, VEC2(cos(angle) * radius, sin(angle) * radius));*/
/*    }*/
/**/
/*    return vertices;*/
/*}*/
/**/

static void create_walls(void) {
    // walls are defined in pixel units because it's easier to place them on the screen like this
    float x_center = WINDOW_WIDTH / 2.0 - gui_width / 2.0;
    float y_center = WINDOW_HEIGHT / 2.0;

    Body* floor = world_new_body(&world);
    body_init_box_pixels(floor, WINDOW_WIDTH - 50 - gui_width, 50, x_center, WINDOW_HEIGHT - 50, 0.0f);
    floor->restitution = 0.8;
    floor->friction = 0.8;

    Body* left_wall = world_new_body(&world);
    body_init_box_pixels(left_wall, 50, WINDOW_HEIGHT - 150, 50, WINDOW_HEIGHT / 2, 0.0f);
    left_wall->restitution = 0.8;
    left_wall->friction = 0.2;

    Body* right_wall = world_new_body(&world); 
    body_init_box_pixels(right_wall, 50, WINDOW_HEIGHT - 150, WINDOW_WIDTH - 50 - gui_width, WINDOW_HEIGHT / 2, 0.0f);
    right_wall->restitution = 0.8;
    right_wall->friction = 0.2;

    Body* ceiling = world_new_body(&world);
    body_init_box_pixels(ceiling, WINDOW_WIDTH - 50 - gui_width, 50, x_center, 50, 0.0f);
    ceiling->restitution = 0.8;
    ceiling->friction = 0.2;
}

static void demo_incline_plane(void) {
    create_walls();
    float x_center = WINDOW_WIDTH / 2.0;
    float y_center = WINDOW_HEIGHT / 2.0;
    Body* static_box = world_new_body(&world);
    body_init_box_pixels(static_box, 200, 800, x_center, y_center, 0.0);
    static_box->restitution = 0.8;
    static_box->friction = 0.2;
    static_box->rotation = 1.4;
}

static void demo_stack(void) {
    create_walls();
    float x_center = pixels_to_meters((WINDOW_WIDTH - gui_width) / 2.0f);
    float ground = pixels_to_meters(WINDOW_HEIGHT - 75.0f);
    float side_len = 0.5f;
    float y_center = ground - side_len / 2.0f;
    draw_fill_circle_meters(x_center, ground, 4, 0xFF0000FF);
    draw_circle_pixels((WINDOW_WIDTH - gui_width) / 2.0f, WINDOW_HEIGHT - 75.0f, 4, 0x00FF00FF);
    for (int i = 0; i < 10; i++) {
        Body* box = world_new_body(&world);
        body_init_box(box, side_len, side_len, x_center, y_center - i * side_len, 1.0);
        box->restitution = 0.0;
        box->friction = 0.2;
    }
}

static void demo_pyramid(void) {
    PIXELS_PER_METER = 20;
    create_walls();
    int len_base = 40;
    float x_center = pixels_to_meters(WINDOW_WIDTH / 2.0 - gui_width / 2.0);
    float ground = pixels_to_meters(WINDOW_HEIGHT - 75.0f);
    float side_len = 1.0f; // 1 meter
    float x_start = x_center - (len_base / 2.0f) * side_len;
    /*float y_offset = side_len * 0.25f;*/
    /*float x_offset = side_len * 1.125f;*/
    float y_offset = side_len;
    float x_offset = side_len * 1.126f;
    float y_start = ground - side_len / 2.0f;// - y_offset;
    for (int i = 0; i < len_base; i++) {
        float y = y_start - i * y_offset;
        float x_row = x_start + i * x_offset / 2.0f;
        for (int j = i; j < len_base; j++) {
            float x = x_row + (j - i) * x_offset;
            Body* box = world_new_body(&world);
            body_init_box(box, side_len, side_len, x, y, 1.0);
            box->restitution = 0.0;
            box->friction = 0.4;
        }
    }
}


static void (*demos[9])(void) = {
    demo_incline_plane,
    demo_stack,
    demo_pyramid,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
};
    

// values to check:
// - fraction of lambda impulse to re-use (probably just use all)
// - bias
// - penetration slop
// - restitution? For now set it 0

// TODO: broad phase
// TODO: collision islands
// TODO: breaking ball (constrained at the top) that destroys a pyramid could be a nice demo
// TODO: zoom in and out with mouse wheel, ability to navigate the world this should be outside the scope of the physics engine though
// TODO: simualtion with rotating motor


static void start_simulation(void (*demo)(void)) {
    PIXELS_PER_METER = 100.0f; // default value
    world_init(&world, 9.8f);
    world.warm_start = warm_start;
    demo();
}

static void setup(void) {
    open_window();
    running = true;

    text_demos_size = MeasureTextEx(GetFontDefault(), "Demos", font_size, 1);
    text_num_size = MeasureTextEx(GetFontDefault(), "8", font_size, 1);

    start_simulation(demos[current_demo]);
}

static void destroy(void) {
    world_free(&world);
    close_window();
}

static void input(void) {
    if (WindowShouldClose() || IsKeyDown(KEY_ESCAPE)) {
        running = false;
    }

    if (IsKeyPressed(KEY_D)) {
        debug = !debug;
    }
    if (IsKeyPressed(KEY_P)) {
        paused = !paused;
    }
    if (IsKeyPressed(KEY_R)) {
        paused = false;
        world_free(&world);
        start_simulation(demos[current_demo]);
    }
    if (IsKeyPressed(KEY_W)) {
        warm_start = !warm_start;
        world.warm_start = warm_start;
    }

    /*world.bodies.items[5].position = mouse_coord;*/

    if (!paused) {
        // mouse
        mouse_coord.x = GetMouseX();
        mouse_coord.y = GetMouseY();
        
        if (!gui_hovering) {
            /*if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {*/
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                //circle
                Body* new_circle = world_new_body(&world);
                body_init_circle_pixels(new_circle, 40, mouse_coord.x, mouse_coord.y, 1.0);
                new_circle->restitution = 0.5f;
                new_circle->friction = 1.0f;
                /*body_set_texture(new_circle, "./assets/basketball.png");*/
            /*} else if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {*/
            } else if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
                // box
                Body* new_box = world_new_body(&world);
                body_init_box_pixels(new_box, 60, 60, mouse_coord.x, mouse_coord.y, 10.0);
                new_box->restitution = 0.2f;
                new_box->friction = 0.8f;

                // polygon
                /*Body* new_poly = world_new_body(&world);*/
                /*Vec2Array vertices = make_regular_polygon(5, 80);*/
                /**new_poly = body_create_polygon(vertices, mouse_coord.x, mouse_coord.y, 1.0);*/
                /*new_poly->restitution = 0.2f;*/
                /*new_poly->friction = 0.2f;*/
                /*free(vertices.items);*/
            }
        }
        gui_hovering = false;
    }
}

static void render_gui(void) {
    int gui_left = WINDOW_WIDTH - gui_width;
    DrawRectangle(gui_left, 0, gui_width, WINDOW_HEIGHT, GetColor(COLOR_GUI));

    int center_x = WINDOW_WIDTH - (float)gui_width / 2;

    DrawText(TextFormat("Demos"), center_x - text_demos_size.x / 2, text_demos_size.y / 2, font_size, GetColor(COLOR_GUI_TEXT));

    int side_len = font_size * 1.6;

    if (mouse_coord.x >= gui_left) {
        gui_hovering = true;
    }

    for (int i = 0; i < num_demos; i++) {
        int center_y = 100 * (i + 1) + 20;
        int top_side = center_y - side_len / 2;
        int left_side = center_x - side_len / 2;

        bool is_hovering = (
            mouse_coord.x >= left_side            &&
            mouse_coord.x <= left_side + side_len &&
            mouse_coord.y >= top_side             &&
            mouse_coord.y <= top_side + side_len
        );

        Color color = GetColor(COLOR_GUI_BUTTON);
        if (is_hovering) {
            color = GetColor(COLOR_GUI_BUTTON_HOVER);
            if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && demos[i] != NULL) {
                current_demo = i;
                world_free(&world);
                start_simulation(demos[current_demo]);
            }
        } 

        DrawRectangle(left_side, top_side, side_len, side_len, color);
        int demo_num = i + 1;
        DrawText(TextFormat("%d", demo_num), center_x - text_num_size.x / 2, center_y - text_num_size.y / 2, font_size, GetColor(COLOR_GUI));
    }
}

static void render(float alpha) {
    if (!paused) {
        // bodies
        for (uint32_t i = 0; i < world.bodies.count; i++) {
            float body_alpha = alpha;
            Body* body = &world.bodies.items[i];
            if (body_is_static(body)) {
                body_alpha = 1;
            }
            Vec2 render_position = vec2_add(vec2_mult(body->prev_position, (1 - body_alpha)), vec2_mult(body->position, body_alpha));
            /*printf("render_pos: (%.2f, %.2f)\n", (double)render_position.x, (double)render_position.y);*/
            if (body->shape.type == CIRCLE_SHAPE) {
                /*if (!debug && body->texture.id) {*/
                /*    float diameter = body->shape.as.circle.radius * 2;*/
                /*    draw_texture(body->position.x, body->position.y, diameter, diameter,*/
                /*            body->rotation, &body->texture);*/
                /*} else {*/
                draw_circle_line_meters(render_position.x, render_position.y,
                        body->shape.as.circle.radius, body->rotation, COLOR_CIRCLE);
                /*}*/
            }  
            if (body->shape.type == BOX_SHAPE) {
                BoxShape* box_shape = &body->shape.as.box;
                /*if (!debug && body->texture.id) {*/
                /*    draw_texture(render_position.x, render_position.y, box_shape->width,*/
                /*            box_shape->height, body->rotation, &body->texture);*/
                /*} else {*/
                draw_polygon_meters(render_position.x, render_position.y, box_shape->polygon.world_vertices, box_shape->polygon.prev_world_vertices, body_alpha, COLOR_BOX);
                /*}*/
            }
            if (body->shape.type == POLYGON_SHAPE) {
                PolygonShape* polygon_shape = &body->shape.as.polygon;
                if (!debug) {
                    draw_fill_polygon_meters(render_position.x, render_position.y, polygon_shape->world_vertices, polygon_shape->prev_world_vertices, body_alpha, COLOR_BOX);
                } else {
                    draw_polygon_meters(render_position.x, render_position.y, polygon_shape->world_vertices, polygon_shape->prev_world_vertices, body_alpha, COLOR_BOX);
                }
            }
        }

        // joints
        /*if (debug) {*/
        /*    for (uint32_t i = 0; i < world.joint_constraints.count; i++) {*/
        /*        JointConstraint* constraint = &world.joint_constraints.items[i];*/
        /*        Body* a = &world.bodies.items[constraint->a_index];*/
        /*        Vec2 anchor = body_local_to_world_space(a, constraint->a_point);*/
        /*        draw_fill_circle_meters(anchor.x, anchor.y, 3, 0xFF0000FF);*/
        /*    }*/
        /*}*/

        render_gui();
    }
    end_frame();
}

static void run(void) {
    setup();
    SetTargetFPS(144);

    while (running) {
        input();

        static float lag = 0;
        // update
        {
            begin_frame();

            static float prev_time = 0;

            // new frame begins
            float cur_time = GetTime();
            float delta_time = cur_time - prev_time;
            if (delta_time > MIN_SECS_PER_FRAME) {
                // cap delta_time so that this thing is debuggable
                delta_time = MIN_SECS_PER_FRAME;
            }
            prev_time = cur_time;

            #if SHOW_FPS

            static int frame_count = 0;
            static float prev_time_fps = 0.0f;

            frame_count++;
            if (cur_time - prev_time_fps >= 1.0f) {
                float frame_ms = 1000.0f / frame_count;
                printf("FPS: %d | Num objects: %d | Num manifolds: %d\n",
                        frame_count, world.bodies.count, world.manifolds.count);
                frame_count = 0;
                prev_time_fps = cur_time;
            }
            #endif

            lag += delta_time;
            while (!paused && lag >= FIXED_DT) {
                // for debug draws
                clear_screen(COLOR_BACKGROUND);
                world_update(&world, FIXED_DT);
                lag -= FIXED_DT;
            }
        }

        float alpha = lag / FIXED_DT;
        render(alpha);
    }

    destroy();
}


int main(void)
{
    run();
    return 0;
}

