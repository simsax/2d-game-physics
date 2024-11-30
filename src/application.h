#ifndef APPLICATION_H
#define APPLICATION_H

#include <stdbool.h>

void setup();
void destroy();
void input();
void update();
void render();

extern bool running;

#endif // APPLICATION_H
