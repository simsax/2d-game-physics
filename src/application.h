#ifndef APPLICATION_H
#define APPLICATION_H

#include <stdbool.h>

void setup(void);
void destroy(void);
void input(void);
void update(void);
void render(void);

extern bool running;

#endif // APPLICATION_H
