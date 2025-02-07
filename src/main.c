#include "application.h"

static void run(void) {
    setup();

    while (running) {
        input();
        update();
        render();
    }

    destroy();
}

int main(void)
{
    run();
    return 0;
}

