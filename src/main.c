#include "application.h"
#include <stdio.h>

void run() {
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

