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
    printf("%d\n", -1 % 5);
    run();
    return 0;
}

