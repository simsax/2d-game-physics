#include "application.h"

#include "physics/constants.h"

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

