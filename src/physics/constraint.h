#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "body.h"

typedef struct {
    Body* a;
    Body* b;
} Contraint;

/*MatNM constraint_get_inv_mass();*/
void constraint_solve(Contraint* constraint);

#endif // CONSTRAINT_H
