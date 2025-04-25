#ifndef TABLE_H
#define TABLE_H

#include "manifold.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

typedef struct {
    uint32_t i;
    uint32_t j;
} Pair;

typedef struct {
    Manifold value;
    Pair key;
    bool occupied;
} Bucket;

typedef struct {
    uint32_t count;
    uint32_t capacity;
    int load_factor;
    Bucket* buckets;
} Table;

void ht_init(Table* table, uint32_t capacity, uint32_t load_factor);
void ht_free(Table* table);
bool ht_remove(Table* table, Pair key);
Manifold* ht_get(Table* table, Pair key);
Manifold* ht_set(Table* table, Pair key, uint32_t num_contacts);

// debug
void ht_print(Table* table);
void ht_sort(Table* table, int (*compare)(const void* b1, const void* b2));

#endif // TABLE_H
