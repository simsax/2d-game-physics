#include "table.h"
#include "manifold.h"
#include <stdint.h>
#include <stdio.h>

#define FREE(x) free(x)
#define CALLOC(capacity, elemsize) calloc((capacity), (elemsize))
#define CALC_LOAD_FACTOR(table) (int)(((float)((table)->count + 1) / (table)->capacity) * 100)

static uint32_t hash_pair(Pair key) {
    uint32_t k = key.j * key.j + key.i; // Szudzik pairing, i < j
    uint32_t hash = ((k >> 16) ^ k) * 0x45d9f3b;
    hash = ((hash >> 16) ^ hash) * 0x45d9f3b;
    hash = (hash >> 16) ^ hash;
    return hash;
}

static void ht_grow(Table* table) {
    uint32_t old_capacity = table->capacity;
    if (table->capacity == 0) {
        table->capacity = 16;
    } else {
        table->capacity *= 2;
    }
    Bucket* old_buckets = table->buckets;
    table->buckets = CALLOC(table->capacity, sizeof *table->buckets);
    if (table->buckets == NULL) {
        printf("ERROR: out of memory, aborting.\n");
        exit(1);
    }

    // create new table from scratch (copy old elements into new table)
    table->count = 0;
    for (uint32_t i = 0; i < old_capacity; i++) {
        Bucket* bucket = &old_buckets[i];
        if (bucket->occupied) {
            Manifold* manifold = ht_set(table, bucket->key, bucket->value.num_contacts);
            *manifold = bucket->value; // copy manifold content
        }
    }

    FREE(old_buckets);
}

void ht_init(Table* table, uint32_t capacity, uint32_t load_factor) {
    table->count = 0;
    table->buckets = NULL;
    table->load_factor = load_factor;

    // init table
    table->capacity = capacity;
    table->buckets = CALLOC(table->capacity, sizeof *table->buckets);
    if (table->buckets == NULL) {
        printf("ERROR: out of memory, aborting.\n");
        exit(1);
    }
}

void ht_free(Table* table) {
    FREE(table->buckets);
}

static Bucket* ht_find(Table* table, Pair key, uint32_t hash) {
    uint32_t index = hash & (table->capacity - 1); // mod of 2^n is equal to the last n bits

    Bucket* tombstone = NULL;
    for (;;) {
        Bucket* bucket = &table->buckets[index];

        if (!bucket->occupied) {
            if (bucket->value.num_contacts == 0) {
                return tombstone == NULL ? bucket : tombstone;
            } else {
                if (tombstone == NULL) {
                    tombstone = bucket;
                }
            }
        } else if (bucket->key.i == key.i && bucket->key.j == key.j) {
            return bucket;
        }

        // linear probing
        index = (index + 1) & (table->capacity - 1);
    }
}

bool ht_remove(Table* table, Pair key) {
    if (table->count == 0) 
        return false;
    uint32_t hash = hash_pair(key);
    Bucket* bucket = ht_find(table, key, hash);
    if (!bucket->occupied)
        return false;

    // place tombstone
    bucket->occupied = false;
    bucket->value.num_contacts = 1;
    return true;
}

void ht_remove_bucket(Bucket* bucket) {
    bucket->occupied = false;
    bucket->value.num_contacts = 1;
}

Manifold* ht_get(Table* table, Pair key) {
    if (table->count == 0)
        return NULL;
    uint32_t hash = hash_pair(key);
    Bucket* bucket = ht_find(table, key, hash);
    if (!bucket->occupied)
        return NULL;
    return &bucket->value;
}

Manifold* ht_set(Table* table, Pair key, uint32_t num_contacts) {
    if (CALC_LOAD_FACTOR(table) >= table->load_factor) {
        ht_grow(table);
    }

    uint32_t hash = hash_pair(key);
    Bucket* bucket = ht_find(table, key, hash);

    if (!bucket->occupied && bucket->value.num_contacts == 0) {
        // new item
        table->count++;
    }
    
    bucket->key = key;
    manifold_init(&bucket->value, num_contacts, key.i, key.j);
    bucket->occupied = true;
    return &bucket->value;
}

Manifold* ht_get_or_new(Table* table, Pair key, uint32_t num_contacts, bool* found) {
    // tries to find manifold, if not found create a new one
    if (CALC_LOAD_FACTOR(table) >= table->load_factor) {
        ht_grow(table);
    }

    uint32_t hash = hash_pair(key);
    Bucket* bucket = ht_find(table, key, hash);

    if (bucket->occupied) {
        *found = true;
        return &bucket->value;
    }

    *found = false;

    if (!bucket->occupied && bucket->value.num_contacts == 0) {
        // new item
        table->count++;
    }
    
    bucket->key = key;
    manifold_init(&bucket->value, num_contacts, key.i, key.j);
    bucket->occupied = true;
    return &bucket->value;
}


void ht_print(Table* table) {
    printf("===== TABLE =====\n");
    for (uint32_t i = 0; i < table->capacity; i++) {
        Bucket* bucket = &table->buckets[i];
        if (bucket->occupied) {
            printf("(%d, %d): (%d, %d)\n", bucket->key.i, bucket->key.j, bucket->value.a_index, bucket->value.b_index);
        } else {
            if (bucket->value.num_contacts == 0) {
                printf("NULL\n");
            } else {
                printf("[Tombstone]\n");
            }
        }
    }
    printf("=================\n");
}

