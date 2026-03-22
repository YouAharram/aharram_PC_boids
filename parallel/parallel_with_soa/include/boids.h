#ifndef BOIDS_H
#define BOIDS_H

//    #include <omp.h>


#include <limits.h>   // per UINT_MAX

// --- COSTANTI SIMULAZIONE ---

#define SCREEN_WIDTH 1580
#define SCREEN_HEIGHT 1000
#define CELL_SIZE 40

#define GRID_COLS (SCREEN_WIDTH / CELL_SIZE + 1)
#define GRID_ROWS (SCREEN_HEIGHT / CELL_SIZE + 1)

#define MAX_BOIDS_PER_CELL 100

#define VISUAL_RANGE 120.0f
#define VISUAL_RANGE_SQ (VISUAL_RANGE * VISUAL_RANGE)

#define PROTECTED_RANGE 15.0f
#define PROTECTED_RANGE_SQ (PROTECTED_RANGE * PROTECTED_RANGE)

#define CENTERING_FACTOR 0.0008f
#define AVOID_FACTOR 0.08f
#define MATCHING_FACTOR 0.09f

#define BOID_MAX_SPEED 4.0f
#define BOID_MIN_SPEED 2.0f

#define TURN_FACTOR 0.18f
#define BORDER_MARGIN 120.0f

// --- STRUTTURE DATI ---

typedef struct {
    float* x;
    float* y;
    float* vx;
    float* vy;
    int n;
} Boids;

typedef struct {
    int count;
    int boid_indices[MAX_BOIDS_PER_CELL];
} Cell;

// --- PROTOTIPI FUNZIONI ---

void init_boids(Boids* flock, int n, unsigned int base_seed);

// RNG deterministico
float rand_float(unsigned int* state, float min, float max);

void update_boids(Boids* in, Boids* out);

void build_grid(Boids* flock, Cell (*grid)[GRID_COLS]);
void update_boids_grid(Boids* in, Boids* out, Cell (*grid)[GRID_COLS]);

#endif
