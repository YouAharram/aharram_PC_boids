#ifndef BOIDS_H
#define BOIDS_H

#include <omp.h>

// --- Costanti di Simulazione ---
#define MAX_BOIDS 20000
#define MAX_BOIDS_PER_CELL 128   // Capacità fissa per evitare malloc in parallelo
#define MAX_NEIGHBORS_TEMP 1024  // Buffer per query_neighbors
#define CELL_SIZE 40.0f          // Deve essere >= al raggio visivo dei boid

// --- Macro per Accesso Memoria (AoS vs SoA) ---
#ifdef USE_SOA
    typedef struct {
        float *x; float *y;
        float *vx; float *vy;
        int count;
    } BoidSystem;
    #define GET_X(s, i)  ((s).x[i])
    #define GET_Y(s, i)  ((s).y[i])
    #define GET_VX(s, i) ((s).vx[i])
    #define GET_VY(s, i) ((s).vy[i])
#else
    typedef struct {
        float x, y, vx, vy;
    } Boid;
    typedef struct {
        Boid *data;
        int count;
    } BoidSystem;
    #define GET_X(s, i)  ((s).data[i].x)
    #define GET_Y(s, i)  ((s).data[i].y)
    #define GET_VX(s, i) ((s).data[i].vx)
    #define GET_VY(s, i) ((s).data[i].vy)
#endif

// --- Macro per Accesso Griglia (Row-Major vs Col-Major) ---
// r = riga, c = colonna, W = larghezza (width), H = altezza (height)
#ifdef ACCESS_COL_MAJOR
    #define GRID_IDX(r, c, W, H) ((c) * (H) + (r))
#else
    #define GRID_IDX(r, c, W, H) ((r) * (W) + (c))
#endif

// --- Strutture Griglia ---
typedef struct {
    int boid_indices[MAX_BOIDS_PER_CELL];
    int boid_count;
    #ifdef STRATEGY_LOCKS
    omp_lock_t lock;
    #endif
} GridCell;

typedef struct {
    GridCell *cells;
    int width;
    int height;
    float cell_size;
} Grid;

// --- Prototipi Funzioni ---
void init_grid(Grid *grid, int width, int height, float cell_size);
void reset_grid(Grid *grid);
void fill_grid(Grid *grid, BoidSystem *sys);
int query_neighbors(Grid *grid, float x, float y, int *neighbor_indices);
void free_grid(Grid *grid);

#endif
