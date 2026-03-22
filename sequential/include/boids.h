#ifndef BOIDS_H
#define BOIDS_H

#include <omp.h> // Utile includerlo qui per avere sempre il timer disponibile
#include <limits.h>


// --- 1. COSTANTI DI SIMULAZIONE ---
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

// --- 2. STRUTTURE DATI ---

// Struttura per il singolo Boid (AoS)
typedef struct {
    float x, y;
    float vx, vy;
} Boid;

// Struttura per la cella della griglia spaziale
typedef struct {
    int count;
    int boid_indices[MAX_BOIDS_PER_CELL];
} Cell;

// --- 3. PROTOTIPI DELLE FUNZIONI ---

// Inizializzazione
void init_boids(Boid* flock, int n, unsigned int seed);

// Versione Brute Force (O(n^2))
void update_boids_sequential(Boid* in, Boid* out, int n);

// Versione con Griglia (O(n))
// Passiamo la griglia come puntatore per evitare di occupare troppo stack
void build_grid(Boid* flock, int n, Cell (*grid)[GRID_COLS]);
void update_boids_grid_sequential(Boid* in, Boid* out, int n, Cell (*grid)[GRID_COLS]);

#endif
