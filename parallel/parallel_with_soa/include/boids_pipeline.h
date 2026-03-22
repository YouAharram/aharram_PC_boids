#ifndef BOIDS_PIPELINE_H
#define BOIDS_PIPELINE_H

#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include "boids.h"

typedef struct {
    int n;               // numero di boids
    bool use_grid;       // uso della griglia

    unsigned int seed;   // seed per inizializzazione deterministica

    Boids* flock;        // buffer corrente (lettura)
    Boids* next_flock;   // buffer successivo (scrittura)
    Cell (*grid)[GRID_COLS]; // griglia spaziale

    atomic_bool running;      // flag thread worker attivo
    pthread_t worker_thread;  // thread di simulazione
    pthread_mutex_t mutex;    // protezione accesso ai buffer
} Pipeline;

// --- Inizializzazione e lifecycle ---
void pipeline_init(Pipeline* p, int n, int use_grid, unsigned int seed);
void pipeline_start(Pipeline* p);
void pipeline_stop(Pipeline* p);
void pipeline_cleanup(Pipeline* p);

// --- Accesso dati per rendering ---
Boids* pipeline_get_latest(Pipeline* p);

// --- Benchmark ---
void pipeline_benchmark(Pipeline* p, int steps_per_run, int num_runs);

#endif
