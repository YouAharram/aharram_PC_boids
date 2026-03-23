#include "boids_pipeline.h"
#include "boids.h"
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <stdatomic.h>
#include <unistd.h>   // usleep
#include <math.h>
#include <omp.h>


// --- worker thread ---
static void* worker_func(void* arg) {
    Pipeline* p = (Pipeline*)arg;

    while (atomic_load(&p->running)) {

        if (p->use_grid) {
            build_grid(p->flock, p->grid);
            update_boids_grid(p->flock, p->next_flock, p->grid);
        } else {
            update_boids(p->flock, p->next_flock);
        }

        // swap dei buffer
        pthread_mutex_lock(&p->mutex);
        Boids* tmp = p->flock;
        p->flock = p->next_flock;
        p->next_flock = tmp;
        pthread_mutex_unlock(&p->mutex);

        usleep(16000); // evita busy wait aggressivo
    }

    return NULL;
}

// --- inizializzazione pipeline ---
void pipeline_init(Pipeline* p, int n, int use_grid, unsigned int seed) {

    p->n = n;
    p->use_grid = use_grid;
    p ->seed = seed;

    // allocazione boids
    p->flock = malloc(sizeof(Boids));
    p->next_flock = malloc(sizeof(Boids));

    p->flock->n = n;
    p->next_flock->n = n;

    p->flock->x  = aligned_alloc(64, sizeof(float) * n);
    p->flock->y  = aligned_alloc(64, sizeof(float) * n);
    p->flock->vx = aligned_alloc(64, sizeof(float) * n);
    p->flock->vy = aligned_alloc(64, sizeof(float) * n);
    
    p->next_flock->x  = aligned_alloc(64, sizeof(float) * n);
    p->next_flock->y  = aligned_alloc(64, sizeof(float) * n);
    p->next_flock->vx = aligned_alloc(64, sizeof(float) * n);
    p->next_flock->vy = aligned_alloc(64, sizeof(float) * n);

    // inizializzazione boids
    init_boids(p->flock, n, seed);

    // allocazione griglia
    p->grid = aligned_alloc(64, sizeof(Cell) * GRID_ROWS * GRID_COLS);

    atomic_store(&p->running, false);
    pthread_mutex_init(&p->mutex, NULL);
}

// --- avvio thread ---
void pipeline_start(Pipeline* p) {
    atomic_store(&p->running, true);
    pthread_create(&p->worker_thread, NULL, worker_func, p);
}

// --- stop pipeline ---
void pipeline_stop(Pipeline* p) {
    atomic_store(&p->running, false);
    pthread_join(p->worker_thread, NULL);
}

// --- cleanup pipeline ---
void pipeline_cleanup(Pipeline* p) {
    if (!p) return;

    // Free dei buffer SoA
    free(p->flock->x); free(p->flock->y);
    free(p->flock->vx); free(p->flock->vy);

    free(p->next_flock->x); free(p->next_flock->y);
    free(p->next_flock->vx); free(p->next_flock->vy);

    free(p->flock);
    free(p->next_flock);

    // Free griglia
    free(p->grid);

    pthread_mutex_destroy(&p->mutex);
}

// --- accesso dati per rendering ---
Boids* pipeline_get_latest(Pipeline* p) {
    Boids* result;
    pthread_mutex_lock(&p->mutex);
    result = p->flock;
    pthread_mutex_unlock(&p->mutex);
    return result;
}

// --- benchmark ---
void pipeline_benchmark(Pipeline* p, int steps_per_run, int num_runs) {
    double total_wall = 0.0;
    double total_cpu = 0.0;

    double min_wall = 1e9, max_wall = 0.0;
    double min_cpu  = 1e9, max_cpu  = 0.0;

    double wall_times[num_runs];

    printf("\n--- Modalita Benchmark Parallela (%s) ---\n",
           p->use_grid ? "GRID" : "BRUTE-FORCE");
    printf("Configurazione: %d boids, %d passi per run\n", p->n, steps_per_run);

    // -------------------------
    // Warm-up (non misurato)
    // -------------------------
    for (int i = 0; i < 5; i++) {
        if (p->use_grid) {
            build_grid(p->flock, p->grid);
            update_boids_grid(p->flock, p->next_flock, p->grid);
        } else {
            update_boids(p->flock, p->next_flock);
        }

        Boids* tmp = p->flock;
        p->flock = p->next_flock;
        p->next_flock = tmp;
    }

    // -------------------------
    // Benchmark
    // -------------------------
    for (int r = 0; r < num_runs; r++) {

        double wall_start = omp_get_wtime();
        clock_t cpu_start = clock();

        for (int s = 0; s < steps_per_run; s++) {
            if (p->use_grid) {
                build_grid(p->flock, p->grid);
                update_boids_grid(p->flock, p->next_flock, p->grid);
            } else {
                update_boids(p->flock, p->next_flock);
            }

            Boids* tmp = p->flock;
            p->flock = p->next_flock;
            p->next_flock = tmp;
        }

        double wall_end = omp_get_wtime();
        clock_t cpu_end = clock();

        double wall_time = wall_end - wall_start;
        double cpu_time = (double)(cpu_end - cpu_start) / CLOCKS_PER_SEC;

        printf("Run %d: Wall %.6f s | CPU %.6f s\n", r + 1, wall_time, cpu_time);

        // Salta il primo run (cold start)
        if (r > 0) {
            total_wall += wall_time;
            total_cpu  += cpu_time;

            wall_times[r - 1] = wall_time;

            if (wall_time < min_wall) min_wall = wall_time;
            if (wall_time > max_wall) max_wall = wall_time;

            if (cpu_time < min_cpu) min_cpu = cpu_time;
            if (cpu_time > max_cpu) max_cpu = cpu_time;
        }
    }

    int effective_runs = num_runs - 1;

    double mean_wall = total_wall / effective_runs;
    double mean_cpu  = total_cpu  / effective_runs;

    // -------------------------
    // Deviazione standard
    // -------------------------
    double variance = 0.0;
    for (int i = 0; i < effective_runs; i++) {
        double diff = wall_times[i] - mean_wall;
        variance += diff * diff;
    }
    variance /= effective_runs;
    double std_wall = sqrt(variance);

    // -------------------------
    // Output finale
    // -------------------------
    printf("\n--- RISULTATI ---\n");
    printf("Wall-clock Mean: %.6f s\n", mean_wall);
    printf("Min: %.6f s | Max: %.6f s | Std: %.6f s\n",
           min_wall, max_wall, std_wall);

    printf("CPU Mean: %.6f s\n", mean_cpu);
    printf("CPU Min: %.6f s | CPU Max: %.6f s\n",
           min_cpu, max_cpu);

    printf("------------------------------\n");
}
