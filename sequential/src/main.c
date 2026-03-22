#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h> 
#include <math.h>   
#include <omp.h>    
#include "raylib.h"
#include "../include/boids.h"
#include <limits.h>

// --- funzione helper per generare float casuali deterministici ---
static inline float rand_float_seq(unsigned int* state, float min, float max) {
    // LCG semplice
    *state = 1664525 * (*state) + 1013904223;
    float t = (float)(*state & 0xFFFFFF) / (float)0x1000000;
    return min + t * (max - min);
}

// --- inizializzazione boids sequenziale deterministica ---
void init_boids(Boid* flock, int n, unsigned int seed) {
    for (int i = 0; i < n; i++) {
        // seed diverso per ogni boid, ma deterministico
        unsigned int boid_seed = seed + i;

        flock[i].x  = rand_float_seq(&boid_seed, 100.0f, SCREEN_WIDTH - 100.0f);
        flock[i].y  = rand_float_seq(&boid_seed, 100.0f, SCREEN_HEIGHT - 100.0f);
        flock[i].vx = rand_float_seq(&boid_seed, -2.0f, 2.0f); // equivalente a -200..200 /100
        flock[i].vy = rand_float_seq(&boid_seed, -2.0f, 2.0f);
    }
}

// Benchmark aggiornato per supportare opzionalmente la griglia
void run_benchmark(Boid* flock, Boid* next_flock, int n, int use_grid, Cell (*grid)[GRID_COLS]) {
    int num_runs = 5;
    int steps_per_run = 100; // Ridotto a 100 per test rapidi, aumenta se necessario

    double wall_times[num_runs];
    double cpu_times[num_runs];
    double total_wall = 0;
    double total_cpu = 0;

    printf("\n--- Modalita Benchmark Sequenziale (%s) ---\n", use_grid ? "GRID" : "BRUTE-FORCE");
    printf("Configurazione: %d boids, %d passi per run\n", n, steps_per_run);

    // Warm-up
    for(int i = 0; i < 5; i++) {
        if(use_grid) {
            build_grid(flock, n, grid);
            update_boids_grid_sequential(flock, next_flock, n, grid);
        } else {
            update_boids_sequential(flock, next_flock, n);
        }
        Boid *temp = flock; flock = next_flock; next_flock = temp;
    }

    for (int r = 0; r < num_runs; r++) {
        double wall_start = omp_get_wtime();
        clock_t cpu_start = clock();

        for (int s = 0; s < steps_per_run; s++) {
            if(use_grid) {
                build_grid(flock, n, grid);
                update_boids_grid_sequential(flock, next_flock, n, grid);
            } else {
                update_boids_sequential(flock, next_flock, n);
            }
            Boid *temp = flock; flock = next_flock; next_flock = temp;
        }

        clock_t cpu_end = clock();
        double wall_end = omp_get_wtime();

        wall_times[r] = wall_end - wall_start;
        cpu_times[r] = (double)(cpu_end - cpu_start) / CLOCKS_PER_SEC;
        total_wall += wall_times[r];
        total_cpu += cpu_times[r];

        printf("Run %d: Wall %.6f s | CPU %.6f s\n", r + 1, wall_times[r], cpu_times[r]);
    }

    // Statistiche (omesse per brevità, rimangono uguali al tuo codice)
    double wall_mean = total_wall / num_runs;
    printf("\nWall-clock Mean: %.6f s\n", wall_mean);
    printf("CPU Mean:        %.6f s\n", total_cpu / num_runs);
    printf("------------------------------\n");
}

int main(int argc, char** argv) {
    int num_boids = 1000;
    int benchmark_mode = 0;
    int use_grid = 0;
    unsigned int seed = 0;

    // Controllo argomenti
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--bench") == 0) {
            benchmark_mode = 1;
            if (i + 1 < argc && argv[i+1][0] != '-') num_boids = atoi(argv[++i]);
        }
	else if (strcmp(argv[i], "--grid") == 0) { 
		use_grid = 1;
	}
	else if (strcmp(argv[i], "--seed") == 0) {
		if (i + 1 < argc) {
			seed = (unsigned int)atoi(argv[++i]);
		}
	}
	else if (argv[i][0] != '-'){
		num_boids = atoi(argv[i]);
	}
    }

    Boid *flock = malloc(sizeof(Boid) * num_boids);
    Boid *next_flock = malloc(sizeof(Boid) * num_boids);
    
    // Allocazione griglia sullo HEAP per evitare Stack Overflow
    Cell (*grid)[GRID_COLS] = malloc(sizeof(Cell) * GRID_ROWS * GRID_COLS);

    if (!flock || !next_flock || !grid) {
        fprintf(stderr, "Errore allocazione memoria\n");
        return 1;
    }

    srand(time(NULL));
    init_boids(flock, num_boids, seed);
    memcpy(next_flock, flock, sizeof(Boid) * num_boids);

    if (benchmark_mode) {
        run_benchmark(flock, next_flock, num_boids, use_grid, grid);
    } else {
        InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Boids - Sequenziale");
        SetTargetFPS(60);

        while (!WindowShouldClose()) {
            if (use_grid) {
                build_grid(flock, num_boids, grid);
                update_boids_grid_sequential(flock, next_flock, num_boids, grid);
            } else {
                update_boids_sequential(flock, next_flock, num_boids);
            }
            
            Boid *temp = flock; flock = next_flock; next_flock = temp;

            BeginDrawing();
                ClearBackground(BLACK);
                for (int i = 0; i < num_boids; i++) {
                    DrawCircleV((Vector2){flock[i].x, flock[i].y}, 2.0f, RAYWHITE);
                }
                DrawFPS(10, 10);
                DrawText(TextFormat("Boids: %d | Grid: %s", num_boids, use_grid ? "ON" : "OFF"), 10, 40, 20, GREEN);
            EndDrawing();
        }
        CloseWindow();
    }

    free(flock);
    free(next_flock);
    free(grid);
    return 0;
}
