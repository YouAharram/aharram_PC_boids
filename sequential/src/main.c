#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>  // Necessario per strcmp
#include <math.h>    // Necessario per pow e sqrt
#include <omp.h>     // Necessario per omp_get_wtime()
#include "raylib.h"
#include "boids.h"

// Funzione di utilità per inizializzare i boid con valori casuali
void init_boids(Boid* flock, int n) {
    for (int i = 0; i < n; i++) {
        // Posizione casuale all'interno dello schermo
        flock[i].x = (float)(GetRandomValue(100, SCREEN_WIDTH - 100));
        flock[i].y = (float)(GetRandomValue(100, SCREEN_HEIGHT - 100));
        
        // Velocità casuale tra -2.0 e 2.0
        flock[i].vx = (float)GetRandomValue(-200, 200) / 100.0f;
        flock[i].vy = (float)GetRandomValue(-200, 200) / 100.0f;
    }
}


void run_benchmark(Boid* flock, Boid* next_flock, int n) {
    int num_runs = 5;
    int steps_per_run = 2000;

    double wall_times[num_runs];
    double cpu_times[num_runs];

    double total_wall = 0;
    double total_cpu = 0;

    printf("\n--- Modalita Benchmark Sequenziale ---\n");
    printf("Configurazione: %d boids, %d passi per run\n", n, steps_per_run);

    // Warm-up
    for(int i = 0; i < 5; i++) {
        update_boids_sequential(flock, next_flock, n);
        Boid *temp = flock; flock = next_flock; next_flock = temp;
    }

    for (int r = 0; r < num_runs; r++) {

        double wall_start = omp_get_wtime();
        clock_t cpu_start = clock();

        for (int s = 0; s < steps_per_run; s++) {
            update_boids_sequential(flock, next_flock, n);
            Boid *temp = flock; flock = next_flock; next_flock = temp;
        }

        clock_t cpu_end = clock();
        double wall_end = omp_get_wtime();

        wall_times[r] = wall_end - wall_start;
        cpu_times[r] = (double)(cpu_end - cpu_start) / CLOCKS_PER_SEC;

        total_wall += wall_times[r];
        total_cpu += cpu_times[r];

        printf("Run %d: Wall %.6f s | CPU %.6f s\n",
               r + 1, wall_times[r], cpu_times[r]);
    }

    // Statistiche WALL
    double wall_mean = total_wall / num_runs;
    double wall_min = wall_times[0], wall_max = wall_times[0], wall_var = 0;

    // Statistiche CPU
    double cpu_mean = total_cpu / num_runs;
    double cpu_min = cpu_times[0], cpu_max = cpu_times[0], cpu_var = 0;

    for (int r = 0; r < num_runs; r++) {

        if (wall_times[r] < wall_min) wall_min = wall_times[r];
        if (wall_times[r] > wall_max) wall_max = wall_times[r];
        wall_var += pow(wall_times[r] - wall_mean, 2);

        if (cpu_times[r] < cpu_min) cpu_min = cpu_times[r];
        if (cpu_times[r] > cpu_max) cpu_max = cpu_times[r];
        cpu_var += pow(cpu_times[r] - cpu_mean, 2);
    }

    double wall_std = sqrt(wall_var / num_runs);
    double cpu_std = sqrt(cpu_var / num_runs);

    printf("\n--- Risultati Statistiche ---\n");

    printf("\nWall-clock time:\n");
    printf("Mean: %.6f s\n", wall_mean);
    printf("Min:  %.6f s | Max: %.6f s\n", wall_min, wall_max);
    printf("StdDev: %.6f s\n", wall_std);

    printf("\nCPU time:\n");
    printf("Mean: %.6f s\n", cpu_mean);
    printf("Min:  %.6f s | Max: %.6f s\n", cpu_min, cpu_max);
    printf("StdDev: %.6f s\n", cpu_std);

    printf("------------------------------\n");
}



int main(int argc, char** argv) {
    int num_boids = 1000; // Valore di default
    int benchmark_mode = 0;

    // Controllo argomenti: ./bin/boids_seq --bench [N]
    if (argc > 1 && strcmp(argv[1], "--bench") == 0) {
        benchmark_mode = 1;
        if (argc > 2) num_boids = atoi(argv[2]);
    }

    // Allocazione memoria (Double Buffering)
    Boid *flock = malloc(sizeof(Boid) * num_boids);
    Boid *next_flock = malloc(sizeof(Boid) * num_boids);
    
    if (flock == NULL || next_flock == NULL) {
        fprintf(stderr, "Errore allocazione memoria\n");
        return 1;
    }

    srand(time(NULL));
    init_boids(flock, num_boids);
    // Inizializziamo anche il secondo buffer per coerenza
    memcpy(next_flock, flock, sizeof(Boid) * num_boids);

    if (benchmark_mode) {
        run_benchmark(flock, next_flock, num_boids);
    } else {
        InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Boids Simulation - GUI");
        SetTargetFPS(60);

        while (!WindowShouldClose()) {
            update_boids_sequential(flock, next_flock, num_boids);
            
            // Swap puntatori per il frame successivo
            Boid *temp = flock; flock = next_flock; next_flock = temp;

            BeginDrawing();
                ClearBackground(BLACK);
                for (int i = 0; i < num_boids; i++) {
                    DrawCircleV((Vector2){flock[i].x, flock[i].y}, 2.0f, RAYWHITE);
                }
                DrawFPS(10, 10);
                DrawText(TextFormat("Boids: %d", num_boids), 10, 40, 20, GREEN);
                DrawText("Modalita: Sequenziale", 10, 70, 20, LIGHTGRAY);
            EndDrawing();
        }
        CloseWindow();
    }

    free(flock);
    free(next_flock);
    return 0;
}
