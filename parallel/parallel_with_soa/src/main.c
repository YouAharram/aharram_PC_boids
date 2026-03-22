#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <omp.h>
#include "raylib.h"
#include <math.h>
#include "boids.h"
#include "boids_pipeline.h"

int main(int argc, char** argv) {

	int num_boids = 1000;
	int use_grid = 0;
	int benchmark_mode = 0;
	unsigned int seed = 0;

	// --- parsing argomenti ---
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "--bench") == 0) {
			benchmark_mode = 1;
			if (i + 1 < argc && argv[i+1][0] != '-') 
				num_boids = atoi(argv[++i]);
		}
		else if (strcmp(argv[i], "--grid") == 0) {
			use_grid = 1;
		}
		else if (strcmp(argv[i], "--seed") == 0) {
			if (i + 1 < argc) {
			    seed = (unsigned int)atoi(argv[++i]);
		    }
		}
		else if (argv[i][0] != '-') {
			num_boids = atoi(argv[i]);
		}
	}

	int max_threads = omp_get_max_threads();
	int threads_for_boids = (max_threads > 1) ? max_threads - 1 : 1;
	omp_set_num_threads(threads_for_boids);
	printf("Usati %d thread OpenMP per i calcoli dei boid\n", threads_for_boids);

	if (benchmark_mode) {
		// --- pipeline per benchmark ---
		Pipeline p;
		pipeline_init(&p, num_boids, use_grid, seed);

		int steps_per_run = 1000;
		int num_runs = 10;
		pipeline_benchmark(&p, steps_per_run, num_runs);

		// pulizia
		pipeline_cleanup(&p);
	}
	else {
		// --- pipeline per grafica ---
		Pipeline p;
		pipeline_init(&p, num_boids, use_grid, seed);
		pipeline_start(&p);  // avvia worker thread

		InitWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Boids - Pipeline SoA");
		SetTargetFPS(60);

		while (!WindowShouldClose()) {

			Boids* flock = pipeline_get_latest(&p);

			BeginDrawing();
			ClearBackground(BLACK);

			if (flock != NULL) {
				for (int i = 0; i < flock->n; i++) {
					DrawCircleV((Vector2){flock->x[i], flock->y[i]}, 2.0f, RAYWHITE);
				}
			}

			DrawFPS(10, 10);
			DrawText(
					TextFormat(
						"Boids: %d | Grid: %s | Threads: %d",
						num_boids,
						use_grid ? "ON" : "OFF",
						threads_for_boids
						),
					10, 40, 20, GREEN
				);
			EndDrawing();
		}

		CloseWindow();
		pipeline_stop(&p); // ferma worker thread e libera memoria
	}

	return 0;
}
