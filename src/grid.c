#include "../include/boids.h"
#include <stdio.h>
#include <stdlib.h>



void init_grid(Grid *grid, int width, int height, float cell_size) {
    grid->width = width;
    grid->height = height;
    grid->cell_size = cell_size;
    grid->cells = (GridCell*)malloc(width * height * sizeof(GridCell));

    if (grid->cells == NULL) {
        fprintf(stderr, "Errore: Memoria insufficiente per la griglia.\n");
        exit(EXIT_FAILURE);
    }

    // Inizializzazione celle (parallela per grandi griglie)
    #pragma omp parallel for schedule(static)
    for (int i = 0; i < width * height; i++) {
        grid->cells[i].boid_count = 0;
        #ifdef STRATEGY_LOCKS
        omp_init_lock(&grid->cells[i].lock);
        #endif
    }
}

void reset_grid(Grid *grid) {
    // Test Row-Major vs Col-Major tramite GRID_IDX
    #pragma omp parallel for schedule(static)
    for (int r = 0; r < grid->height; r++) {
        for (int c = 0; c < grid->width; c++) {
            int idx = GRID_IDX(r, c, grid->width, grid->height);
            grid->cells[idx].boid_count = 0;
        }
    }
}

void fill_grid(Grid *grid, BoidSystem *sys) {
    // schedule(runtime) per testare static, dynamic, guided da terminale
    #pragma omp parallel for schedule(runtime)
    for (int i = 0; i < sys->count; i++) {
        float x = GET_X(*sys, i);
        float y = GET_Y(*sys, i);

        int col = (int)(x / grid->cell_size);
        int row = (int)(y / grid->cell_size);

        // Clamping bordi
        if (col < 0) col = 0; if (col >= grid->width) col = grid->width - 1;
        if (row < 0) row = 0; if (row >= grid->height) row = grid->height - 1;

        int idx = GRID_IDX(row, col, grid->width, grid->height);
        GridCell *cell = &grid->cells[idx];

        // --- Test Strategie di Sincronizzazione ---
        #if defined(STRATEGY_ATOMIC)
            int pos;
            #pragma omp atomic capture
            pos = cell->boid_count++;
            if (pos < MAX_BOIDS_PER_CELL) {
                cell->boid_indices[pos] = i;
            }

        #elif defined(STRATEGY_LOCKS)
            omp_set_lock(&cell->lock);
            if (cell->boid_count < MAX_BOIDS_PER_CELL) {
                cell->boid_indices[cell->boid_count++] = i;
            }
            omp_unset_lock(&cell->lock);

        #elif defined(STRATEGY_CRITICAL)
            #pragma omp critical
            {
                if (cell->boid_count < MAX_BOIDS_PER_CELL) {
                    cell->boid_indices[cell->boid_count++] = i;
                }
            }
        #endif
    }
}

int query_neighbors(Grid *grid, float x, float y, int *neighbor_indices) {
    int count = 0;
    int col = (int)(x / grid->cell_size);
    int row = (int)(y / grid->cell_size);

    // Controllo delle 9 celle limitrofe
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            int nr = row + i;
            int nc = col + j;

            if (nr >= 0 && nr < grid->height && nc >= 0 && nc < grid->width) {
                int idx = GRID_IDX(nr, nc, grid->width, grid->height);
                GridCell *cell = &grid->cells[idx];
                
                for (int k = 0; k < cell->boid_count; k++) {
                    neighbor_indices[count++] = cell->boid_indices[k];
                    if (count >= MAX_NEIGHBORS_TEMP) return count;
                }
            }
        }
    }
    return count;
}

void free_grid(Grid *grid) {
    #ifdef STRATEGY_LOCKS
    for (int i = 0; i < grid->width * grid->height; i++) {
        omp_destroy_lock(&grid->cells[i].lock);
    }
    #endif
    free(grid->cells);
}
