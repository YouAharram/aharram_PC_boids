#include "../include/boids.h"
#include <math.h>
#include <omp.h>
#include <stddef.h>
#include <stdlib.h>

#include <limits.h>


static inline unsigned int lcg_rand(unsigned int* state) {
    *state = (*state * 1664525u + 1013904223u);
    return *state;
}

float rand_float(unsigned int* state, float min, float max) {
    return min + (lcg_rand(state) / (float)UINT_MAX) * (max - min);
}


// --- Inizializzazione Boids SoA --- 
void init_boids(Boids* flock, int n, unsigned int base_seed) {
    flock->n = n;

    flock->x = malloc(sizeof(float) * n);
    flock->y = malloc(sizeof(float) * n);
    flock->vx = malloc(sizeof(float) * n);
    flock->vy = malloc(sizeof(float) * n);

    #pragma omp parallel for schedule(runtime)
    for (int i = 0; i < n; i++) {
        // seed locale per ogni boid (deterministico e thread-safe)
        unsigned int local_seed = base_seed + i;

        flock->x[i] = rand_float(&local_seed, 100, SCREEN_WIDTH - 100);
        flock->y[i] = rand_float(&local_seed, 100, SCREEN_HEIGHT - 100);

        float angle = rand_float(&local_seed, 0.0f, 2.0f * 3.1415926f);
        float speed = rand_float(&local_seed, BOID_MIN_SPEED, BOID_MAX_SPEED);

        flock->vx[i] = cosf(angle) * speed;
        flock->vy[i] = sinf(angle) * speed;
    }
}

// --- Brute-force parallelizzata (SoA) ---
void update_boids(Boids* in, Boids* out) {
    int n = in->n;
    #pragma omp parallel for schedule(runtime)
    for (int i = 0; i < n; i++) {
        float xpos_avg = 0, ypos_avg = 0;
        float xvel_avg = 0, yvel_avg = 0;
        int neighboring_boids = 0;
        float close_dx = 0, close_dy = 0;

        float boid_x = in->x[i];
        float boid_y = in->y[i];
        float boid_vx = in->vx[i];
        float boid_vy = in->vy[i];

        for (int j = 0; j < n; j++) {
            if (i == j) continue;
            float dx = boid_x - in->x[j];
            float dy = boid_y - in->y[j];

            if (fabsf(dx) < VISUAL_RANGE && fabsf(dy) < VISUAL_RANGE) {
                float dist_sq = dx*dx + dy*dy;
                if (dist_sq < PROTECTED_RANGE_SQ) {
                    close_dx += dx;
                    close_dy += dy;
                } else if (dist_sq < VISUAL_RANGE_SQ) {
                    xpos_avg += in->x[j];
                    ypos_avg += in->y[j];
                    xvel_avg += in->vx[j];
                    yvel_avg += in->vy[j];
                    neighboring_boids++;
                }
            }
        }

        float next_vx = boid_vx;
        float next_vy = boid_vy;

        if (neighboring_boids > 0) {
            xpos_avg /= neighboring_boids;
            ypos_avg /= neighboring_boids;
            xvel_avg /= neighboring_boids;
            yvel_avg /= neighboring_boids;

            next_vx += (xpos_avg - boid_x) * CENTERING_FACTOR + (xvel_avg - boid_vx) * MATCHING_FACTOR;
            next_vy += (ypos_avg - boid_y) * CENTERING_FACTOR + (yvel_avg - boid_vy) * MATCHING_FACTOR;
        }

        next_vx += (close_dx * AVOID_FACTOR);
        next_vy += (close_dy * AVOID_FACTOR);

        if (boid_y < 50)  next_vy += TURN_FACTOR;
        if (boid_y > SCREEN_HEIGHT - 50) next_vy -= TURN_FACTOR;
        if (boid_x < 50)  next_vx += TURN_FACTOR;
        if (boid_x > SCREEN_WIDTH - 50) next_vx -= TURN_FACTOR;

        float speed = sqrtf(next_vx * next_vx + next_vy * next_vy);
        if (speed > 0) {
            if (speed < BOID_MIN_SPEED) {
                next_vx = (next_vx / speed) * BOID_MIN_SPEED;
                next_vy = (next_vy / speed) * BOID_MIN_SPEED;
            }
            if (speed > BOID_MAX_SPEED) {
                next_vx = (next_vx / speed) * BOID_MAX_SPEED;
                next_vy = (next_vy / speed) * BOID_MAX_SPEED;
            }
        }

        out->vx[i] = next_vx;
        out->vy[i] = next_vy;
        out->x[i]  = boid_x + next_vx;
        out->y[i]  = boid_y + next_vy;
    }
}

// --- Costruzione griglia parallelizzata (SoA) ---
void build_grid(Boids* flock, Cell (*grid)[GRID_COLS]) {
    int nthreads = omp_get_max_threads();
    static Cell* local_grids = NULL;
    static int allocated_threads = 0;

    if (local_grids == NULL || allocated_threads != nthreads) {
        free(local_grids);
        local_grids = malloc(sizeof(Cell) * nthreads * GRID_ROWS * GRID_COLS);
        allocated_threads = nthreads;
    }

    int n = flock->n;

    #pragma omp parallel
    {
        int tid = omp_get_thread_num();
        Cell* local_grid = &local_grids[tid * GRID_ROWS * GRID_COLS];

        // reset
        for (int r = 0; r < GRID_ROWS; r++)
            for (int c = 0; c < GRID_COLS; c++)
                local_grid[r * GRID_COLS + c].count = 0;

        #pragma omp for schedule(runtime)
        for (int i = 0; i < n; i++) {
            int col = (int)(flock->x[i] / CELL_SIZE);
            int row = (int)(flock->y[i] / CELL_SIZE);

            if (col >= 0 && col < GRID_COLS && row >= 0 && row < GRID_ROWS) {
                Cell* cell = &local_grid[row * GRID_COLS + col];
                int index = cell->count;
                if (index < MAX_BOIDS_PER_CELL) {
                    cell->boid_indices[index] = i;
                    cell->count++;
                }
            }
        }
    }

    // reset grid principale
    #pragma omp parallel for collapse(2) schedule(runtime)
    for (int r = 0; r < GRID_ROWS; r++)
        for (int c = 0; c < GRID_COLS; c++)
            grid[r][c].count = 0;

    // merge dei local_grids nel grid principale
    #pragma omp parallel for collapse(2) schedule(runtime)
    for (int r = 0; r < GRID_ROWS; r++) {
        for (int c = 0; c < GRID_COLS; c++) {
            int count = 0;
            for (int t = 0; t < nthreads; t++) {
                Cell* local_cell = &local_grids[t * GRID_ROWS * GRID_COLS + r * GRID_COLS + c];
                int limit = local_cell->count;
                if (limit > MAX_BOIDS_PER_CELL) limit = MAX_BOIDS_PER_CELL;
                for (int k = 0; k < limit && count < MAX_BOIDS_PER_CELL; k++) {
                    grid[r][c].boid_indices[count++] = local_cell->boid_indices[k];
                }
            }
            grid[r][c].count = count;
        }
    }
}
// --- Aggiornamento boid con griglia parallelizzato (SoA) ---
void update_boids_grid(Boids* in, Boids* out, Cell (*grid)[GRID_COLS]) {
    int n = in->n;
    #pragma omp parallel for schedule(runtime)
    for (int i = 0; i < n; i++) {
        float xpos_avg = 0, ypos_avg = 0;
        float xvel_avg = 0, yvel_avg = 0;
        int neighboring_boids = 0;
        float close_dx = 0, close_dy = 0;

        float boid_x = in->x[i];
        float boid_y = in->y[i];
        float boid_vx = in->vx[i];
        float boid_vy = in->vy[i];

        int boid_col = (int)(boid_x / CELL_SIZE);
        int boid_row = (int)(boid_y / CELL_SIZE);

        for (int dr = -1; dr <= 1; dr++) {
            for (int dc = -1; dc <= 1; dc++) {
                int r = boid_row + dr;
                int c = boid_col + dc;
                if (r < 0 || r >= GRID_ROWS || c < 0 || c >= GRID_COLS) continue;

                Cell* cell = &grid[r][c];
                int limit = cell->count;
                if (limit > MAX_BOIDS_PER_CELL) limit = MAX_BOIDS_PER_CELL;

                for (int k = 0; k < limit; k++) {
                    int j = cell->boid_indices[k];
                    if (i == j) continue;

                    float dx = boid_x - in->x[j];
                    float dy = boid_y - in->y[j];

		    if (fabsf(dx) > VISUAL_RANGE || fabsf(dy) > VISUAL_RANGE)
    			continue;
                    
		    float dist_sq = dx*dx + dy*dy;

                    if (dist_sq >= VISUAL_RANGE_SQ) continue;

                    if (dist_sq < PROTECTED_RANGE_SQ) {
                        close_dx += dx;
                        close_dy += dy;
                    } else {
                        xpos_avg += in->x[j];
                        ypos_avg += in->y[j];
                        xvel_avg += in->vx[j];
                        yvel_avg += in->vy[j];
                        neighboring_boids++;
                    }
                }
            }
        }

        float next_vx = boid_vx;
        float next_vy = boid_vy;

        if (neighboring_boids > 0) {
            xpos_avg /= neighboring_boids;
            ypos_avg /= neighboring_boids;
            xvel_avg /= neighboring_boids;
            yvel_avg /= neighboring_boids;

            next_vx += (xpos_avg - boid_x) * CENTERING_FACTOR + (xvel_avg - boid_vx) * MATCHING_FACTOR;
            next_vy += (ypos_avg - boid_y) * CENTERING_FACTOR + (yvel_avg - boid_vy) * MATCHING_FACTOR;
        }

        next_vx += (close_dx * AVOID_FACTOR);
        next_vy += (close_dy * AVOID_FACTOR);

	const float border_margin = 80.0f;  // circa 2x VISUAL_RANGE
	if (boid_y < border_margin)  next_vy += TURN_FACTOR;
	if (boid_y > SCREEN_HEIGHT - border_margin) next_vy -= TURN_FACTOR;
	if (boid_x < border_margin)  next_vx += TURN_FACTOR;
	if (boid_x > SCREEN_WIDTH - border_margin) next_vx -= TURN_FACTOR;

        float speed = sqrtf(next_vx*next_vx + next_vy*next_vy);
        if (speed > 0) {
            if (speed < BOID_MIN_SPEED) {
                next_vx = (next_vx / speed) * BOID_MIN_SPEED;
                next_vy = (next_vy / speed) * BOID_MIN_SPEED;
            }
            if (speed > BOID_MAX_SPEED) {
                next_vx = (next_vx / speed) * BOID_MAX_SPEED;
                next_vy = (next_vy / speed) * BOID_MAX_SPEED;
            }
        }

        out->vx[i] = next_vx;
        out->vy[i] = next_vy;
        out->x[i]  = boid_x + next_vx;
        out->y[i]  = boid_y + next_vy;
    }
}
