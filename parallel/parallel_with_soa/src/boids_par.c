#include "../include/boids.h"
#include <math.h>
#include <omp.h>
#include <stddef.h>
#include <stdlib.h>
#include <limits.h>
#include <stdatomic.h>  // FIX: necessario per atomic_fetch_add

// --- RNG deterministico ---
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

    // FIX: aligned_alloc sicuro (dimensione multipla di 64)
    size_t size = sizeof(float) * n;
    size = ((size + 63) / 64) * 64;

    flock->x  = aligned_alloc(64, size);
    flock->y  = aligned_alloc(64, size);
    flock->vx = aligned_alloc(64, size);
    flock->vy = aligned_alloc(64, size);

    #pragma omp parallel for schedule(runtime)
    for (int i = 0; i < n; i++) {
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
    int n = flock->n;

    // reset griglia principale
    #pragma omp parallel for collapse(2) schedule(runtime)
    for (int r = 0; r < GRID_ROWS; r++)
        for (int c = 0; c < GRID_COLS; c++)
            grid[r][c].count = 0;

    // riempimento parallelo (atomic fetch add)
    #pragma omp parallel for schedule(runtime)
    for (int i = 0; i < n; i++) {
        int col = (int)(flock->x[i] / CELL_SIZE);
        int row = (int)(flock->y[i] / CELL_SIZE);

        if (col >= 0 && col < GRID_COLS && row >= 0 && row < GRID_ROWS) {
            Cell* cell = &grid[row][col];
            int index = atomic_fetch_add(&cell->count, 1);
            if (index < MAX_BOIDS_PER_CELL) {
                cell->boid_indices[index] = i;
            }
        }
    }
}

// --- Aggiornamento boid con griglia parallelizzato (SoA) ---
void update_boids_grid(Boids* in, Boids* out, Cell (*grid)[GRID_COLS]) {
    int n = in->n;

    #pragma omp parallel for schedule(runtime)
    for (int i = 0; i < n; i++) {
        float xpos_avg = 0.0f, ypos_avg = 0.0f;
        float xvel_avg = 0.0f, yvel_avg = 0.0f;
        int neighboring_boids = 0;
        float close_dx = 0.0f, close_dy = 0.0f;

        const float boid_x = in->x[i];
        const float boid_y = in->y[i];
        const float boid_vx = in->vx[i];
        const float boid_vy = in->vy[i];

        const int boid_col = (int)(boid_x / CELL_SIZE);
        const int boid_row = (int)(boid_y / CELL_SIZE);

        for (int dr = -1; dr <= 1; dr++) {
            int r = boid_row + dr;
            if (r < 0 || r >= GRID_ROWS) continue;

            for (int dc = -1; dc <= 1; dc++) {
                int c = boid_col + dc;
                if (c < 0 || c >= GRID_COLS) continue;

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

                    float dist_sq = dx * dx + dy * dy;
                    if (dist_sq >= VISUAL_RANGE_SQ)
                        continue;

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
            float inv = 1.0f / neighboring_boids;
            xpos_avg *= inv;
            ypos_avg *= inv;
            xvel_avg *= inv;
            yvel_avg *= inv;

            next_vx += (xpos_avg - boid_x) * CENTERING_FACTOR + (xvel_avg - boid_vx) * MATCHING_FACTOR;
            next_vy += (ypos_avg - boid_y) * CENTERING_FACTOR + (yvel_avg - boid_vy) * MATCHING_FACTOR;
        }

        next_vx += close_dx * AVOID_FACTOR;
        next_vy += close_dy * AVOID_FACTOR;

        const float border_margin = 50.0f;
        if (boid_y < border_margin) next_vy += TURN_FACTOR;
        if (boid_y > SCREEN_HEIGHT - border_margin) next_vy -= TURN_FACTOR;
        if (boid_x < border_margin) next_vx += TURN_FACTOR;
        if (boid_x > SCREEN_WIDTH - border_margin) next_vx -= TURN_FACTOR;

        float speed_sq = next_vx * next_vx + next_vy * next_vy;
        if (speed_sq > 0.0f) {
            float speed = sqrtf(speed_sq);
            if (speed < BOID_MIN_SPEED) {
                float scale = BOID_MIN_SPEED / speed;
                next_vx *= scale;
                next_vy *= scale;
            } else if (speed > BOID_MAX_SPEED) {
                float scale = BOID_MAX_SPEED / speed;
                next_vx *= scale;
                next_vy *= scale;
            }
        }

        out->vx[i] = next_vx;
        out->vy[i] = next_vy;
        out->x[i]  = boid_x + next_vx;
        out->y[i]  = boid_y + next_vy;
    }
}
