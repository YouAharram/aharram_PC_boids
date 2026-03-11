#include "boids.h"
#include <math.h>

// Parametri fisici (calibrabili)
static const float visual_range = 40.0f;
static const float visual_range_sq = 1600.0f;    // 40 * 40
static const float protected_range_sq = 64.0f;  // 8 * 8
static const float centering_factor = 0.0005f;
static const float avoidfactor = 0.05f;
static const float matching_factor = 0.05f;
static const float maxspeed = 3.0f;
static const float minspeed = 2.0f;
static const float turnfactor = 0.2f;

void update_boids_sequential(Boid* in, Boid* out, int n) {
    for (int i = 0; i < n; i++) {
        // 1. Inizializza accumulatori
        float xpos_avg = 0, ypos_avg = 0;
        float xvel_avg = 0, yvel_avg = 0;
        int neighboring_boids = 0;
        float close_dx = 0, close_dy = 0;

        // Salviamo i valori correnti in variabili locali per velocità e leggibilità
        float boid_x = in[i].x;
        float boid_y = in[i].y;
        float boid_vx = in[i].vx;
        float boid_vy = in[i].vy;

        for (int j = 0; j < n; j++) {
            if (i == j) continue; 

            // Leggiamo sempre dal buffer "in"
            float dx = boid_x - in[j].x;
            float dy = boid_y - in[j].y;

            // Controllo rapido nel range visivo (bounding box)
            if (fabsf(dx) < visual_range && fabsf(dy) < visual_range) {
                float squared_distance = dx*dx + dy*dy;

                // Regola 1: Separazione (Protected Range)
                if (squared_distance < protected_range_sq) {
                    close_dx += boid_x - in[j].x;
                    close_dy += boid_y - in[j].y;
                }
                // Regole 2 e 3: Allineamento e Coesione (Visual Range)
                else if (squared_distance < visual_range_sq) {
                    xpos_avg += in[j].x;
                    ypos_avg += in[j].y;
                    xvel_avg += in[j].vx;
                    yvel_avg += in[j].vy;
                    neighboring_boids++;
                }
            }
        }

        // Calcoliamo la nuova velocità partendo da quella attuale
        float next_vx = boid_vx;
        float next_vy = boid_vy;

        // Applicazione medie (Coesione e Allineamento)
        if (neighboring_boids > 0) {
            xpos_avg /= neighboring_boids;
            ypos_avg /= neighboring_boids;
            xvel_avg /= neighboring_boids;
            yvel_avg /= neighboring_boids;

            next_vx += (xpos_avg - boid_x) * centering_factor + 
                       (xvel_avg - boid_vx) * matching_factor;
            next_vy += (ypos_avg - boid_y) * centering_factor + 
                       (yvel_avg - boid_vy) * matching_factor;
        }

        // Aggiunta forza di evitamento (Separazione)
        next_vx += (close_dx * avoidfactor);
        next_vy += (close_dy * avoidfactor);

        // --- GESTIONE BORDI (Box) ---
        if (boid_y < 50)  next_vy += turnfactor;
        if (boid_y > SCREEN_HEIGHT - 50) next_vy -= turnfactor;
        if (boid_x < 50)  next_vx += turnfactor;
        if (boid_x > SCREEN_WIDTH - 50)  next_vx -= turnfactor;

        // --- LIMITAZIONE VELOCITÀ ---
        float speed = sqrtf(next_vx * next_vx + next_vy * next_vy);
        if (speed > 0) { // Evitiamo divisione per zero
            if (speed < minspeed) {
                next_vx = (next_vx / speed) * minspeed;
                next_vy = (next_vy / speed) * minspeed;
            }
            if (speed > maxspeed) {
                next_vx = (next_vx / speed) * maxspeed;
                next_vy = (next_vy / speed) * maxspeed;
            }
        }

        // --- SCRITTURA NEL BUFFER DI OUTPUT ---
        out[i].vx = next_vx;
        out[i].vy = next_vy;
        out[i].x = boid_x + next_vx;
        out[i].y = boid_y + next_vy;
    }
}
