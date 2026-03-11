#ifndef BOIDS_H
#define BOIDS_H

// Struttura dati (AoS)
typedef struct {
    float x, y;
    float vx, vy;
} Boid;

// Parametri di simulazione (Costanti)
#define SCREEN_WIDTH 1380
#define SCREEN_HEIGHT 820

// Funzioni per la versione sequenziale
void init_boids(Boid* flock, int n);
void update_boids_sequential(Boid* in, Boid* out, int n);

#endif
