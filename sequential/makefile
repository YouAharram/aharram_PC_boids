# Compilatore e Flag
CC = gcc
CFLAGS = -Wall -Wextra -Iinclude -fopenmp
LDFLAGS = -lraylib -lGL -lm -lpthread -ldl -lrt -lX11 -fopenmp

# Flag per l'ottimizzazione (richiesti per i test di performance)
RELEASE_FLAGS = -O3 -march=native
DEBUG_FLAGS = -g

# Directory
SRC_DIR = src
INC_DIR = include
BIN_DIR = bin

# File Sorgente e Oggetti
SRCS_SEQ = $(SRC_DIR)/main.c $(SRC_DIR)/boids_seq.c
OBJS_SEQ = $(SRCS_SEQ:.c=.o)

# Target principale (Sequenziale)
TARGET_SEQ = $(BIN_DIR)/boids_seq

# Regola di default
all: $(BIN_DIR) $(TARGET_SEQ)

# Creazione cartella bin se non esiste
$(BIN_DIR):
	mkdir -p $(BIN_DIR)

# Compilazione dell'eseguibile sequenziale
$(TARGET_SEQ): $(SRCS_SEQ)
	$(CC) $(CFLAGS) $(RELEASE_FLAGS) $^ -o $@ $(LDFLAGS)

# Target per il Debug (senza ottimizzazioni, utile per gdb/valgrind)
debug: CFLAGS += $(DEBUG_FLAGS)
debug: all

# Pulizia dei file generati
clean:
	rm -rf $(BIN_DIR)/* $(SRC_DIR)/*.o

.PHONY: all clean debug
