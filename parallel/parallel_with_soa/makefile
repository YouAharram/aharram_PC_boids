# Compilatore e Flag
CC = gcc
CFLAGS = -Wall -Wextra -Iinclude -fopenmp
LDFLAGS = -lraylib -lGL -lm -lpthread -ldl -lrt -lX11 -fopenmp

# Flag ottimizzazione
RELEASE_FLAGS = -O3 -march=native
DEBUG_FLAGS = -g

# Directory
SRC_DIR = src
INC_DIR = include
BIN_DIR = bin
OBJ_DIR = obj

# Header
DEPS = $(INC_DIR)/boids.h $(INC_DIR)/boids_pipeline.h

# File sorgente
SRCS = main.c boids_par.c boids_pipeline.c
OBJS = $(addprefix $(OBJ_DIR)/, $(SRCS:.c=.o))

# Target
TARGET = $(BIN_DIR)/boids

# Regola default
all: $(BIN_DIR) $(OBJ_DIR) $(TARGET)

# Creazione cartelle
$(BIN_DIR):
	mkdir -p $(BIN_DIR)

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

# Compilazione oggetti
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c $(DEPS)
	$(CC) $(CFLAGS) $(RELEASE_FLAGS) -c $< -o $@

# Link finale
$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

# Build debug
debug: RELEASE_FLAGS = $(DEBUG_FLAGS)
debug: all

# Pulizia
clean:
	rm -rf $(BIN_DIR)/* $(OBJ_DIR)/*

.PHONY: all clean debug
