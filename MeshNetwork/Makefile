# Makefile for Resilient Mesh Emergency Communication System

# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -Werror -g -I./include
LDFLAGS = -lm

# Raspberry Pi specific settings
# To enable Raspberry Pi support, uncomment these lines:
# CFLAGS += -DUSE_RASPBERRY_PI
# LDFLAGS += -lwiringPi

# Source files
SRC_DIR = src
SRC = $(SRC_DIR)/main.c \
      $(SRC_DIR)/hal.c \
      $(SRC_DIR)/hal_simulation.c \
      $(SRC_DIR)/message.c \
      $(SRC_DIR)/network.c \
      $(SRC_DIR)/location.c \
      $(SRC_DIR)/map.c \
      $(SRC_DIR)/utils.c \
      $(SRC_DIR)/system.c

# Raspberry Pi specific source files
ifeq ($(USE_RASPBERRY_PI),1)
    SRC += $(SRC_DIR)/hal_raspberry_pi.c
    CFLAGS += -DUSE_RASPBERRY_PI
    LDFLAGS += -lwiringPi
endif

# Object files
OBJ_DIR = obj
OBJ = $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRC))

# Output executable
BIN_DIR = bin
BIN = $(BIN_DIR)/mesh_network

# Create directories if they don't exist
DIRS = $(OBJ_DIR) $(BIN_DIR)

# Default target
all: $(DIRS) $(BIN)

# Create directories
$(DIRS):
	mkdir -p $@

# Link object files to create executable
$(BIN): $(OBJ)
	$(CC) $(OBJ) -o $@ $(LDFLAGS)

# Compile source files
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean build files
clean:
	rm -rf $(OBJ_DIR) $(BIN_DIR)

# Run the application
run: all
	$(BIN)

# Run with a specific node ID
run-node-%: all
	$(BIN) $*

# Run with a specific node ID and port (for simulation)
run-node-%-port-%: all
	$(BIN) $* $*

# Enable Raspberry Pi support
raspberry-pi:
	$(MAKE) USE_RASPBERRY_PI=1

.PHONY: all clean run raspberry-pi