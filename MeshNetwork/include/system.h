/**
 * Resilient Mesh Emergency Communication System
 * Enhanced with GPS, Offline Maps, and RF Triangulation
 * For Emergency Response Operations
 * 
 * system.h - System initialization and management
 */

#ifndef MESH_SYSTEM_H
#define MESH_SYSTEM_H

#include "common.h"

/* ---- System Functions ---- */
void system_init(system_state_t* state, hal_t hal, node_id_t node_id);
void set_encryption_key(system_state_t* state, const uint8_t* key);
void set_location(system_state_t* state, geo_location_t location);
void process_command(system_state_t* state, char* command);

/* External declaration for global state and running flag */
extern system_state_t mesh_state;
extern bool running;

#endif /* MESH_SYSTEM_H */