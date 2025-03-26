/**
 * Resilient Mesh Emergency Communication System
 * Enhanced with GPS, Offline Maps, and RF Triangulation
 * For Emergency Response Operations
 * 
 * utils.h - Utility functions
 */

#ifndef MESH_UTILS_H
#define MESH_UTILS_H

#include "common.h"

/* ---- Formatting Helper Functions ---- */
void print_header(const char* title);
void print_separator(void);
void print_status_message(const char* message, int status_type);
void print_table_header(const char* columns[], int num_columns);
void print_table_row(const char* values[], int num_values);
void print_location_info(geo_location_t* location);
void print_welcome_banner(void);
void print_command_help(void);
char* get_priority_string(message_priority_t priority);
char* get_gps_fix_string(gps_fix_t fix);

/* Show network status */
void show_network_status(system_state_t* state);

#endif /* MESH_UTILS_H */