/**
 * Resilient Mesh Emergency Communication System
 * Enhanced with GPS, Offline Maps, and RF Triangulation
 * For Emergency Response Operations
 * 
 * location.h - GPS and location-related functions
 */

#ifndef MESH_LOCATION_H
#define MESH_LOCATION_H

#include "common.h"

/* ---- GPS and Location Functions ---- */
void update_gps_location(system_state_t* state);
void broadcast_location(system_state_t* state);
void process_location_message(system_state_t* state, message_t* msg);
geo_location_t* get_node_location(system_state_t* state, node_id_t node_id);
void update_node_location(system_state_t* state, node_id_t node_id, geo_location_t* location);
char* format_location_string(geo_location_t* location, char* buffer, size_t buffer_size);

/* ---- RF Triangulation Functions ---- */
void estimate_node_distance(system_state_t* state, node_id_t node_id);
void triangulate_position(system_state_t* state, node_id_t node_id);
float calculate_distance_from_rssi(signal_strength_t rssi);
float calculate_distance_from_rtt(uint32_t rtt);
float haversine_distance(double lat1, double lon1, double lat2, double lon2);

#endif /* MESH_LOCATION_H */