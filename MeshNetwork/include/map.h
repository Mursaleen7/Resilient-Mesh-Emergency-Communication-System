/**
 * Resilient Mesh Emergency Communication System
 * Enhanced with GPS, Offline Maps, and RF Triangulation
 * For Emergency Response Operations
 * 
 * map.h - Map system functions
 */

#ifndef MESH_MAP_H
#define MESH_MAP_H

#include "common.h"

/* ---- Map System Functions ---- */
void init_map_system(map_system_t* map_system);
bool load_map_tile(map_system_t* map_system, uint16_t tile_id, uint16_t zoom_level);
void request_map_tile(system_state_t* state, uint16_t tile_id, uint16_t zoom_level);
void process_map_request(system_state_t* state, message_t* msg);
void process_map_data(system_state_t* state, message_t* msg);

/* ---- POI Functions ---- */
void add_point_of_interest(system_state_t* state, poi_t* poi);
void broadcast_point_of_interest(system_state_t* state, poi_t* poi);
void process_poi_message(system_state_t* state, message_t* msg);
char* get_poi_type_string(uint8_t type);

/* ---- Zone Functions ---- */
void add_rescue_zone(system_state_t* state, rescue_zone_t* zone);
void broadcast_rescue_zone(system_state_t* state, rescue_zone_t* zone);
void process_zone_message(system_state_t* state, message_t* msg);
bool is_in_rescue_zone(geo_location_t* location, rescue_zone_t* zone);
char* get_zone_status_string(uint8_t status);

#endif /* MESH_MAP_H */