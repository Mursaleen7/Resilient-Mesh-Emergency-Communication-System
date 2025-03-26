/**
 * Resilient Mesh Emergency Communication System
 * Enhanced with GPS, Offline Maps, and RF Triangulation
 * For Emergency Response Operations
 * 
 * network.h - Network management functions
 */

#ifndef MESH_NETWORK_H
#define MESH_NETWORK_H

#include "common.h"

/* ---- Network Management Functions ---- */
void process_incoming_messages(system_state_t* state);
void send_beacon(system_state_t* state);
void update_node_info(system_state_t* state, node_id_t id, signal_strength_t signal, 
                      battery_level_t battery, hop_count_t hops, geo_location_t* location);
node_info_t* find_node(system_state_t* state, node_id_t id);
void clean_stale_nodes(system_state_t* state);

/* ---- Routing Functions ---- */
route_entry_t* find_route(system_state_t* state, node_id_t destination);
bool add_or_update_route(system_state_t* state, node_id_t destination, 
                         node_id_t next_hop, hop_count_t hop_count, signal_strength_t quality);
void send_route_request(system_state_t* state, node_id_t destination);
void process_route_request(system_state_t* state, message_t* msg);
void process_route_response(system_state_t* state, message_t* msg);
void clean_stale_routes(system_state_t* state);

/* ---- Channel Management Functions ---- */
void scan_channels(system_state_t* state);
void switch_channel(system_state_t* state, uint8_t channel);

/* ---- Power Management Functions ---- */
void check_battery_status(system_state_t* state);
void adapt_power_settings(system_state_t* state);
void enter_low_power_mode(system_state_t* state);
void exit_low_power_mode(system_state_t* state);

#endif /* MESH_NETWORK_H */