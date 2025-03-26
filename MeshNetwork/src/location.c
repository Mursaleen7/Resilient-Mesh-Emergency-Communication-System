/**
 * Resilient Mesh Emergency Communication System
 * Enhanced with GPS, Offline Maps, and RF Triangulation
 * For Emergency Response Operations
 * 
 * location.c - GPS and location functions
 */

#include "../include/location.h"
#include "../include/message.h"
#include "../include/utils.h"
#include "../include/system.h" /* Add this for set_location declaration */
#include "../include/network.h"
/**
 * Calculate the Haversine distance between two geographic points (in meters)
 */
float haversine_distance(double lat1, double lon1, double lat2, double lon2) {
    // Convert latitude and longitude from degrees to radians
    lat1 *= M_PI / 180.0;
    lon1 *= M_PI / 180.0;
    lat2 *= M_PI / 180.0;
    lon2 *= M_PI / 180.0;
    
    // Haversine formula
    double dlon = lon2 - lon1;
    double dlat = lat2 - lat1;
    double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double r = 6371000; // Earth radius in meters
    
    return (float)(r * c);
}

/**
 * Format a geo location into a human-readable string
 */
char* format_location_string(geo_location_t* location, char* buffer, size_t buffer_size) {
    const char* fix_type = get_gps_fix_string(location->fix_quality);
    
    // Convert decimal degrees to degrees, minutes, seconds
    double lat_abs = fabs(location->latitude);
    int lat_deg = (int)lat_abs;
    double lat_min = (lat_abs - lat_deg) * 60.0;
    
    double lon_abs = fabs(location->longitude);
    int lon_deg = (int)lon_abs;
    double lon_min = (lon_abs - lon_deg) * 60.0;
    
    snprintf(buffer, buffer_size, 
            "%d°%.4f'%c, %d°%.4f'%c, Alt:%.1fm, Fix:%s, Sats:%d", 
            lat_deg, lat_min, (location->latitude >= 0) ? 'N' : 'S',
            lon_deg, lon_min, (location->longitude >= 0) ? 'E' : 'W',
            location->altitude, fix_type, location->satellites);
    
    return buffer;
}

/**
 * Update the local GPS position
 */
void update_gps_location(system_state_t* state) {
    geo_location_t new_location;
    
    // Try to get GPS data from hardware
    if (state->hal.get_gps_data(&new_location)) {
        // Update our local position
        set_location(state, new_location);
        
        // Store in our location history as well
        update_node_location(state, state->node_id, &new_location);
        
        if (DEBUG) {
            char loc_str[128];
            format_location_string(&new_location, loc_str, sizeof(loc_str));
            print_status_message("Updated GPS position", 1);
            printf("  %s\n", loc_str);
        }
    } else if (DEBUG) {
        print_status_message("Failed to get GPS fix", 2);
    }
}

/**
 * Broadcast our location to the network
 */
void broadcast_location(system_state_t* state) {
    uint32_t current_time = state->hal.get_time_ms();
    
    // Only broadcast at configured intervals and if we have valid data
    if (current_time - state->map_system.last_location_broadcast >= LOCATION_UPDATE_INTERVAL_MS &&
        state->location.fix_quality != GPS_FIX_NONE) {
        
        if (DEBUG) print_status_message("Broadcasting location update", 0);
        
        // Create a GPS update message
        uint8_t payload[sizeof(geo_location_t)];
        memcpy(payload, &state->location, sizeof(geo_location_t));
        
        // Send as broadcast with normal priority
        send_message(state, 0xFFFF, MSG_LOCATION, payload, sizeof(geo_location_t), PRIORITY_NORMAL);
        
        state->map_system.last_location_broadcast = current_time;
    }
}

/**
 * Process a received location update message
 */
void process_location_message(system_state_t* state, message_t* msg) {
    if (msg->payload_len >= sizeof(geo_location_t)) {
        geo_location_t location;
        memcpy(&location, msg->payload, sizeof(geo_location_t));
        
        // Update the node's location in our database
        update_node_location(state, msg->source, &location);
        
        if (DEBUG) {
            char loc_str[128];
            format_location_string(&location, loc_str, sizeof(loc_str));
            char buffer[256];
            snprintf(buffer, sizeof(buffer), "Updated location for node %d", msg->source);
            print_status_message(buffer, 0);
            printf("  %s\n", loc_str);
        }
    }
}

/**
 * Get the location of a specific node
 */
geo_location_t* get_node_location(system_state_t* state, node_id_t node_id) {
    // If it's our own ID, return our current location
    if (node_id == state->node_id) {
        return &state->location;
    }
    
    // Check our location database
    for (int i = 0; i < MAX_NODES; i++) {
        if (state->map_system.location_timestamps[i] > 0 && 
            i < state->node_count && 
            state->known_nodes[i].id == node_id) {
            return &state->map_system.last_known_locations[i];
        }
    }
    
    return NULL;
}

/**
 * Update a node's location in our database
 */
void update_node_location(system_state_t* state, node_id_t node_id, geo_location_t* location) {
    // If it's our own node, update the main location
    if (node_id == state->node_id) {
        state->location = *location;
    }
    
    // Find the node's entry or create a new one
    for (int i = 0; i < state->node_count; i++) {
        if (state->known_nodes[i].id == node_id) {
            // Update existing entry
            state->map_system.last_known_locations[i] = *location;
            state->map_system.location_timestamps[i] = state->hal.get_time_ms();
            
            // Also update the location in the node info
            state->known_nodes[i].location = *location;
            return;
        }
    }
    
    // If we reach here, the node isn't known yet - wait for it to be added first
}

/**
 * Estimate distance to a node based on signal strength and/or round trip time
 */
void estimate_node_distance(system_state_t* state, node_id_t node_id) {
    node_info_t* node = find_node(state, node_id);
    if (!node) return;
    
    // Method 1: RSSI-based distance estimation
    float rssi_distance = calculate_distance_from_rssi(node->signal_strength);
    
    // Method 2: RTT-based distance estimation (if available)
    float rtt_distance = 0;
    if (node->rtt > 0) {
        rtt_distance = calculate_distance_from_rtt(node->rtt);
    }
    
    // Combine the two estimates (weighted average)
    if (node->rtt > 0) {
        node->distance_estimate = (rssi_distance * 0.7f) + (rtt_distance * 0.3f);
    } else {
        node->distance_estimate = rssi_distance;
    }
    
    if (DEBUG) {
        char buffer[256];
        snprintf(buffer, sizeof(buffer), 
                "Estimated distance to node %d: %.2f meters (RSSI: %.2f m, RTT: %.2f m)",
                node_id, node->distance_estimate, rssi_distance, rtt_distance);
        print_status_message(buffer, 0);
    }
}

/**
 * Calculate approximate distance based on RSSI value
 * Uses a simplified log-distance path loss model
 */
float calculate_distance_from_rssi(signal_strength_t rssi) {
    // RSSI at 1 meter distance (calibration value)
    const float RSSI_1M = -40.0f;  
    
    // Path loss exponent (typically 2.0 to 4.0 depending on environment)
    const float PATH_LOSS_EXPONENT = 2.5f;
    
    // Convert RSSI to dBm (assuming RSSI is 0-255 range)
    float rssi_dbm = ((float)rssi / 255.0f) * (-30.0f) - 70.0f;
    
    // Calculate distance using log-distance path loss model
    float distance = pow(10.0f, (RSSI_1M - rssi_dbm) / (10.0f * PATH_LOSS_EXPONENT));
    
    // Constrain result to reasonable values
    if (distance < 0.1f) distance = 0.1f;
    if (distance > 1000.0f) distance = 1000.0f;
    
    return distance;
}

/**
 * Calculate approximate distance based on round-trip time
 * This assumes radio waves travel at the speed of light
 */
float calculate_distance_from_rtt(uint32_t rtt) {
    // Speed of light in air: ~299,702,547 m/s
    // Distance = (RTT / 2) * speed of light
    // RTT is in milliseconds, so convert to seconds
    
    // Factor in processing delay (estimated as 1ms)
    uint32_t adjusted_rtt = (rtt > 1) ? rtt - 1 : 0;
    
    float distance = ((float)adjusted_rtt / 2000.0f) * 299702547.0f;
    
    // Constrain result to reasonable values
    if (distance < 0.1f) distance = 0.1f;
    if (distance > 1000.0f) distance = 1000.0f;
    
    return distance;
}

/**
 * Triangulate position of a node based on our position and distances to other nodes
 * Requires at least 3 nodes with known positions and estimated distances
 */
void triangulate_position(system_state_t* state, node_id_t target_node_id) {
    node_info_t* target = find_node(state, target_node_id);
    if (!target) return;
    
    // We need our own position to be valid
    if (state->location.fix_quality == GPS_FIX_NONE) {
        if (DEBUG) print_status_message("Cannot triangulate without own position", 2);
        return;
    }
    
    // Count how many nodes we have with valid positions and distance estimates
    int valid_nodes = 0;
    node_info_t* reference_nodes[MAX_NODES];
    
    for (int i = 0; i < state->node_count; i++) {
        node_info_t* node = &state->known_nodes[i];
        
        // Skip the target node and ourselves
        if (node->id == target_node_id || node->id == state->node_id) continue;
        
        // Skip nodes without valid position
        geo_location_t* node_location = get_node_location(state, node->id);
        if (!node_location || node_location->fix_quality == GPS_FIX_NONE) continue;
        
        // Calculate distance estimate if we don't have one
        if (node->distance_estimate <= 0) {
            estimate_node_distance(state, node->id);
        }
        
        // If we have a valid distance estimate, include this node
        if (node->distance_estimate > 0) {
            reference_nodes[valid_nodes++] = node;
            
            if (valid_nodes >= 3) break; // We only need 3 reference points
        }
    }
    
    // Can't triangulate with fewer than 3 reference points
    if (valid_nodes < 3) {
        if (DEBUG) print_status_message("Not enough reference nodes to triangulate position", 2);
        return;
    }
    
    // We use a simple centroid algorithm with distance weighting
    // This is a simplified approach - for better accuracy we would use
    // multilateration or least squares optimization
    double lat_sum = 0, lon_sum = 0, weight_sum = 0;
    
    // Our own position is very important, so include it with a high weight
    geo_location_t* own_location = &state->location;
    float own_distance = target->distance_estimate;
    
    lat_sum += own_location->latitude * (1.0 / own_distance);
    lon_sum += own_location->longitude * (1.0 / own_distance);
    weight_sum += (1.0 / own_distance);
    
    // Add the reference nodes
    for (int i = 0; i < valid_nodes; i++) {
        node_info_t* ref_node = reference_nodes[i];
        geo_location_t* ref_location = get_node_location(state, ref_node->id);
        float distance = ref_node->distance_estimate;
        
        if (ref_location) {
            lat_sum += ref_location->latitude * (1.0 / distance);
            lon_sum += ref_location->longitude * (1.0 / distance);
            weight_sum += (1.0 / distance);
        }
    }
    
    // Calculate weighted average
    double estimated_lat = lat_sum / weight_sum;
    double estimated_lon = lon_sum / weight_sum;
    
    // Update the target node's location
    target->location.latitude = estimated_lat;
    target->location.longitude = estimated_lon;
    target->location.fix_quality = GPS_FIX_ESTIMATED;
    target->location.timestamp = state->hal.get_time_ms();
    
    // Also update our location database
    update_node_location(state, target_node_id, &target->location);
    
    if (DEBUG) {
        char loc_str[128];
        format_location_string(&target->location, loc_str, sizeof(loc_str));
        char buffer[256];
        snprintf(buffer, sizeof(buffer), "Triangulated position for node %d", target_node_id);
        print_status_message(buffer, 1);
        printf("  %s\n", loc_str);
    }
}