/**
 * Resilient Mesh Emergency Communication System
 * Enhanced with GPS, Offline Maps, and RF Triangulation
 * For Emergency Response Operations
 * 
 * map.c - Map system implementation
 */

#include "../include/map.h"
#include "../include/message.h"
#include "../include/location.h"
#include "../include/utils.h"

/**
 * Get zone status as string
 */
char* get_zone_status_string(uint8_t status) {
    switch (status) {
        case 0: return "Proposed";
        case 1: return "Active";
        case 2: return "Clearing";
        case 3: return "Cleared";
        default: return "Unknown";
    }
}

/**
 * Get POI type as string
 */
char* get_poi_type_string(uint8_t type) {
    switch (type) {
        case 0: return "General";
        case 1: return "Shelter";
        case 2: return "Medical";
        case 3: return "Water";
        case 4: return "Food";
        default: return "Unknown";
    }
}

/**
 * Initialize the map system
 */
void init_map_system(map_system_t* map_system) {
    memset(map_system, 0, sizeof(map_system_t));
    
    // Initialize map tiles
    for (int i = 0; i < MAX_MAP_TILES; i++) {
        map_system->map_tiles[i].data = NULL;
    }
    
    // Reset POI and zones
    map_system->poi_count = 0;
    map_system->zone_count = 0;
    
    // Reset location tracking
    for (int i = 0; i < MAX_NODES; i++) {
        map_system->location_timestamps[i] = 0;
    }
}

/**
 * Load a map tile from storage
 */
bool load_map_tile(map_system_t* map_system, uint16_t tile_id, uint16_t zoom_level) {
    char filename[64];
    snprintf(filename, sizeof(filename), "map_tile_%d_%d.osm", tile_id, zoom_level);
    
    // Check if we already have this tile
    for (int i = 0; i < map_system->map_tile_count; i++) {
        if (map_system->map_tiles[i].tile_id == tile_id && 
            map_system->map_tiles[i].zoom_level == zoom_level) {
            if (DEBUG) {
                char buffer[128];
                snprintf(buffer, sizeof(buffer), "Map tile %d (zoom %d) already loaded", 
                         tile_id, zoom_level);
                print_status_message(buffer, 0);
            }
            return true;
        }
    }
    
    // Try to open the file
    FILE* file = fopen(filename, "rb");
    if (!file) {
        if (DEBUG) {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Map tile file %s not found", filename);
            print_status_message(buffer, 2);
        }
        return false;
    }
    
    // Get file size
    fseek(file, 0, SEEK_END);
    long file_size = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    // Allocate memory
    uint8_t* data = malloc(file_size);
    if (!data) {
        if (DEBUG) print_status_message("Failed to allocate memory for map tile", 3);
        fclose(file);
        return false;
    }
    
    // Read data
    if (fread(data, 1, file_size, file) != (size_t)file_size) {
        if (DEBUG) print_status_message("Failed to read map tile data", 3);
        free(data);
        fclose(file);
        return false;
    }
    
    fclose(file);
    
    // Add to our collection if we have space
    if (map_system->map_tile_count < MAX_MAP_TILES) {
        map_tile_t* tile = &map_system->map_tiles[map_system->map_tile_count++];
        tile->tile_id = tile_id;
        tile->zoom_level = zoom_level;
        tile->data_size = file_size;
        tile->timestamp = time(NULL);
        tile->data = data;
        
        if (DEBUG) {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Loaded map tile %d (zoom %d), %ld bytes", 
                     tile_id, zoom_level, file_size);
            print_status_message(buffer, 1);
        }
        return true;
    } else {
        if (DEBUG) print_status_message("No space for new map tile", 2);
        free(data);
        return false;
    }
}

/**
 * Request a map tile from the network
 */
void request_map_tile(system_state_t* state, uint16_t tile_id, uint16_t zoom_level) {
    if (DEBUG) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "Requesting map tile %d (zoom %d) from network", 
                 tile_id, zoom_level);
        print_status_message(buffer, 0);
    }
    
    // Create payload with tile ID and zoom level
    uint8_t payload[4];
    payload[0] = tile_id & 0xFF;
    payload[1] = (tile_id >> 8) & 0xFF;
    payload[2] = zoom_level & 0xFF;
    payload[3] = (zoom_level >> 8) & 0xFF;
    
    // Broadcast request
    send_message(state, 0xFFFF, MSG_MAP_REQUEST, payload, sizeof(payload), PRIORITY_LOW);
}

/**
 * Process a map tile request
 */
void process_map_request(system_state_t* state, message_t* msg) {
    if (msg->payload_len < 4) return;
    
    // Extract tile ID and zoom level
    uint16_t tile_id = msg->payload[0] | (msg->payload[1] << 8);
    uint16_t zoom_level = msg->payload[2] | (msg->payload[3] << 8);
    
    if (DEBUG) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "Received request for map tile %d (zoom %d)", 
                 tile_id, zoom_level);
        print_status_message(buffer, 0);
    }
    
    // Check if we have this tile
    for (int i = 0; i < state->map_system.map_tile_count; i++) {
        map_tile_t* tile = &state->map_system.map_tiles[i];
        
        if (tile->tile_id == tile_id && tile->zoom_level == zoom_level && tile->data != NULL) {
            if (DEBUG) print_status_message("Sending requested map tile", 1);
            
            // We have the tile, send it back
            // We would normally fragment large tiles, but for simplicity
            // we'll assume they fit in a single message
            if (tile->data_size <= MAX_MSG_SIZE - 8) {
                uint8_t payload[MAX_MSG_SIZE];
                
                // Add header with tile ID, zoom level, and size
                payload[0] = tile_id & 0xFF;
                payload[1] = (tile_id >> 8) & 0xFF;
                payload[2] = zoom_level & 0xFF;
                payload[3] = (zoom_level >> 8) & 0xFF;
                payload[4] = tile->data_size & 0xFF;
                payload[5] = (tile->data_size >> 8) & 0xFF;
                payload[6] = (tile->data_size >> 16) & 0xFF;
                payload[7] = (tile->data_size >> 24) & 0xFF;
                
                // Copy tile data
                memcpy(payload + 8, tile->data, tile->data_size);
                
                // Send to requester
                send_message(state, msg->source, MSG_MAP_DATA, 
                           payload, tile->data_size + 8, PRIORITY_LOW);
            } else {
                // Tile too large for a single message - would require fragmentation
                if (DEBUG) {
                    char buffer[128];
                    snprintf(buffer, sizeof(buffer), "Map tile too large to send (%d bytes)", 
                             tile->data_size);
                    print_status_message(buffer, 2);
                }
            }
            return;
        }
    }
    
    // We don't have this tile
    if (DEBUG) print_status_message("We don't have the requested map tile", 2);
}

/**
 * Process received map tile data
 */
void process_map_data(system_state_t* state, message_t* msg) {
    if (msg->payload_len < 8) return;
    
    // Extract header info
    uint16_t tile_id = msg->payload[0] | (msg->payload[1] << 8);
    uint16_t zoom_level = msg->payload[2] | (msg->payload[3] << 8);
    uint32_t data_size = msg->payload[4] | (msg->payload[5] << 8) | 
                        (msg->payload[6] << 16) | (msg->payload[7] << 24);
    
    if (DEBUG) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "Received map tile %d (zoom %d), %d bytes", 
                 tile_id, zoom_level, data_size);
        print_status_message(buffer, 1);
    }
    
    // Verify data size matches what we received
    if (msg->payload_len - 8 != data_size) {
        if (DEBUG) print_status_message("Map data size mismatch", 3);
        return;
    }
    
    // Check if we already have this tile
    for (int i = 0; i < state->map_system.map_tile_count; i++) {
        if (state->map_system.map_tiles[i].tile_id == tile_id && 
            state->map_system.map_tiles[i].zoom_level == zoom_level) {
            
            // Free old data
            if (state->map_system.map_tiles[i].data) {
                free(state->map_system.map_tiles[i].data);
            }
            
            // Allocate and copy new data
            uint8_t* data = malloc(data_size);
            if (data) {
                memcpy(data, msg->payload + 8, data_size);
                state->map_system.map_tiles[i].data = data;
                state->map_system.map_tiles[i].data_size = data_size;
                state->map_system.map_tiles[i].timestamp = state->hal.get_time_ms();
                
                if (DEBUG) print_status_message("Updated existing map tile", 1);
                return;
            }
        }
    }
    
    // We don't have this tile yet, add it if we have space
    if (state->map_system.map_tile_count < MAX_MAP_TILES) {
        map_tile_t* tile = &state->map_system.map_tiles[state->map_system.map_tile_count];
        
        // Allocate and copy data
        uint8_t* data = malloc(data_size);
        if (data) {
            memcpy(data, msg->payload + 8, data_size);
            
            tile->tile_id = tile_id;
            tile->zoom_level = zoom_level;
            tile->data = data;
            tile->data_size = data_size;
            tile->timestamp = state->hal.get_time_ms();
            
            state->map_system.map_tile_count++;
            
            if (DEBUG) print_status_message("Added new map tile", 1);
            
            // Save to file for persistence
            char filename[64];
            snprintf(filename, sizeof(filename), "map_tile_%d_%d.osm", tile_id, zoom_level);
            FILE* file = fopen(filename, "wb");
            if (file) {
                fwrite(data, 1, data_size, file);
                fclose(file);
                if (DEBUG) {
                    char buffer[128];
                    snprintf(buffer, sizeof(buffer), "Saved map tile to %s", filename);
                    print_status_message(buffer, 1);
                }
            }
        }
    } else {
        if (DEBUG) print_status_message("No space for new map tile", 2);
    }
}

/**
 * Add a point of interest to our database
 */
void add_point_of_interest(system_state_t* state, poi_t* poi) {
    // Check if we already have this POI (by name and location)
    for (int i = 0; i < state->map_system.poi_count; i++) {
        poi_t* existing = &state->map_system.points_of_interest[i];
        
        // Compare name
        if (strcmp(existing->name, poi->name) == 0) {
            // Same name, check if the location is close
            float distance = haversine_distance(
                existing->location.latitude, existing->location.longitude,
                poi->location.latitude, poi->location.longitude);
            
            if (distance < 50.0f) {  // Within 50 meters
                // Update the existing POI
                memcpy(existing, poi, sizeof(poi_t));
                if (DEBUG) {
                    char buffer[128];
                    snprintf(buffer, sizeof(buffer), "Updated existing POI: %s", poi->name);
                    print_status_message(buffer, 1);
                }
                return;
            }
        }
    }
    
    // Add new POI if we have space
    if (state->map_system.poi_count < MAX_POI) {
        memcpy(&state->map_system.points_of_interest[state->map_system.poi_count], 
              poi, sizeof(poi_t));
        state->map_system.poi_count++;
        
        if (DEBUG) {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Added new POI: %s", poi->name);
            print_status_message(buffer, 1);
        }
    } else {
        if (DEBUG) print_status_message("No space for new POI", 2);
    }
}

/**
 * Broadcast a point of interest to the network
 */
void broadcast_point_of_interest(system_state_t* state, poi_t* poi) {
    if (DEBUG) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "Broadcasting POI: %s", poi->name);
        print_status_message(buffer, 0);
    }
    
    // Pack the POI data
    uint8_t payload[sizeof(poi_t)];
    memcpy(payload, poi, sizeof(poi_t));
    
    // Broadcast with normal priority
    send_message(state, 0xFFFF, MSG_POI, payload, sizeof(poi_t), PRIORITY_NORMAL);
}

/**
 * Process a received POI message
 */
void process_poi_message(system_state_t* state, message_t* msg) {
    if (msg->payload_len < sizeof(poi_t)) return;
    
    // Extract POI data
    poi_t poi;
    memcpy(&poi, msg->payload, sizeof(poi_t));
    
    if (DEBUG) {
        char loc_str[128];
        format_location_string(&poi.location, loc_str, sizeof(loc_str));
        
        char buffer[256];
        snprintf(buffer, sizeof(buffer), "Received POI: %s (%s)", 
                 poi.name, get_poi_type_string(poi.type));
        print_status_message(buffer, 0);
        printf("  Location: %s\n", loc_str);
        printf("  Description: %s\n", poi.description);
    }
    
    // Add to our database
    add_point_of_interest(state, &poi);
}

/**
 * Add a rescue zone to our database
 */
void add_rescue_zone(system_state_t* state, rescue_zone_t* zone) {
    // Check if we already have this zone (by name and location)
    for (int i = 0; i < state->map_system.zone_count; i++) {
        rescue_zone_t* existing = &state->map_system.rescue_zones[i];
        
        // Compare name
        if (strcmp(existing->name, zone->name) == 0) {
            // Same name, check if the location is close
            float distance = haversine_distance(
                existing->center.latitude, existing->center.longitude,
                zone->center.latitude, zone->center.longitude);
            
            if (distance < 100.0f) {  // Within 100 meters
                // Update the existing zone
                memcpy(existing, zone, sizeof(rescue_zone_t));
                if (DEBUG) {
                    char buffer[128];
                    snprintf(buffer, sizeof(buffer), "Updated existing rescue zone: %s", zone->name);
                    print_status_message(buffer, 1);
                }
                return;
            }
        }
    }
    
    // Add new zone if we have space
    if (state->map_system.zone_count < MAX_ZONES) {
        memcpy(&state->map_system.rescue_zones[state->map_system.zone_count], 
              zone, sizeof(rescue_zone_t));
        state->map_system.zone_count++;
        
        if (DEBUG) {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Added new rescue zone: %s", zone->name);
            print_status_message(buffer, 1);
        }
    } else {
        if (DEBUG) print_status_message("No space for new rescue zone", 2);
    }
}

/**
 * Broadcast a rescue zone to the network
 */
void broadcast_rescue_zone(system_state_t* state, rescue_zone_t* zone) {
    if (DEBUG) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "Broadcasting rescue zone: %s", zone->name);
        print_status_message(buffer, 0);
    }
    
    // Pack the zone data
    uint8_t payload[sizeof(rescue_zone_t)];
    memcpy(payload, zone, sizeof(rescue_zone_t));
    
    // Broadcast with high priority
    send_message(state, 0xFFFF, MSG_ZONE, payload, sizeof(rescue_zone_t), PRIORITY_HIGH);
}

/**
 * Process a received rescue zone message
 */
void process_zone_message(system_state_t* state, message_t* msg) {
    if (msg->payload_len < sizeof(rescue_zone_t)) return;
    
    // Extract zone data
    rescue_zone_t zone;
    memcpy(&zone, msg->payload, sizeof(rescue_zone_t));
    
    if (DEBUG) {
        char loc_str[128];
        format_location_string(&zone.center, loc_str, sizeof(loc_str));
        
        char buffer[256];
        snprintf(buffer, sizeof(buffer), 
                "Received rescue zone: %s (%s)", 
                zone.name, get_zone_status_string(zone.status));
        print_status_message(buffer, 0);
        printf("  Location: %s\n", loc_str);
        printf("  Radius: %.1f meters\n", zone.radius);
        printf("  Description: %s\n", zone.description);
    }
    
    // Add to our database
    add_rescue_zone(state, &zone);
}

/**
 * Check if a location is within a rescue zone
 */
bool is_in_rescue_zone(geo_location_t* location, rescue_zone_t* zone) {
    float distance = haversine_distance(
        location->latitude, location->longitude,
        zone->center.latitude, zone->center.longitude);
    
    return distance <= zone->radius;
}