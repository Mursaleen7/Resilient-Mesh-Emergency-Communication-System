/**
 * Resilient Mesh Emergency Communication System
 * Enhanced with GPS, Offline Maps, and RF Triangulation
 * For Emergency Response Operations
 * 
 * system.c - System initialization and management
 */

#include "../include/system.h"
#include "../include/message.h"
#include "../include/network.h"
#include "../include/location.h"
#include "../include/map.h"
#include "../include/utils.h"

// Global state and running flag
system_state_t mesh_state;
bool running = true;

/**
 * Initialize the system state
 */
void system_init(system_state_t* state, hal_t hal, node_id_t node_id) {
    memset(state, 0, sizeof(system_state_t));
    state->hal = hal;
    state->node_id = node_id;
    state->outbound_queue = NULL;
    state->outbound_queue_size = 0;
    state->last_beacon_time = 0;
    state->battery_level = state->hal.get_battery_level();
    state->current_rf_channel = 0;
    state->low_power_mode = false;
    state->next_msg_id = 0;
    
    // Initialize hardware
    state->hal.init_radio();
    state->hal.set_rf_channel(state->current_rf_channel);
    
    // Initialize map system
    init_map_system(&state->map_system);
    
    // Try to get a GPS fix
    update_gps_location(state);
    
    // Initialize with a random message ID
    for (int i = 0; i < 4; i++) {
        state->next_msg_id = (state->next_msg_id << 8) | state->hal.get_random_byte();
    }
    
    // Use fixed encryption key for all nodes
    uint8_t key[ENCRYPTION_KEY_SIZE];
    memset(key, 0, ENCRYPTION_KEY_SIZE);
    strncpy((char*)key, SHARED_KEY, ENCRYPTION_KEY_SIZE);
    set_encryption_key(state, key);
}

/**
 * Set the encryption key for secure communications
 */
void set_encryption_key(system_state_t* state, const uint8_t* key) {
    memcpy(state->encryption_key, key, ENCRYPTION_KEY_SIZE);
}

/**
 * Update the node's geographic location
 */
void set_location(system_state_t* state, geo_location_t location) {
    state->location = location;
    
    // Update in our location database too
    update_node_location(state, state->node_id, &location);
}

/**
 * Process user commands
 */
void process_command(system_state_t* state, char* command) {
    if (strncmp(command, "send ", 5) == 0) {
        // Format: send <destination> <message>
        int dest;
        char message[MAX_MSG_SIZE];
        
        if (sscanf(command, "send %d %[^\n]", &dest, message) == 2) {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Sending message to node %d: %s", dest, message);
            print_status_message(buffer, 0);
            
            send_message(state, dest, MSG_DATA, 
                        (uint8_t*)message, strlen(message) + 1, PRIORITY_NORMAL);
        } else {
            print_status_message("Invalid format. Use: send <destination> <message>", 3);
        }
    } else if (strncmp(command, "broadcast ", 10) == 0) {
        // Format: broadcast <message>
        char message[MAX_MSG_SIZE];
        
        if (sscanf(command, "broadcast %[^\n]", message) == 1) {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Broadcasting message: %s", message);
            print_status_message(buffer, 0);
            
            send_message(state, 0xFFFF, MSG_DATA, 
                        (uint8_t*)message, strlen(message) + 1, PRIORITY_NORMAL);
        } else {
            print_status_message("Invalid format. Use: broadcast <message>", 3);
        }
    } else if (strcmp(command, "status") == 0 || strcmp(command, "st") == 0) {
        // Show network status
        show_network_status(state);
    } else if (strcmp(command, "scan") == 0) {
        // Rescan for devices
        if (state->hal.hardware_type == 0) {
            // Use Bluetooth scan for simulation mode
            extern void bt_scan_for_devices(void);
            bt_scan_for_devices();
        } else {
            print_status_message("Scanning for devices...", 0);
            // For Raspberry Pi, this would trigger an RF scan
            scan_channels(state);
        }
    } else if (strncmp(command, "ping ", 5) == 0) {
        // Format: ping <node_id>
        int dest;
        
        if (sscanf(command, "ping %d", &dest) == 1) {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Pinging node %d", dest);
            print_status_message(buffer, 0);
            
            // Create ping message with timestamp
            uint32_t timestamp = state->hal.get_time_ms();
            uint8_t payload[8];
            memcpy(payload, &timestamp, sizeof(timestamp));
            
            send_message(state, dest, MSG_PING, payload, sizeof(timestamp), PRIORITY_LOW);
        } else {
            print_status_message("Invalid format. Use: ping <node_id>", 3);
        }
    } else if (strcmp(command, "location") == 0 || strcmp(command, "loc") == 0) {
        // Show current location
        update_gps_location(state);
        
        print_header("CURRENT LOCATION");
        print_location_info(&state->location);
    } else if (strcmp(command, "locations") == 0) {
        // Show all known locations
        print_header("KNOWN NODE LOCATIONS");
        
        printf("%sNode %d (self):%s\n", TERM_BOLD, state->node_id, TERM_RESET);
        print_location_info(&state->location);
        
        for (int i = 0; i < state->node_count; i++) {
            node_info_t* node = &state->known_nodes[i];
            if (node->location.fix_quality != GPS_FIX_NONE) {
                printf("\n%sNode %d:%s\n", TERM_BOLD, node->id, TERM_RESET);
                print_location_info(&node->location);
                
                // Calculate distance
                float distance = haversine_distance(
                    state->location.latitude, state->location.longitude,
                    node->location.latitude, node->location.longitude);
                    
                printf("Distance: %.2f meters\n", distance);
            }
        }
    } else if (strncmp(command, "poi ", 4) == 0) {
        // Format: poi <name> <lat> <lon> <type> <desc>
        char name[32];
        double lat, lon;
        int type;
        char desc[128];
        
        if (sscanf(command, "poi %31s %lf %lf %d %127[^\n]", 
                 name, &lat, &lon, &type, desc) == 5) {
            // Create and add POI
            poi_t poi;
            memset(&poi, 0, sizeof(poi_t));
            
            strncpy(poi.name, name, sizeof(poi.name) - 1);
            strncpy(poi.description, desc, sizeof(poi.description) - 1);
            
            poi.location.latitude = lat;
            poi.location.longitude = lon;
            poi.location.altitude = state->location.altitude;
            poi.location.fix_quality = GPS_FIX_MANUAL;
            poi.location.timestamp = state->hal.get_time_ms();
            
            poi.type = type;
            poi.timestamp = state->hal.get_time_ms();
            
            // Add to our database
            add_point_of_interest(state, &poi);
            
            // Broadcast to network
            broadcast_point_of_interest(state, &poi);
            
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Added and broadcast POI: %s", name);
            print_status_message(buffer, 1);
        } else {
            print_status_message("Invalid format. Use: poi <name> <lat> <lon> <type> <desc>", 3);
            printf("Types: 0=general, 1=shelter, 2=medical, 3=water, 4=food\n");
        }
    } else if (strncmp(command, "zone ", 5) == 0) {
        // Format: zone <name> <lat> <lon> <radius> <status> <desc>
        char name[32];
        double lat, lon;
        float radius;
        int status;
        char desc[128];
        
if (sscanf(command, "zone %31s %lf %lf %f %d %127[^\n]", 
                 name, &lat, &lon, &radius, &status, desc) == 6) {
            // Create and add rescue zone
            rescue_zone_t zone;
            memset(&zone, 0, sizeof(rescue_zone_t));
            
            strncpy(zone.name, name, sizeof(zone.name) - 1);
            strncpy(zone.description, desc, sizeof(zone.description) - 1);
            
            zone.center.latitude = lat;
            zone.center.longitude = lon;
            zone.center.altitude = state->location.altitude;
            zone.center.fix_quality = GPS_FIX_MANUAL;
            zone.center.timestamp = state->hal.get_time_ms();
            
            zone.radius = radius;
            zone.status = status;
            zone.timestamp = state->hal.get_time_ms();
            
            // Add to our database
            add_rescue_zone(state, &zone);
            
            // Broadcast to network
            broadcast_rescue_zone(state, &zone);
            
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Added and broadcast rescue zone: %s", name);
            print_status_message(buffer, 1);
            
            // Check if we're in this zone
            if (is_in_rescue_zone(&state->location, &zone)) {
                print_status_message("Alert: You are currently inside this rescue zone!", 2);
            }
        } else {
            print_status_message("Invalid format. Use: zone <name> <lat> <lon> <radius> <status> <desc>", 3);
            printf("Status: 0=proposed, 1=active, 2=clearing, 3=cleared\n");
        }
    } else if (strncmp(command, "map ", 4) == 0) {
        // Format: map <tile_id> <zoom>
        int tile_id, zoom;
        
        if (sscanf(command, "map %d %d", &tile_id, &zoom) == 2) {
            // First try to load locally
            bool loaded = load_map_tile(&state->map_system, tile_id, zoom);
            
            if (!loaded) {
                // Request from network
                request_map_tile(state, tile_id, zoom);
                
                char buffer[128];
                snprintf(buffer, sizeof(buffer), 
                        "Requested map tile %d (zoom %d) from network", tile_id, zoom);
                print_status_message(buffer, 0);
            } else {
                char buffer[128];
                snprintf(buffer, sizeof(buffer), 
                        "Loaded map tile %d (zoom %d) from local storage", tile_id, zoom);
                print_status_message(buffer, 1);
            }
        } else {
            print_status_message("Invalid format. Use: map <tile_id> <zoom>", 3);
        }
    } else if (strcmp(command, "help") == 0 || strcmp(command, "?") == 0) {
        print_command_help();
    } else if (strcmp(command, "quit") == 0 || strcmp(command, "exit") == 0) {
        running = false;
    } else if (strlen(command) > 0) {
        print_status_message("Unknown command. Type 'help' for available commands.", 3);
    }
}