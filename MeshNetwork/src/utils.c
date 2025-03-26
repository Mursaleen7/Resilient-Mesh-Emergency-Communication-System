/**
 * Resilient Mesh Emergency Communication System
 * Enhanced with GPS, Offline Maps, and RF Triangulation
 * For Emergency Response Operations
 * 
 * utils.c - Utility functions
 */

#include "../include/utils.h"
#include "../include/message.h"
#include "../include/map.h"
#include "../include/location.h"

/**
 * Print a header with a title
 */
void print_header(const char* title) {
    int padding = (TABLE_WIDTH - strlen(title) - 4) / 2;
    printf("\n%s", TERM_BOLD);
    for (int i = 0; i < TABLE_WIDTH; i++) printf("=");
    printf("\n%*s", padding + 2, "");
    printf("%s%s%s", TERM_CYAN, title, TERM_RESET);
    printf("%s\n", TERM_BOLD);
    for (int i = 0; i < TABLE_WIDTH; i++) printf("=");
    printf("%s\n", TERM_RESET);
}

/**
 * Print a separator line
 */
void print_separator(void) {
    printf("%s", TERM_BOLD);
    for (int i = 0; i < TABLE_WIDTH; i++) printf("-");
    printf("%s\n", TERM_RESET);
}

/**
 * Print a status message with appropriate coloring
 * status_type: 0 = info, 1 = success, 2 = warning, 3 = error
 */
void print_status_message(const char* message, int status_type) {
    const char* prefix;
    const char* color;
    
    switch (status_type) {
        case 0: // Info
            prefix = "[INFO]";
            color = TERM_BLUE;
            break;
        case 1: // Success
            prefix = "[SUCCESS]";
            color = TERM_GREEN;
            break;
        case 2: // Warning
            prefix = "[WARNING]";
            color = TERM_YELLOW;
            break;
        case 3: // Error
            prefix = "[ERROR]";
            color = TERM_RED;
            break;
        default:
            prefix = "[LOG]";
            color = TERM_RESET;
            break;
    }
    
    printf("%s%s%s %s\n", color, prefix, TERM_RESET, message);
}

/**
 * Print a table header with column names
 */
void print_table_header(const char* columns[], int num_columns) {
    print_separator();
    printf("%s", TERM_BOLD);
    
    for (int i = 0; i < num_columns; i++) {
        if (i > 0) printf(" | ");
        printf("%s", columns[i]);
    }
    
    printf("%s\n", TERM_RESET);
    print_separator();
}

/**
 * Print a table row with values
 */
void print_table_row(const char* values[], int num_values) {
    for (int i = 0; i < num_values; i++) {
        if (i > 0) printf(" | ");
        printf("%s", values[i]);
    }
    printf("\n");
}

/**
 * Print detailed information about a location
 */
void print_location_info(geo_location_t* location) {
    if (!location) return;
    
    char loc_str[128];
    format_location_string(location, loc_str, sizeof(loc_str));
    
    print_separator();
    printf("%-15s: %s\n", "Coordinates", loc_str);
    printf("%-15s: %.1f meters\n", "Altitude", location->altitude);
    printf("%-15s: %.1f m/s\n", "Speed", location->speed);
    printf("%-15s: %.1f°\n", "Course", location->course);
    printf("%-15s: %.2f\n", "HDOP", location->hdop);
    printf("%-15s: %d\n", "Satellites", location->satellites);
    printf("%-15s: %s\n", "Fix Quality", get_gps_fix_string(location->fix_quality));
    print_separator();
}

/**
 * Print a welcome banner for the application
 */
void print_welcome_banner(void) {
    printf("\n\n%s%s", TERM_BOLD, TERM_CYAN);
    printf("╔══════════════════════════════════════════════════════════════════════════════╗\n");
    printf("║                                                                              ║\n");
    printf("║            RESILIENT MESH EMERGENCY COMMUNICATION SYSTEM                     ║\n");
    printf("║                                                                              ║\n");
    printf("║                Enhanced with GPS, Offline Maps and RF Triangulation          ║\n");
    printf("║                      For Emergency Response Operations                       ║\n");
    printf("║                                                                              ║\n");
    printf("╚══════════════════════════════════════════════════════════════════════════════╝\n");
    printf("%s\n\n", TERM_RESET);
}

/**
 * Print command help
 */
void print_command_help(void) {
    print_header("AVAILABLE COMMANDS");
    printf("%-25s: %s\n", "send <dest> <message>", "Send a message to another node");
    printf("%-25s: %s\n", "broadcast <message>", "Send a broadcast message to all nodes");
    printf("%-25s: %s\n", "status", "Show network status");
    printf("%-25s: %s\n", "scan", "Scan for new devices");
    printf("%-25s: %s\n", "ping <node_id>", "Ping another node");
    printf("%-25s: %s\n", "location", "Show current location");
    printf("%-25s: %s\n", "locations", "Show known node locations");
    printf("%-25s: %s\n", "poi <name> <lat> <lon> <type> <desc>", "Add a point of interest");
    printf("%-25s: %s\n", "", "Types: 0=general, 1=shelter, 2=medical, 3=water, 4=food");
    printf("%-25s: %s\n", "zone <name> <lat> <lon> <radius> <status> <desc>", "Add a rescue zone");
    printf("%-25s: %s\n", "", "Status: 0=proposed, 1=active, 2=clearing, 3=cleared");
    printf("%-25s: %s\n", "map <tile_id> <zoom>", "Request a map tile");
    printf("%-25s: %s\n", "help", "Show this help message");
    printf("%-25s: %s\n", "quit", "Exit the application");
    print_separator();
}

/**
 * Get priority as string
 */
char* get_priority_string(message_priority_t priority) {
    switch (priority) {
        case PRIORITY_LOW: return "Low";
        case PRIORITY_NORMAL: return "Normal";
        case PRIORITY_HIGH: return "High";
        case PRIORITY_EMERGENCY: return "Emergency";
        default: return "Unknown";
    }
}

/**
 * Get GPS fix type as string
 */
char* get_gps_fix_string(gps_fix_t fix) {
    switch (fix) {
        case GPS_FIX_NONE: return "No Fix";
        case GPS_FIX_GPS: return "GPS";
        case GPS_FIX_DGPS: return "DGPS";
        case GPS_FIX_ESTIMATED: return "Estimated (RF)";
        case GPS_FIX_MANUAL: return "Manual";
        case GPS_FIX_SIMULATION: return "Simulation";
        default: return "Unknown";
    }
}

/**
 * Show network status with detailed formatting
 */
void show_network_status(system_state_t* state) {
    print_header("NETWORK STATUS");
    
    // Node information
    printf("%sNode ID:%s %d\n", TERM_BOLD, TERM_RESET, state->node_id);
    
    // Battery and channel info
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "%d%%", state->battery_level);
    
    const char* battery_color;
    if (state->battery_level <= BATTERY_CRIT_THRESHOLD) {
        battery_color = TERM_RED;
    } else if (state->battery_level <= BATTERY_LOW_THRESHOLD) {
        battery_color = TERM_YELLOW;
    } else {
        battery_color = TERM_GREEN;
    }
    
    printf("%sBattery:%s %s%s%s\n", TERM_BOLD, TERM_RESET, battery_color, buffer, TERM_RESET);
    printf("%sRF Channel:%s %d\n", TERM_BOLD, TERM_RESET, state->current_rf_channel);
    
    // Location info
    print_separator();
    printf("%sCurrent Location:%s\n", TERM_BOLD, TERM_RESET);
    print_location_info(&state->location);
    
    // Known nodes
    print_header("KNOWN NODES");
    if (state->node_count == 0) {
        printf("No known nodes\n");
    } else {
        const char* columns[] = {"Node ID", "Signal", "Battery", "Hops", "Last Seen", "Distance"};
        print_table_header(columns, 6);
        
        for (int i = 0; i < state->node_count; i++) {
            char node_id[16], signal[16], battery[16], hops[16], last_seen[32], distance[32];
            
            snprintf(node_id, sizeof(node_id), "%d", state->known_nodes[i].id);
            snprintf(signal, sizeof(signal), "%d", state->known_nodes[i].signal_strength);
            
            if (state->known_nodes[i].battery_level <= BATTERY_CRIT_THRESHOLD) {
                snprintf(battery, sizeof(battery), "%s%d%%%s", TERM_RED, 
                        state->known_nodes[i].battery_level, TERM_RESET);
            } else if (state->known_nodes[i].battery_level <= BATTERY_LOW_THRESHOLD) {
                snprintf(battery, sizeof(battery), "%s%d%%%s", TERM_YELLOW, 
                        state->known_nodes[i].battery_level, TERM_RESET);
            } else {
                snprintf(battery, sizeof(battery), "%s%d%%%s", TERM_GREEN, 
                        state->known_nodes[i].battery_level, TERM_RESET);
            }
            
            snprintf(hops, sizeof(hops), "%d", state->known_nodes[i].hop_count);
            
            uint32_t time_diff = state->hal.get_time_ms() - state->known_nodes[i].last_seen;
            if (time_diff < 1000) {
                snprintf(last_seen, sizeof(last_seen), "just now");
            } else if (time_diff < 60000) {
                snprintf(last_seen, sizeof(last_seen), "%d seconds ago", time_diff / 1000);
            } else if (time_diff < 3600000) {
                snprintf(last_seen, sizeof(last_seen), "%d minutes ago", time_diff / 60000);
            } else {
                snprintf(last_seen, sizeof(last_seen), "%d hours ago", time_diff / 3600000);
            }
            
            if (state->known_nodes[i].distance_estimate > 0) {
                snprintf(distance, sizeof(distance), "%.1f m", state->known_nodes[i].distance_estimate);
            } else {
                snprintf(distance, sizeof(distance), "Unknown");
            }
            
            const char* values[] = {node_id, signal, battery, hops, last_seen, distance};
            print_table_row(values, 6);
            
            // If we have location info for this node, display it
            if (state->known_nodes[i].location.fix_quality != GPS_FIX_NONE) {
                char loc_str[128];
                format_location_string(&state->known_nodes[i].location, loc_str, sizeof(loc_str));
                printf("  %sLocation:%s %s\n", TERM_BOLD, TERM_RESET, loc_str);
            }
        }
    }
    
    // Routes
    print_header("ROUTING TABLE");
    if (state->route_count == 0) {
        printf("No routes\n");
    } else {
        const char* columns[] = {"Destination", "Next Hop", "Hops", "Quality", "Last Updated"};
        print_table_header(columns, 5);
        
        for (int i = 0; i < state->route_count; i++) {
            char dest[16], next_hop[16], hops[16], quality[16], last_updated[32];
            
            snprintf(dest, sizeof(dest), "%d", state->routing_table[i].destination);
            snprintf(next_hop, sizeof(next_hop), "%d", state->routing_table[i].next_hop);
            snprintf(hops, sizeof(hops), "%d", state->routing_table[i].hop_count);
            snprintf(quality, sizeof(quality), "%d", state->routing_table[i].signal_quality);
            
            uint32_t time_diff = state->hal.get_time_ms() - state->routing_table[i].last_updated;
            if (time_diff < 1000) {
                snprintf(last_updated, sizeof(last_updated), "just now");
            } else if (time_diff < 60000) {
                snprintf(last_updated, sizeof(last_updated), "%d seconds ago", time_diff / 1000);
            } else if (time_diff < 3600000) {
                snprintf(last_updated, sizeof(last_updated), "%d minutes ago", time_diff / 60000);
            } else {
                snprintf(last_updated, sizeof(last_updated), "%d hours ago", time_diff / 3600000);
            }
            
            const char* values[] = {dest, next_hop, hops, quality, last_updated};
            print_table_row(values, 5);
        }
    }
    
    // Points of Interest
    if (state->map_system.poi_count > 0) {
        print_header("POINTS OF INTEREST");
        
        const char* columns[] = {"Name", "Type", "Location"};
        print_table_header(columns, 3);
        
        for (int i = 0; i < state->map_system.poi_count; i++) {
            poi_t* poi = &state->map_system.points_of_interest[i];
            char loc_str[128];
            format_location_string(&poi->location, loc_str, sizeof(loc_str));
            
            const char* values[] = {poi->name, get_poi_type_string(poi->type), loc_str};
            print_table_row(values, 3);
            
            printf("  %sDescription:%s %s\n", TERM_BOLD, TERM_RESET, poi->description);
        }
    }
    
    // Rescue Zones
    if (state->map_system.zone_count > 0) {
        print_header("RESCUE ZONES");
        
        const char* columns[] = {"Name", "Status", "Center", "Radius"};
        print_table_header(columns, 4);
        
        for (int i = 0; i < state->map_system.zone_count; i++) {
            rescue_zone_t* zone = &state->map_system.rescue_zones[i];
            char loc_str[128], radius[32];
            
            format_location_string(&zone->center, loc_str, sizeof(loc_str));
            snprintf(radius, sizeof(radius), "%.1f m", zone->radius);
            
            const char* values[] = {
                zone->name, 
                get_zone_status_string(zone->status),
                loc_str,
                radius
            };
            
            print_table_row(values, 4);
            printf("  %sDescription:%s %s\n", TERM_BOLD, TERM_RESET, zone->description);
            
            // Check if we're in this zone
            if (is_in_rescue_zone(&state->location, zone)) {
                printf("  %s>>> You are currently inside this zone! <<<%s\n", 
                      TERM_BG_YELLOW TERM_BLACK, TERM_RESET);
            }
        }
    }
    
    // Show map tiles
    if (state->map_system.map_tile_count > 0) {
        print_header("MAP TILES");
        
        const char* columns[] = {"Tile ID", "Zoom", "Size", "Last Updated"};
        print_table_header(columns, 4);
        
        for (int i = 0; i < state->map_system.map_tile_count; i++) {
            map_tile_t* tile = &state->map_system.map_tiles[i];
            char tile_id[16], zoom[16], size[32], updated[32];
            
            snprintf(tile_id, sizeof(tile_id), "%d", tile->tile_id);
            snprintf(zoom, sizeof(zoom), "%d", tile->zoom_level);
            snprintf(size, sizeof(size), "%d bytes", tile->data_size);
            
            time_t timestamp = tile->timestamp;
            struct tm* tm_info = localtime(&timestamp);
            strftime(updated, sizeof(updated), "%Y-%m-%d %H:%M:%S", tm_info);
            
            const char* values[] = {tile_id, zoom, size, updated};
            print_table_row(values, 4);
        }
    }
    
    print_separator();
}