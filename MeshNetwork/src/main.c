/**
 * Resilient Mesh Emergency Communication System
 * Enhanced with GPS, Offline Maps, and RF Triangulation
 * For Emergency Response Operations
 * 
 * main.c - Main application
 */

#include "../include/system.h"
#include "../include/hal.h"
#include "../include/message.h"
#include "../include/network.h"
#include "../include/location.h"
#include "../include/utils.h"

/* Signal handler for clean shutdown */
void handle_signal(int sig) {
    printf("\n%sReceived signal %d, shutting down...%s\n", TERM_YELLOW, sig, TERM_RESET);
    running = false;
}

/**
 * Main entry point
 */
int main(int argc, char **argv) {
    // Initialize random number generator
    srand(time(NULL));
    
    // Register signal handlers
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    
    // Make stdout unbuffered for better interactive experience
    setbuf(stdout, NULL);
    
    // Print welcome banner
    print_welcome_banner();
    
    // Parse command line arguments
    node_id_t node_id = 0;
    if (argc > 1) {
        node_id = atoi(argv[1]);
    } else {
        // Generate a node ID based on hostname
        char hostname[256];
        if (gethostname(hostname, sizeof(hostname)) == 0) {
            // Use hash of hostname as node ID
            unsigned int hash = 0;
            for (int i = 0; hostname[i] != '\0'; i++) {
                hash = hash * 31 + hostname[i];
            }
            node_id = hash & 0xFFFF;
        } else {
            // Generate random node ID
            node_id = (rand() & 0xFFFF);
        }
    }
    
    // Use custom port if specified (for simulation mode)
    if (argc > 2) {
        bt_port = atoi(argv[2]);
    }
    
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "Starting mesh network with node ID %d", node_id);
    print_status_message(buffer, 1);
    
    // Create hardware abstraction layer based on platform
    hal_t hal;
    
    // Check if we're running on a Raspberry Pi
    bool is_pi = is_raspberry_pi();
    
    if (is_pi) {
        print_status_message("Detected Raspberry Pi platform", 1);
        
        #ifdef USE_RASPBERRY_PI
        // Create Raspberry Pi HAL
        hal = create_raspberry_pi_hal();
        #else
        print_status_message("Raspberry Pi support not compiled in, using simulation mode", 2);
        hal = create_bt_hal();
        #endif
    } else {
        print_status_message("Using simulation mode", 0);
        hal = create_bt_hal();
    }
    
    // Initialize mesh network
    system_init(&mesh_state, hal, node_id);
    
    print_status_message("Mesh network initialized", 1);
    print_command_help();
    
    // Main loop
    char command[1024];
    uint32_t last_housekeeping = 0;
    uint32_t last_gps_update = 0;
    uint32_t last_location_broadcast = 0;
    
    while (running) {
        // Process mesh network tasks
        // Check for messages multiple times per cycle for better responsiveness
        for (int i = 0; i < 5; i++) {
            process_incoming_messages(&mesh_state);
            hal.sleep_ms(1); // Small sleep between checks
        }
        
        // Send beacons and location updates periodically
        uint32_t current_time = hal.get_time_ms();
        
        send_beacon(&mesh_state);
        
        // Update GPS data periodically
        if (current_time - last_gps_update >= 5000) { // Every 5 seconds
            update_gps_location(&mesh_state);
            last_gps_update = current_time;
        }
        
        // Broadcast our location periodically
        if (current_time - last_location_broadcast >= LOCATION_UPDATE_INTERVAL_MS) {
            broadcast_location(&mesh_state);
            last_location_broadcast = current_time;
        }
        
        process_queue(&mesh_state);
        
        // Periodic maintenance
        if (current_time - last_housekeeping >= 30000) { // Every 30 seconds
            clean_stale_nodes(&mesh_state);
            clean_stale_routes(&mesh_state);
            check_battery_status(&mesh_state);
            adapt_power_settings(&mesh_state); 
            last_housekeeping = current_time;
        }
        
        // Check for user input (non-blocking)
        fd_set readfds;
        struct timeval tv;
        
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        tv.tv_sec = 0;
        tv.tv_usec = 100000; // 100ms
        
        if (select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv) > 0) {
            if (fgets(command, sizeof(command), stdin) != NULL) {
                // Remove newline
                command[strcspn(command, "\n")] = 0;
                
                // Process command
                process_command(&mesh_state, command);
            }
        }
        
        // Short sleep to prevent high CPU usage
        hal.sleep_ms(10);
    }
    
    print_header("SHUTTING DOWN");
    
    // Clean up
    if (mesh_state.hal.hardware_type == 0 && bt_socket >= 0) {
        // Now bt_socket is properly declared in hal.h
        close(bt_socket);
    }
    
    #ifdef USE_RASPBERRY_PI
    if (mesh_state.hal.hardware_type == 1 && pi_gps_fd >= 0) {
        // Now pi_gps_fd is properly declared in hal.h
        serialClose(pi_gps_fd);
    }
    #endif
    
    if (mesh_state.hal.receiver_buffer) {
        free(mesh_state.hal.receiver_buffer);
    }
    
    // Free map tiles
    for (int i = 0; i < mesh_state.map_system.map_tile_count; i++) {
        if (mesh_state.map_system.map_tiles[i].data) {
            free(mesh_state.map_system.map_tiles[i].data);
        }
    }
    
    print_status_message("Mesh network shutdown complete", 1);
    
    return 0;
}