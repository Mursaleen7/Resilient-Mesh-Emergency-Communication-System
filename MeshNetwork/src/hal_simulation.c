/**
 * Resilient Mesh Emergency Communication System
 * Enhanced with GPS, Offline Maps, and RF Triangulation
 * For Emergency Response Operations
 * 
 * hal_simulation.c - Bluetooth simulation HAL implementation
 */

#include "../include/hal.h"
#include "../include/utils.h"
#include "../include/message.h"

/* ---- Bluetooth specific data structures (for simulation fallback) ---- */
#define BT_MAX_DEVICES 10

typedef struct {
    char addr[18];          // Bluetooth address as string (e.g., "00:11:22:33:44:55")
    char name[248];         // Device name
    int8_t rssi;            // Signal strength
    uint8_t connected;      // Connection status
} bt_device_t;

/* Bluetooth state */
int bt_socket = -1; /* Changed from static to make it accessible from main.c */
static bt_device_t known_bt_devices[BT_MAX_DEVICES];
static int bt_device_count = 0;
static uint8_t bt_rx_buffer[MAX_MSG_SIZE];
static uint16_t bt_rx_buffer_len = 0;
static uint8_t bt_last_rssi = 0;
int bt_port = 31415;  // Port used for Bluetooth communication

/* Function prototypes for Bluetooth HAL */
static void bt_init_radio(void);
static bool bt_send_packet(uint8_t* data, uint16_t len, uint8_t power_level);
static uint16_t bt_receive_packet(uint8_t* buffer, uint16_t max_len);
static void bt_sleep_ms(uint32_t ms);
static uint32_t bt_get_time_ms(void);
static battery_level_t bt_get_battery_level(void);
static void bt_set_led(bool state);
static bool bt_get_button_state(uint8_t button_id);
static void bt_enter_low_power_mode(void);
static void bt_exit_low_power_mode(void);
static void bt_set_rf_channel(uint8_t channel);
static signal_strength_t bt_get_last_rssi(void);
static int16_t bt_get_temperature(void);
static uint8_t bt_get_random_byte(void);
static bool bt_get_gps_data(geo_location_t* location);
void bt_scan_for_devices(void);

/**
 * Create HAL structure with Bluetooth functions
 */
hal_t create_bt_hal(void) {
    hal_t hal;
    
    hal.init_radio = bt_init_radio;
    hal.send_packet = bt_send_packet;
    hal.receive_packet = bt_receive_packet;
    hal.sleep_ms = bt_sleep_ms;
    hal.get_time_ms = bt_get_time_ms;
    hal.get_battery_level = bt_get_battery_level;
    hal.set_led = bt_set_led;
    hal.get_button_state = bt_get_button_state;
    hal.enter_low_power_mode = bt_enter_low_power_mode;
    hal.exit_low_power_mode = bt_exit_low_power_mode;
    hal.set_rf_channel = bt_set_rf_channel;
    hal.get_last_rssi = bt_get_last_rssi;
    hal.get_temperature = bt_get_temperature;
    hal.get_random_byte = bt_get_random_byte;
    hal.get_gps_data = bt_get_gps_data;
    
    // Allocate receive buffer
    hal.receiver_buffer = malloc(MAX_MSG_SIZE);
    hal.receiver_len = 0;
    hal.hardware_type = 0; // Simulation
    
    return hal;
}

/**
 * Enable Bluetooth for simulation
 */
static void bt_init_radio(void) {
    print_status_message("Initializing Bluetooth simulation...", 0);
    
    // Create a UDP socket to simulate Bluetooth communication
    bt_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (bt_socket < 0) {
        perror("Failed to create socket");
        return;
    }
    
    // Enable broadcast
    int broadcast = 1;
    if (setsockopt(bt_socket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0) {
        perror("Failed to set broadcast option");
    }
    
    // Set up socket address structure
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(bt_port);
    
    // Bind the socket
    if (bind(bt_socket, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("Failed to bind socket");
        close(bt_socket);
        bt_socket = -1;
        return;
    }
    
    // Set socket to non-blocking mode
    int flags = fcntl(bt_socket, F_GETFL, 0);
    fcntl(bt_socket, F_SETFL, flags | O_NONBLOCK);
    
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "Bluetooth simulation initialized on port %d", bt_port);
    print_status_message(buffer, 1);
    
    // Scan for devices
    bt_scan_for_devices();
}

/**
 * Scan for nearby Bluetooth devices (simulated)
 */
void bt_scan_for_devices(void) {
    print_status_message("Scanning for simulated devices...", 0);
    
    // Reset device count
    bt_device_count = 0;
    
    // Add virtual nodes at different ports
    for (int i = 0; i < 3; i++) {
        int target_port = bt_port + i - 1;  // Include ports before and after current port
        
        // Skip our own port
        if (target_port == bt_port) {
            continue;
        }
        
        // Skip negative ports
        if (target_port <= 0) {
            continue;
        }
        
        snprintf(known_bt_devices[bt_device_count].addr, 18, "127.0.0.1:%d", target_port);
        snprintf(known_bt_devices[bt_device_count].name, 248, "Virtual Node %d", target_port);
        known_bt_devices[bt_device_count].rssi = 80 - (abs(target_port - bt_port) * 10); // Simulated RSSI
        known_bt_devices[bt_device_count].connected = 0;
        
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "Found virtual device: %s (%s) RSSI: %d", 
               known_bt_devices[bt_device_count].name, 
               known_bt_devices[bt_device_count].addr, 
               known_bt_devices[bt_device_count].rssi);
        print_status_message(buffer, 1);
        
        bt_device_count++;
    }
    
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "Found %d devices", bt_device_count);
    print_status_message(buffer, 0);
}

/**
 * Send a packet via simulated Bluetooth
 */
static bool bt_send_packet(uint8_t* data, uint16_t len, uint8_t power_level) {
    bool success = false;
    
    // Create message_t pointer for debug info
    message_t* msg = (message_t*)data;
    
    if (DEBUG && len >= sizeof(message_t)) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "Sending packet of type %s from node %d to node %d", 
               get_message_type_name(msg->type), msg->source, msg->destination);
        print_status_message(buffer, 0);
    }
    
    // Send to all known devices
    for (int i = 0; i < bt_device_count; i++) {
        // Skip devices that are too far away based on power level
        if (known_bt_devices[i].rssi < 40 + power_level * 10) {
            if (DEBUG) {
                char buffer[128];
                snprintf(buffer, sizeof(buffer), "Device %s is too far for current power level", 
                        known_bt_devices[i].name);
                print_status_message(buffer, 2);
            }
            continue; // Device is too far for current power level
        }
        
        // Parse the address to get IP and port
        char ip[16];
        int port;
        if (sscanf(known_bt_devices[i].addr, "%15[^:]:%d", ip, &port) != 2) {
            if (DEBUG) {
                char buffer[128];
                snprintf(buffer, sizeof(buffer), "Invalid address format: %s", 
                        known_bt_devices[i].addr);
                print_status_message(buffer, 3);
            }
            continue; // Invalid address format
        }
        
        // Set up destination address
        struct sockaddr_in dest_addr;
        memset(&dest_addr, 0, sizeof(dest_addr));
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_addr.s_addr = inet_addr(ip);
        dest_addr.sin_port = htons(port);
        
        // Send the data
        ssize_t sent = sendto(bt_socket, data, len, 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
        if (sent == len) {
            if (DEBUG) {
                char buffer[128];
                snprintf(buffer, sizeof(buffer), "Sent packet to %s", known_bt_devices[i].name);
                print_status_message(buffer, 1);
            }
            success = true;
        } else {
            perror("Failed to send packet");
            if (DEBUG) {
                char buffer[256];
                snprintf(buffer, sizeof(buffer), 
                        "sendto error: %s (errno: %d), Destination: %s:%d", 
                        strerror(errno), errno, ip, port);
                print_status_message(buffer, 3);
            }
        }
    }
    
    return success;
}

/**
 * Receive a packet via simulated Bluetooth
 */
static uint16_t bt_receive_packet(uint8_t* buffer, uint16_t max_len) {
    // Check if we have data in the buffer
    if (bt_rx_buffer_len > 0 && bt_rx_buffer_len <= max_len) {
        memcpy(buffer, bt_rx_buffer, bt_rx_buffer_len);
        uint16_t len = bt_rx_buffer_len;
        bt_rx_buffer_len = 0;
        return len;
    }
    
    // Try to receive data
    struct sockaddr_in sender_addr;
    socklen_t sender_len = sizeof(sender_addr);
    int received = recvfrom(bt_socket, buffer, max_len, 0, 
                          (struct sockaddr*)&sender_addr, &sender_len);
    
    if (received > 0) {
        // Log packet receipt
        if (DEBUG && (size_t)received >= sizeof(message_t)) {  // Fixed sign comparison issue here
            message_t* msg = (message_t*)buffer;
            char sender_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &(sender_addr.sin_addr), sender_ip, INET_ADDRSTRLEN);
            
            char buffer[256];
            snprintf(buffer, sizeof(buffer), 
                    "Received packet from %s:%d, type: %s, source: %d, dest: %d, size: %d",
                    sender_ip, ntohs(sender_addr.sin_port),
                    get_message_type_name(msg->type), msg->source, msg->destination, received);
            print_status_message(buffer, 0);
        }
        
        // Update simulated RSSI
        bt_last_rssi = 70; // Default RSSI value
        
        // Find the device in our list and update its RSSI
        char addr_str[18];
        char sender_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &(sender_addr.sin_addr), sender_ip, INET_ADDRSTRLEN);
        snprintf(addr_str, sizeof(addr_str), "%s:%d", 
                sender_ip, ntohs(sender_addr.sin_port));
        
        for (int i = 0; i < bt_device_count; i++) {
            if (strcmp(known_bt_devices[i].addr, addr_str) == 0) {
                bt_last_rssi = known_bt_devices[i].rssi;
                break;
            }
        }
        
        return received;
    }
    
    return 0;
}

/**
 * Sleep for the specified number of milliseconds
 */
static void bt_sleep_ms(uint32_t ms) {
    usleep(ms * 1000);
}

/**
 * Get current time in milliseconds
 */
static uint32_t bt_get_time_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

/**
 * Get battery level (simulated)
 */
static battery_level_t bt_get_battery_level(void) {
    static int battery_level = 100;
    static uint32_t last_update = 0;
    
    // Slowly decrease battery level over time
    uint32_t current_time = bt_get_time_ms();
    if (current_time - last_update > 60000) { // Every minute
        battery_level--;
        last_update = current_time;
    }
    
    // Ensure it doesn't go below 5%
    if (battery_level < 5) {
        battery_level = 5;
    }
    
    return (battery_level_t)battery_level;
}

/**
 * Set LED (just print status)
 */
static void bt_set_led(bool state) {
    if (DEBUG) printf("LED: %s\n", state ? "ON" : "OFF");
}

/**
 * Get button state (simulated)
 * The button_id parameter is deliberately unused in this simulation
 */
static bool bt_get_button_state(uint8_t button_id) {
    (void)button_id;  /* Mark parameter as deliberately unused to avoid warning */
    return false;
}

/**
 * Enter low power mode (simulated)
 */
static void bt_enter_low_power_mode(void) {
    print_status_message("Entering low power mode (simulated)", 2);
}

/**
 * Exit low power mode (simulated)
 */
static void bt_exit_low_power_mode(void) {
    print_status_message("Exiting low power mode (simulated)", 1);
}

/**
 * Set RF channel (simulated)
 */
static void bt_set_rf_channel(uint8_t channel) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "Switching to channel %d (simulated)", channel);
    print_status_message(buffer, 0);
}

/**
 * Get last RSSI value (simulated)
 */
static signal_strength_t bt_get_last_rssi(void) {
    return bt_last_rssi;
}

/**
 * Get temperature (simulated)
 */
static int16_t bt_get_temperature(void) {
    return 20 * 10; // 20°C fixed value
}

/**
 * Get random byte (simulated)
 */
static uint8_t bt_get_random_byte(void) {
    return rand() & 0xFF;
}

/**
 * Get GPS data (simulated)
 */
static bool bt_get_gps_data(geo_location_t* location) {
    // 80% chance of getting a GPS fix
    if (rand() % 100 < 80) {
        // Generate a random location near San Francisco for testing
        location->latitude = 37.7749 + ((double)rand() / RAND_MAX - 0.5) * 0.02;
        location->longitude = -122.4194 + ((double)rand() / RAND_MAX - 0.5) * 0.02;
        location->altitude = 10.0 + (rand() % 200);
        location->speed = (float)(rand() % 10);
        location->course = (float)(rand() % 360);
        location->hdop = 1.0f + ((float)rand() / RAND_MAX) * 2.0f;
        location->satellites = 6 + (rand() % 8);
        location->timestamp = bt_get_time_ms();
        location->fix_quality = GPS_FIX_GPS;
        
        return true;
    } else {
        // No GPS fix
        location->fix_quality = GPS_FIX_NONE;
        location->satellites = 0;
        location->timestamp = bt_get_time_ms();
        
        return false;
    }
}