/**
 * Resilient Mesh Emergency Communication System
 * Enhanced with GPS, Offline Maps, and RF Triangulation
 * For Emergency Response Operations
 * 
 * common.h - Common definitions, types, and constants
 */

#ifndef MESH_COMMON_H
#define MESH_COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>  /* For sockaddr_in and INADDR_ANY */
#include <arpa/inet.h>   /* For inet_addr and inet_ntoa */
#include <math.h>        /* For triangulation calculations */
#include <sys/utsname.h> /* For detecting Raspberry Pi */

/* ---- Terminal Formatting Constants ---- */
#define TERM_RESET      "\033[0m"
#define TERM_BOLD       "\033[1m"
#define TERM_UNDERLINE  "\033[4m"
#define TERM_BLINK      "\033[5m"
#define TERM_INVERT     "\033[7m"

/* Text color */
#define TERM_BLACK      "\033[30m"
#define TERM_RED        "\033[31m"
#define TERM_GREEN      "\033[32m"
#define TERM_YELLOW     "\033[33m"
#define TERM_BLUE       "\033[34m"
#define TERM_MAGENTA    "\033[35m"
#define TERM_CYAN       "\033[36m"
#define TERM_WHITE      "\033[37m"

/* Background color */
#define TERM_BG_BLACK   "\033[40m"
#define TERM_BG_RED     "\033[41m"
#define TERM_BG_GREEN   "\033[42m"
#define TERM_BG_YELLOW  "\033[43m"
#define TERM_BG_BLUE    "\033[44m"
#define TERM_BG_MAGENTA "\033[45m"
#define TERM_BG_CYAN    "\033[46m"
#define TERM_BG_WHITE   "\033[47m"

/* ---- Configuration Constants ---- */
#define MAX_NODES 64
#define MAX_ROUTES 16
#define MAX_MSG_SIZE 1024  // Increased to support map data
#define MAX_QUEUE_SIZE 32
#define NODE_TIMEOUT_MS 300000   // 5 minutes
#define BEACON_INTERVAL_MS 10000 // 10 seconds
#define LOCATION_UPDATE_INTERVAL_MS 15000 // 15 seconds
#define RETRY_INTERVAL_MS 5000   // 5 seconds
#define MAX_HOPS 8
#define BATTERY_CRIT_THRESHOLD 10
#define BATTERY_LOW_THRESHOLD 25
#define ENCRYPTION_KEY_SIZE 32
#define MAC_SIZE 16
#define RF_CHANNEL_COUNT 16
#define DEBUG 1 // Enable debug output
#define MAX_MAP_TILES 16
#define MAX_POI 32     // Points of Interest
#define MAX_ZONES 8    // Rescue zones
#define SHARED_KEY "EMERGENCY_MESH_NETWORK_KEY_2025" // Use a fixed key for all nodes
#define TABLE_WIDTH 80 // Width for table formatting

/* ---- Raspberry Pi Specific Constants ---- */
#define PI_RF_SPI_CHANNEL 0       // SPI channel for RF module
#define PI_RF_SPI_SPEED 1000000   // 1MHz SPI speed
#define PI_GPS_SERIAL_DEVICE "/dev/ttyS0" // UART for GPS
#define PI_GPS_BAUD_RATE 9600     // GPS baud rate

/* ---- GPIO Pin Assignments ---- */
#define LED_PIN 0        // BCM GPIO 17
#define BUTTON_PIN 2     // BCM GPIO 27
#define RF_CS_PIN 10     // BCM GPIO 8 (SPI CE0)
#define RF_IRQ_PIN 7     // BCM GPIO 4

/* ---- System Typedefs ---- */
typedef uint16_t node_id_t;
typedef uint8_t hop_count_t;
typedef uint8_t msg_id_t;
typedef uint8_t battery_level_t;
typedef uint8_t signal_strength_t;

/* ---- Message Types ---- */
typedef enum {
    MSG_BEACON = 0x01,
    MSG_DATA = 0x02,
    MSG_ACK = 0x03,
    MSG_ROUTE_REQUEST = 0x04,
    MSG_ROUTE_RESPONSE = 0x05,
    MSG_ALERT = 0x06,
    MSG_PING = 0x07,
    MSG_PONG = 0x08,
    MSG_LOCATION = 0x09,      // New message type for location updates
    MSG_MAP_REQUEST = 0x0A,   // Request map tile
    MSG_MAP_DATA = 0x0B,      // Send map tile data
    MSG_POI = 0x0C,           // Point of Interest
    MSG_ZONE = 0x0D           // Rescue zone
} message_type_t;

/* ---- Message Priority Levels ---- */
typedef enum {
    PRIORITY_LOW = 0,
    PRIORITY_NORMAL = 1,
    PRIORITY_HIGH = 2,
    PRIORITY_EMERGENCY = 3
} message_priority_t;

/* ---- GPS Fix Quality ---- */
typedef enum {
    GPS_FIX_NONE = 0,
    GPS_FIX_GPS = 1,
    GPS_FIX_DGPS = 2,
    GPS_FIX_ESTIMATED = 3,    // RF triangulation
    GPS_FIX_MANUAL = 4,       // Manually entered
    GPS_FIX_SIMULATION = 5    // Simulated position
} gps_fix_t;

/* ---- Enhanced Geographic Location ---- */
typedef struct {
    double latitude;           // Decimal degrees
    double longitude;          // Decimal degrees
    float altitude;            // Meters above sea level
    float speed;               // Meters per second
    float course;              // Degrees from true north
    float hdop;                // Horizontal dilution of precision
    uint8_t satellites;        // Number of satellites in use
    uint32_t timestamp;        // Time of fix
    gps_fix_t fix_quality;     // Quality of the GPS fix
} geo_location_t;

/* ---- Point of Interest ---- */
typedef struct {
    geo_location_t location;
    char name[32];
    char description[128];
    uint8_t type;             // 0=general, 1=shelter, 2=medical, 3=water, 4=food
    uint32_t timestamp;
} poi_t;

/* ---- Rescue Zone ---- */
typedef struct {
    geo_location_t center;
    float radius;             // Meters
    char name[32];
    char description[128];
    uint8_t status;           // 0=proposed, 1=active, 2=clearing, 3=cleared
    uint32_t timestamp;
} rescue_zone_t;

/* ---- Map Tile ---- */
typedef struct {
    uint16_t tile_id;         // X/Y grid identifier
    uint16_t zoom_level;      // Zoom level
    uint32_t data_size;       // Size of map data
    uint32_t timestamp;       // Last update time
    uint8_t* data;            // Compressed map data
} map_tile_t;

/* ---- Message Structure ---- */
typedef struct {
    message_type_t type;
    node_id_t source;
    node_id_t destination;
    msg_id_t id;
    hop_count_t hop_count;
    uint8_t ttl;
    message_priority_t priority;
    uint32_t timestamp;
    uint16_t payload_len;
    uint8_t payload[MAX_MSG_SIZE];
    uint8_t mac[MAC_SIZE];  // Message Authentication Code
} message_t;

/* ---- Node Information ---- */
typedef struct {
    node_id_t id;
    uint32_t last_seen;
    signal_strength_t signal_strength;
    battery_level_t battery_level;
    hop_count_t hop_count;
    geo_location_t location;
    bool is_relay;
    uint8_t rf_channel;
    
    // RF triangulation data
    uint32_t last_ping_time;
    uint32_t rtt;             // Round trip time in ms
    float distance_estimate;  // Estimated distance in meters
} node_info_t;

/* ---- Routing Table Entry ---- */
typedef struct {
    node_id_t destination;
    node_id_t next_hop;
    hop_count_t hop_count;
    uint32_t last_updated;
    signal_strength_t signal_quality;
} route_entry_t;

/* ---- Message Queue Entry ---- */
typedef struct msg_queue_entry {
    message_t message;
    uint32_t next_retry;
    uint8_t retry_count;
    bool awaiting_ack;
    struct msg_queue_entry* next;
} msg_queue_entry_t;

/* Forward declarations of structures to resolve circular dependencies */
typedef struct hal_struct hal_t;
typedef struct map_system_struct map_system_t;
typedef struct system_state_struct system_state_t;

/* ---- Map and Location System State ---- */
struct map_system_struct {
    map_tile_t map_tiles[MAX_MAP_TILES];
    uint8_t map_tile_count;
    
    poi_t points_of_interest[MAX_POI];
    uint8_t poi_count;
    
    rescue_zone_t rescue_zones[MAX_ZONES];
    uint8_t zone_count;
    
    geo_location_t last_known_locations[MAX_NODES];
    uint32_t location_timestamps[MAX_NODES];
    
    uint32_t last_location_broadcast;
};

/* ---- Hardware Abstraction Layer (HAL) ---- */
struct hal_struct {
    void (*init_radio)(void);
    bool (*send_packet)(uint8_t* data, uint16_t len, uint8_t power_level);
    uint16_t (*receive_packet)(uint8_t* buffer, uint16_t max_len);
    void (*sleep_ms)(uint32_t ms);
    uint32_t (*get_time_ms)(void);
    battery_level_t (*get_battery_level)(void);
    void (*set_led)(bool state);
    bool (*get_button_state)(uint8_t button_id);
    void (*enter_low_power_mode)(void);
    void (*exit_low_power_mode)(void);
    void (*set_rf_channel)(uint8_t channel);
    signal_strength_t (*get_last_rssi)(void);
    int16_t (*get_temperature)(void);
    uint8_t (*get_random_byte)(void);
    bool (*get_gps_data)(geo_location_t* location);
    
    // Hardware specific data
    uint8_t* receiver_buffer;
    uint16_t receiver_len;
    int hardware_type;  // 0=simulation, 1=Raspberry Pi, etc.
};

/* ---- System State ---- */
struct system_state_struct {
    hal_t hal;
    node_id_t node_id;
    uint8_t encryption_key[ENCRYPTION_KEY_SIZE];
    geo_location_t location;
    node_info_t known_nodes[MAX_NODES];
    uint8_t node_count;
    route_entry_t routing_table[MAX_ROUTES];
    uint8_t route_count;
    msg_queue_entry_t* outbound_queue;
    uint8_t outbound_queue_size;
    uint32_t last_beacon_time;
    battery_level_t battery_level;
    uint8_t current_rf_channel;
    bool low_power_mode;
    msg_id_t next_msg_id;
    
    // Map and location system
    map_system_t map_system;
};

#endif /* MESH_COMMON_H */