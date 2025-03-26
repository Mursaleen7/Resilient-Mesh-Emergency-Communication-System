/**
 * Resilient Mesh Emergency Communication System
 * Enhanced with GPS, Offline Maps, and RF Triangulation
 * For Emergency Response Operations
 * 
 * hal_raspberry_pi.c - Raspberry Pi HAL implementation
 */

#include "../include/hal.h"
#include "../include/utils.h"

#ifdef USE_RASPBERRY_PI

#include <wiringPi.h>    /* GPIO control for Raspberry Pi */
#include <wiringSerial.h> /* Serial communication for GPS */
#include <wiringPiSPI.h> /* SPI for RF module */

/* ---- Raspberry Pi Specific Data ---- */
int pi_gps_fd = -1;         // GPS serial port file descriptor - changed from static for main.c access
static uint8_t pi_rf_last_rssi = 0; // Last RSSI reading from RF module
static uint8_t pi_rf_rx_buffer[MAX_MSG_SIZE]; // RF receive buffer
static uint16_t pi_rf_rx_buffer_len = 0; 

/* Function prototypes for Raspberry Pi HAL */
static void pi_init_radio(void);
static bool pi_send_packet(uint8_t* data, uint16_t len, uint8_t power_level);
static uint16_t pi_receive_packet(uint8_t* buffer, uint16_t max_len);
static void pi_sleep_ms(uint32_t ms);
static uint32_t pi_get_time_ms(void);
static battery_level_t pi_get_battery_level(void);
static void pi_set_led(bool state);
static bool pi_get_button_state(uint8_t button_id);
static void pi_enter_low_power_mode(void);
static void pi_exit_low_power_mode(void);
static void pi_set_rf_channel(uint8_t channel);
static signal_strength_t pi_get_last_rssi(void);
static int16_t pi_get_temperature(void);
static uint8_t pi_get_random_byte(void);
static bool pi_get_gps_data(geo_location_t* location);
static void pi_process_gps(void);
static bool pi_parse_nmea(const char* sentence, geo_location_t* location);

/**
 * Create HAL structure for Raspberry Pi functions
 */
hal_t create_raspberry_pi_hal(void) {
    hal_t hal;
    
    hal.init_radio = pi_init_radio;
    hal.send_packet = pi_send_packet;
    hal.receive_packet = pi_receive_packet;
    hal.sleep_ms = pi_sleep_ms;
    hal.get_time_ms = pi_get_time_ms;
    hal.get_battery_level = pi_get_battery_level;
    hal.set_led = pi_set_led;
    hal.get_button_state = pi_get_button_state;
    hal.enter_low_power_mode = pi_enter_low_power_mode;
    hal.exit_low_power_mode = pi_exit_low_power_mode;
    hal.set_rf_channel = pi_set_rf_channel;
    hal.get_last_rssi = pi_get_last_rssi;
    hal.get_temperature = pi_get_temperature;
    hal.get_random_byte = pi_get_random_byte;
    hal.get_gps_data = pi_get_gps_data;
    
    // Allocate receive buffer
    hal.receiver_buffer = malloc(MAX_MSG_SIZE);
    hal.receiver_len = 0;
    hal.hardware_type = 1; // Raspberry Pi
    
    return hal;
}

/**
 * Initialize RF module on Raspberry Pi
 */
static void pi_init_radio(void) {
    print_status_message("Initializing Raspberry Pi RF module...", 0);
    
    // Initialize WiringPi library
    if (wiringPiSetup() == -1) {
        print_status_message("Failed to initialize WiringPi", 3);
        return;
    }
    
    // Setup GPIO pins
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT);
    digitalWrite(LED_PIN, LOW);
    
    // Initialize RF module via SPI
    if (wiringPiSPISetup(PI_RF_SPI_CHANNEL, PI_RF_SPI_SPEED) == -1) {
        print_status_message("Failed to initialize SPI for RF module", 3);
        return;
    }
    
    // Setup interrupt pin for RF module
    pinMode(RF_IRQ_PIN, INPUT);
    
    // Initialize GPS serial port
    pi_gps_fd = serialOpen(PI_GPS_SERIAL_DEVICE, PI_GPS_BAUD_RATE);
    if (pi_gps_fd == -1) {
        print_status_message("Failed to open GPS serial port. Running without GPS.", 2);
    } else {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "GPS initialized on %s", PI_GPS_SERIAL_DEVICE);
        print_status_message(buffer, 1);
    }
    
    print_status_message("Raspberry Pi hardware initialized", 1);
}

/**
 * Send a packet via the RF module
 */
static bool pi_send_packet(uint8_t* data, uint16_t len, uint8_t power_level) {
    // Set power level on RF module first
    uint8_t power_cmd[2] = {0x04, power_level}; // Example command - adjust for actual RF module
    wiringPiSPIDataRW(PI_RF_SPI_CHANNEL, power_cmd, 2);
    
    // Prepare buffer with length + data
    uint8_t tx_buffer[MAX_MSG_SIZE + 2];
    tx_buffer[0] = len & 0xFF;
    tx_buffer[1] = (len >> 8) & 0xFF;
    memcpy(tx_buffer + 2, data, len);
    
    // Send command to RF module to transmit data
    uint8_t tx_cmd = 0x01; // Example command - adjust for actual RF module
    wiringPiSPIDataRW(PI_RF_SPI_CHANNEL, &tx_cmd, 1);
    
    // Send data
    if (wiringPiSPIDataRW(PI_RF_SPI_CHANNEL, tx_buffer, len + 2) == -1) {
        print_status_message("SPI write failed", 3);
        return false;
    }
    
    // Flash LED to indicate transmission
    digitalWrite(LED_PIN, HIGH);
    delay(1);
    digitalWrite(LED_PIN, LOW);
    
    return true;
}

/**
 * Receive a packet from the RF module
 */
static uint16_t pi_receive_packet(uint8_t* buffer, uint16_t max_len) {
    // Check if we have data in the buffer
    if (pi_rf_rx_buffer_len > 0 && pi_rf_rx_buffer_len <= max_len) {
        memcpy(buffer, pi_rf_rx_buffer, pi_rf_rx_buffer_len);
        uint16_t len = pi_rf_rx_buffer_len;
        pi_rf_rx_buffer_len = 0;
        return len;
    }
    
    // Check if RF module has data (IRQ pin should be high)
    if (digitalRead(RF_IRQ_PIN) == HIGH) {
        // Send command to RF module to get received data
        uint8_t rx_cmd = 0x02; // Example command - adjust for actual RF module
        wiringPiSPIDataRW(PI_RF_SPI_CHANNEL, &rx_cmd, 1);
        
        // Read length first
        uint8_t len_bytes[2];
        if (wiringPiSPIDataRW(PI_RF_SPI_CHANNEL, len_bytes, 2) == -1) {
            print_status_message("SPI read failed", 3);
            return 0;
        }
        
        uint16_t data_len = len_bytes[0] | (len_bytes[1] << 8);
        if (data_len > max_len) {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Received packet too large (%d bytes)", data_len);
            print_status_message(buffer, 3);
            return 0;
        }
        
        // Read actual data
        if (wiringPiSPIDataRW(PI_RF_SPI_CHANNEL, buffer, data_len) == -1) {
            print_status_message("SPI read failed", 3);
            return 0;
        }
        
        // Read RSSI
        uint8_t rssi_cmd = 0x03; // Example command - adjust for actual RF module
        wiringPiSPIDataRW(PI_RF_SPI_CHANNEL, &rssi_cmd, 1);
        
        uint8_t rssi_value;
        wiringPiSPIDataRW(PI_RF_SPI_CHANNEL, &rssi_value, 1);
        pi_rf_last_rssi = rssi_value;
        
        // Flash LED to indicate reception
        digitalWrite(LED_PIN, HIGH);
        delay(1);
        digitalWrite(LED_PIN, LOW);
        
        return data_len;
    }
    
    return 0; // No data available
}

/**
 * Sleep for specified milliseconds
 */
static void pi_sleep_ms(uint32_t ms) {
    delay(ms); // WiringPi delay function
}

/**
 * Get current time in milliseconds
 */
static uint32_t pi_get_time_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}

/**
 * Get battery level from Pi's power management
 */
static battery_level_t pi_get_battery_level(void) {
    // On Raspberry Pi, we might read this from a connected battery module
    // For this example, we'll check if a UPS/battery HAT is connected
    // by looking for its I2C address
    
    FILE *fp;
    char line[128];
    int battery_level = 100; // Default to 100% if no battery system detected
    
    // Try to read battery level from connected UPS/battery HAT
    fp = popen("i2cdetect -y 1 | grep '0x36\\|0x48' | wc -l", "r");
    if (fp != NULL) {
        if (fgets(line, sizeof(line), fp) != NULL) {
            if (atoi(line) > 0) {
                // Battery module detected, try to read its value
                // This would need to be adjusted for the specific battery HAT
                fp = popen("python3 -c \"import smbus; bus = smbus.SMBus(1); print(bus.read_byte_data(0x36, 0x04))\"", "r");
                if (fp != NULL) {
                    if (fgets(line, sizeof(line), fp) != NULL) {
                        battery_level = atoi(line);
                        if (battery_level < 0 || battery_level > 100) {
                            battery_level = 100; // Use default if reading is invalid
                        }
                    }
                    pclose(fp);
                }
            }
        }
        pclose(fp);
    }
    
    return (battery_level_t)battery_level;
}

/**
 * Set the status LED
 */
static void pi_set_led(bool state) {
    digitalWrite(LED_PIN, state ? HIGH : LOW);
}

/**
 * Get button state
 */
static bool pi_get_button_state(uint8_t button_id) {
    if (button_id == 0) {
        return digitalRead(BUTTON_PIN) == HIGH;
    }
    return false;
}

/**
 * Enter low power mode
 */
static void pi_enter_low_power_mode(void) {
    print_status_message("Entering low power mode", 2);
    // Turn off non-essential peripherals
    
    // For a real implementation, we would:
    // - Reduce CPU frequency
    // - Turn off HDMI
    // - Disable USB ports
    // - Reduce RF module power
    
    system("vcgencmd display_power 0"); // Turn off HDMI
}

/**
 * Exit low power mode
 */
static void pi_exit_low_power_mode(void) {
    print_status_message("Exiting low power mode", 1);
    // Restore normal operation
    system("vcgencmd display_power 1"); // Turn on HDMI
}

/**
 * Set RF channel
 */
static void pi_set_rf_channel(uint8_t channel) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "Switching to channel %d", channel);
    print_status_message(buffer, 0);
    
    // Send channel change command to RF module
    uint8_t channel_cmd[2] = {0x05, channel}; // Example command - adjust for actual RF module
    wiringPiSPIDataRW(PI_RF_SPI_CHANNEL, channel_cmd, 2);
}

/**
 * Get last RSSI value
 */
static signal_strength_t pi_get_last_rssi(void) {
    return pi_rf_last_rssi;
}

/**
 * Get temperature from Pi's internal sensor
 */
static int16_t pi_get_temperature(void) {
    FILE *fp;
    char line[128];
    int temp = 20 * 10; // Default 20°C
    
    // Read CPU temperature
    fp = popen("cat /sys/class/thermal/thermal_zone0/temp", "r");
    if (fp != NULL) {
        if (fgets(line, sizeof(line), fp) != NULL) {
            temp = atoi(line) / 100; // Convert from millidegrees to tenths of degrees
        }
        pclose(fp);
    }
    
    return (int16_t)temp;
}

/**
 * Get random byte from system entropy
 */
static uint8_t pi_get_random_byte(void) {
    static FILE* urandom = NULL;
    uint8_t byte = 0;
    
    if (urandom == NULL) {
        urandom = fopen("/dev/urandom", "r");
        if (urandom == NULL) {
            return rand() & 0xFF; // Fallback if /dev/urandom is not available
        }
    }
    
    fread(&byte, 1, 1, urandom);
    return byte;
}

/**
 * Process incoming GPS data
 */
static void pi_process_gps(void) {
    static char buffer[256];
    static int pos = 0;
    
    // Check if GPS is initialized
    if (pi_gps_fd == -1) {
        return;
    }
    
    // Read available data
    while (serialDataAvail(pi_gps_fd) > 0) {
        char c = serialGetchar(pi_gps_fd);
        
        // Store character
        if (pos < sizeof(buffer) - 1) {
            buffer[pos++] = c;
        }
        
        // Process complete sentences
        if (c == '\n') {
            buffer[pos] = '\0';
            pos = 0;
            
            // Parse NMEA sentence
            if (strncmp(buffer, "$GP", 3) == 0 || strncmp(buffer, "$GN", 3) == 0) {
                if (DEBUG) printf("GPS data: %s", buffer);
            }
        }
    }
}

/**
 * Parse NMEA sentence to extract location
 */
static bool pi_parse_nmea(const char* sentence, geo_location_t* location) {
    // Check for GGA sentence (Global Positioning System Fix Data)
    if (strstr(sentence, "GGA") != NULL) {
        char lat_str[12], lon_str[12], alt_str[12];
        char lat_dir, lon_dir;
        int satellites;
        float hdop;
        int fix_quality;
        
        // Example of parsing GGA sentence
        // $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
        if (sscanf(sentence, "$G?GGA,%*[^,],%11[^,],%c,%11[^,],%c,%d,%d,%f,%11[^,],", 
                  lat_str, &lat_dir, lon_str, &lon_dir, &fix_quality, 
                  &satellites, &hdop, alt_str) >= 8) {
            
            double lat = atof(lat_str);
            double lon = atof(lon_str);
            
            // Convert NMEA format (DDMM.MMMM) to decimal degrees
            double lat_deg = (int)(lat / 100);
            double lon_deg = (int)(lon / 100);
            
            lat = lat_deg + (lat - lat_deg * 100) / 60.0;
            lon = lon_deg + (lon - lon_deg * 100) / 60.0;
            
            // Apply direction
            if (lat_dir == 'S') lat = -lat;
            if (lon_dir == 'W') lon = -lon;
            
            // Update location structure
            location->latitude = lat;
            location->longitude = lon;
            location->altitude = atof(alt_str);
            location->satellites = satellites;
            location->hdop = hdop;
            location->timestamp = pi_get_time_ms();
            
            // Convert fix quality
            switch (fix_quality) {
                case 1: location->fix_quality = GPS_FIX_GPS; break;
                case 2: location->fix_quality = GPS_FIX_DGPS; break;
                default: location->fix_quality = GPS_FIX_NONE; break;
            }
            
            return true;
        }
    }
    // Check for RMC sentence (Recommended Minimum Navigation Information)
    else if (strstr(sentence, "RMC") != NULL) {
        char lat_str[12], lon_str[12];
        char lat_dir, lon_dir, status;
        float speed, course;
        
        // Example of parsing RMC sentence
        // $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
        if (sscanf(sentence, "$G?RMC,%*[^,],%c,%11[^,],%c,%11[^,],%c,%f,%f,", 
                  &status, lat_str, &lat_dir, lon_str, &lon_dir, &speed, &course) >= 7) {
            
            // Skip if status is not 'A' (data valid)
            if (status != 'A') {
                return false;
            }
            
            double lat = atof(lat_str);
            double lon = atof(lon_str);
            
            // Convert NMEA format (DDMM.MMMM) to decimal degrees
            double lat_deg = (int)(lat / 100);
            double lon_deg = (int)(lon / 100);
            
            lat = lat_deg + (lat - lat_deg * 100) / 60.0;
            lon = lon_deg + (lon - lon_deg * 100) / 60.0;
            
            // Apply direction
            if (lat_dir == 'S') lat = -lat;
            if (lon_dir == 'W') lon = -lon;
            
            // Update location structure
            location->latitude = lat;
            location->longitude = lon;
            location->speed = speed * 0.514444f; // Convert knots to m/s
            location->course = course;
            location->timestamp = pi_get_time_ms();
            location->fix_quality = GPS_FIX_GPS; // Assume standard GPS
            
            return true;
        }
    }
    
    return false;
}

/**
 * Get GPS data from connected module
 */
static bool pi_get_gps_data(geo_location_t* location) {
    static geo_location_t last_valid;
    static uint32_t last_update = 0;
    
    // Check if GPS is initialized
    if (pi_gps_fd == -1) {
        return false;
    }
    
    // Process any pending GPS data
    pi_process_gps();
    
    // Read and parse sentences
    char buffer[256];
    bool got_valid_data = false;
    uint32_t start_time = pi_get_time_ms();
    
    // Try for up to 100ms to get fresh GPS data
    while (pi_get_time_ms() - start_time < 100 && !got_valid_data) {
        if (serialDataAvail(pi_gps_fd) > 0) {
            int i = 0;
            char c;
            
            // Read until newline or buffer full
            while (i < sizeof(buffer) - 1 && (c = serialGetchar(pi_gps_fd)) != '\n') {
                buffer[i++] = c;
                // Short timeout for reading sentence
                if (pi_get_time_ms() - start_time > 150) break;
            }
            
            if (c == '\n') {
                buffer[i++] = c;
                buffer[i] = '\0';
                
                // Parse NMEA sentence
                if (pi_parse_nmea(buffer, location)) {
                    got_valid_data = true;
                    last_valid = *location;
                    last_update = pi_get_time_ms();
                }
            }
        }
        
        // Small delay to avoid busy waiting
        pi_sleep_ms(1);
    }
    
    // If we didn't get new data but have recent data, use that
    if (!got_valid_data && pi_get_time_ms() - last_update < 10000) {
        *location = last_valid;
        return true;
    }
    
    return got_valid_data;
}

#endif /* USE_RASPBERRY_PI */