/**
 * Resilient Mesh Emergency Communication System
 * Enhanced with GPS, Offline Maps, and RF Triangulation
 * For Emergency Response Operations
 * 
 * hal.c - Hardware Abstraction Layer common functions
 */

#include "../include/hal.h"
#include "../include/utils.h"

/**
 * Check if running on a Raspberry Pi
 */
bool is_raspberry_pi(void) {
    struct utsname system_info;
    
    if (uname(&system_info) != 0) {
        return false;
    }
    
    // Check machine name (typically "armv7l" for Raspberry Pi)
    if (strstr(system_info.machine, "arm") != NULL) {
        // Further check by looking for Raspberry Pi specific files
        if (access("/proc/device-tree/model", F_OK) != -1) {
            FILE* model_file = fopen("/proc/device-tree/model", "r");
            if (model_file) {
                char model[256];
                size_t bytes_read = fread(model, 1, sizeof(model) - 1, model_file);
                fclose(model_file);
                
                if (bytes_read > 0) {
                    model[bytes_read] = '\0';
                    if (strstr(model, "Raspberry Pi") != NULL) {
                        return true;
                    }
                }
            }
        }
    }
    
    return false;
}