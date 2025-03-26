/**
 * Resilient Mesh Emergency Communication System
 * Enhanced with GPS, Offline Maps, and RF Triangulation
 * For Emergency Response Operations
 * 
 * hal.h - Hardware Abstraction Layer interface
 */

#ifndef MESH_HAL_H
#define MESH_HAL_H

#include "common.h"

/* Function prototypes for creating HAL instances */
hal_t create_bt_hal(void);

#ifdef USE_RASPBERRY_PI
hal_t create_raspberry_pi_hal(void);
extern int pi_gps_fd;  /* Add this for Raspberry Pi GPS file descriptor */
#endif

/* External declaration for Bluetooth specific data */
extern int bt_port;
extern int bt_socket;

/* Function prototype for checking if running on Raspberry Pi */
bool is_raspberry_pi(void);

#endif /* MESH_HAL_H */