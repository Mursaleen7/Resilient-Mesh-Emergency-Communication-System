/**
 * Resilient Mesh Emergency Communication System
 * Enhanced with GPS, Offline Maps, and RF Triangulation
 * For Emergency Response Operations
 * 
 * message.h - Message handling functions
 */

#ifndef MESH_MESSAGE_H
#define MESH_MESSAGE_H

#include "common.h"

/* ---- Message Handling Functions ---- */
bool send_message(system_state_t* state, node_id_t destination, 
                  message_type_t type, const uint8_t* payload, 
                  uint16_t payload_len, message_priority_t priority);
void forward_message(system_state_t* state, message_t* msg);
void process_message(system_state_t* state, message_t* msg);
void enqueue_message(system_state_t* state, message_t* msg);
void process_queue(system_state_t* state);
void acknowledge_message(system_state_t* state, message_t* msg);
void remove_from_queue(system_state_t* state, msg_id_t id, node_id_t destination);

/* ---- Encryption & Security Functions ---- */
void encrypt_message(system_state_t* state, message_t* msg);
bool decrypt_message(system_state_t* state, message_t* msg);
void calculate_mac(system_state_t* state, message_t* msg);
bool verify_mac(system_state_t* state, message_t* msg);

/* Get the name of a message type for debugging */
const char* get_message_type_name(message_type_t type);

#endif /* MESH_MESSAGE_H */