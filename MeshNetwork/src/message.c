/**
 * Resilient Mesh Emergency Communication System
 * Enhanced with GPS, Offline Maps, and RF Triangulation
 * For Emergency Response Operations
 * 
 * message.c - Message handling implementation
 */

#include "../include/message.h"
#include "../include/network.h"
#include "../include/utils.h"
#include "../include/location.h" /* Add this for location function declarations */
#include "../include/map.h"

/**
 * Get the name of a message type for debugging
 */
const char* get_message_type_name(message_type_t type) {
    switch (type) {
        case MSG_BEACON: return "BEACON";
        case MSG_DATA: return "DATA";
        case MSG_ACK: return "ACK";
        case MSG_ROUTE_REQUEST: return "ROUTE_REQUEST";
        case MSG_ROUTE_RESPONSE: return "ROUTE_RESPONSE";
        case MSG_ALERT: return "ALERT";
        case MSG_PING: return "PING";
        case MSG_PONG: return "PONG";
        case MSG_LOCATION: return "LOCATION";
        case MSG_MAP_REQUEST: return "MAP_REQUEST";
        case MSG_MAP_DATA: return "MAP_DATA";
        case MSG_POI: return "POI";
        case MSG_ZONE: return "ZONE";
        default: return "UNKNOWN";
    }
}

/**
 * Send a new message to a destination
 */
bool send_message(system_state_t* state, node_id_t destination, 
                 message_type_t type, const uint8_t* payload, 
                 uint16_t payload_len, message_priority_t priority) {
    if (payload_len > MAX_MSG_SIZE) {
        if (DEBUG) print_status_message("Payload too large", 3);
        return false; // Payload too large
    }
    
    message_t msg;
    memset(&msg, 0, sizeof(message_t));
    
    msg.type = type;
    msg.source = state->node_id;
    msg.destination = destination;
    msg.id = state->next_msg_id++;
    msg.hop_count = 0;
    msg.ttl = MAX_HOPS;
    msg.priority = priority;
    msg.timestamp = state->hal.get_time_ms();
    msg.payload_len = payload_len;
    
    if (payload_len > 0 && payload != NULL) {
        memcpy(msg.payload, payload, payload_len);
    }
    
    // Encrypt message if it's a data or alert message
    if (type == MSG_DATA || type == MSG_ALERT) {
        encrypt_message(state, &msg);
    }
    
    // Calculate MAC
    calculate_mac(state, &msg);
    
    // Determine power level based on priority
    uint8_t power_level;
    switch (priority) {
        case PRIORITY_EMERGENCY:
            power_level = 3; // Maximum power
            break;
        case PRIORITY_HIGH:
            power_level = 2; // High power
            break;
        case PRIORITY_NORMAL:
            power_level = 1; // Normal power
            break;
        case PRIORITY_LOW:
        default:
            power_level = 0; // Minimum power
            break;
    }
    
    if (DEBUG) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "Sending message type %s to node %d", 
                get_message_type_name(type), destination);
        print_status_message(buffer, 0);
    }
    
    // Try to send immediately if we have a direct route or it's a broadcast
    if (destination == 0xFFFF || find_node(state, destination) != NULL) {
        // Direct send
        bool sent = state->hal.send_packet((uint8_t*)&msg, sizeof(message_t), power_level);
        if (sent) {
            if (DEBUG) print_status_message("Message sent directly", 1);
            // Queue for ACK if it's a data or alert message
            if ((type == MSG_DATA || type == MSG_ALERT) && destination != 0xFFFF) {
                enqueue_message(state, &msg);
            }
            return true;
        }
    } else {
        // Try using routing table
        route_entry_t* route = find_route(state, destination);
        if (route) {
            // Send to next hop
            bool sent = state->hal.send_packet((uint8_t*)&msg, sizeof(message_t), power_level);
            if (sent) {
                if (DEBUG) print_status_message("Message sent via route", 1);
                // Queue for ACK if it's a data or alert message
                if (type == MSG_DATA || type == MSG_ALERT) {
                    enqueue_message(state, &msg);
                }
                return true;
            }
        }
    }
    
    // If we don't have a route or sending failed, initiate route discovery and queue
    if (destination != 0xFFFF) {  // Don't need routes for broadcast
        if (DEBUG) print_status_message("No route available, initiating route discovery", 2);
        send_route_request(state, destination);
    }
    
    enqueue_message(state, &msg);
    
    return true;
}

/**
 * Forward a message to its next hop
 */
void forward_message(system_state_t* state, message_t* msg) {
    // Don't forward if TTL is 0 or we're the source
    if (msg->ttl == 0 || msg->source == state->node_id) {
        if (DEBUG) {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), 
                    "Not forwarding message (TTL: %d, source: %d)", 
                    msg->ttl, msg->source);
            print_status_message(buffer, 2);
        }
        return;
    }
    
    // Don't forward if we've already seen this message (loop prevention)
    for (msg_queue_entry_t* entry = state->outbound_queue; entry != NULL; entry = entry->next) {
        if (entry->message.source == msg->source && 
            entry->message.id == msg->id) {
            if (DEBUG) print_status_message("Not forwarding message (already seen)", 2);
            return; // Already seen this message
        }
    }
    
    // Decrement TTL
    msg->ttl--;
    msg->hop_count++;
    
    // Recalculate MAC
    calculate_mac(state, msg);
    
    if (DEBUG) {
        char buffer[256];
        snprintf(buffer, sizeof(buffer), 
                "Forwarding message: %s from %d to %d (hop count: %d, TTL: %d)", 
                get_message_type_name(msg->type), msg->source, msg->destination, 
                msg->hop_count, msg->ttl);
        print_status_message(buffer, 0);
    }
    
    // If it's a broadcast, just forward it
    if (msg->destination == 0xFFFF) {
        // Use lower power for broadcasts to avoid congestion
        if (state->hal.send_packet((uint8_t*)msg, sizeof(message_t), 1)) {
            if (DEBUG) print_status_message("Broadcast message forwarded", 1);
        } else {
            if (DEBUG) print_status_message("Failed to forward broadcast message", 3);
        }
    } else {
        // Look up next hop
        route_entry_t* route = find_route(state, msg->destination);
        if (route) {
            // Send to next hop
            if (state->hal.send_packet((uint8_t*)msg, sizeof(message_t), 1)) {
                if (DEBUG) {
                    char buffer[128];
                    snprintf(buffer, sizeof(buffer), 
                            "Message forwarded to next hop (node %d)", route->next_hop);
                    print_status_message(buffer, 1);
                }
            } else {
                if (DEBUG) print_status_message("Failed to forward message to next hop", 3);
            }
        } else {
            // We don't know how to reach the destination
            // Start route discovery and queue the message
            if (DEBUG) print_status_message("No route for forwarding, initiating route discovery", 2);
            send_route_request(state, msg->destination);
            enqueue_message(state, msg);
        }
    }
}

/**
 * Process a received message
 */
void process_message(system_state_t* state, message_t* msg) {
    // Check if message is valid
    if (msg == NULL) {
        return;
    }
    
    // Update route information based on received message
    signal_strength_t quality = state->hal.get_last_rssi();
    add_or_update_route(state, msg->source, msg->source, 1, quality);
    
    if (DEBUG) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), 
                "Processing message type %s from node %d", 
                get_message_type_name(msg->type), msg->source);
        print_status_message(buffer, 0);
    }
    
    // Process based on message type
    switch (msg->type) {
        case MSG_BEACON:
            if (msg->payload_len >= sizeof(battery_level_t) + sizeof(geo_location_t)) {
                battery_level_t battery;
                geo_location_t location;
                
                uint8_t* p = msg->payload;
                memcpy(&battery, p, sizeof(battery_level_t));
                p += sizeof(battery_level_t);
                
                memcpy(&location, p, sizeof(geo_location_t));
                
                // Update node information
                update_node_info(state, msg->source, quality, battery, msg->hop_count, &location);
                
                // Estimate distance based on signal strength
                node_info_t* node = find_node(state, msg->source);
                if (node) {
                    estimate_node_distance(state, msg->source);
                    
                    // If the node doesn't have a GPS fix, try to estimate its position
                    if (node->location.fix_quality == GPS_FIX_NONE) {
                        triangulate_position(state, msg->source);
                    }
                }
            }
            break;
            
        case MSG_DATA:
        case MSG_ALERT:
            if (msg->destination == state->node_id) {
                // Message is for us
                char buffer[MAX_MSG_SIZE + 64];
                char priority_str[32];
                
                // Set priority color
                switch (msg->priority) {
                    case PRIORITY_EMERGENCY:
                        strcpy(priority_str, TERM_RED);
                        break;
                    case PRIORITY_HIGH:
                        strcpy(priority_str, TERM_YELLOW);
                        break;
                    case PRIORITY_NORMAL:
                        strcpy(priority_str, TERM_GREEN);
                        break;
                    default:
                        strcpy(priority_str, TERM_RESET);
                        break;
                }
                
                snprintf(buffer, sizeof(buffer), "%sMessage from node %d [%s]:%s %s", 
                         priority_str, msg->source, 
                         get_priority_string(msg->priority), 
                         TERM_RESET, msg->payload);
                
                print_header("INCOMING MESSAGE");
                printf("%s\n", buffer);
                print_separator();
                
                // Send acknowledgment
                acknowledge_message(state, msg);
            } else if (msg->destination == 0xFFFF) {
                // Broadcast message
                char buffer[MAX_MSG_SIZE + 64];
                snprintf(buffer, sizeof(buffer), "%sBroadcast from node %d:%s %s", 
                         TERM_CYAN, msg->source, TERM_RESET, msg->payload);
                
                print_header("BROADCAST MESSAGE");
                printf("%s\n", buffer);
                print_separator();
                
                // No need to acknowledge broadcasts
            } else {
                // Message for someone else, forward it
                forward_message(state, msg);
            }
            break;
            
        case MSG_ACK:
            // Got an acknowledgment, remove from queue
            if (DEBUG) {
                char buffer[128];
                snprintf(buffer, sizeof(buffer), 
                        "Received ACK for message %d from node %d", 
                        msg->id, msg->source);
                print_status_message(buffer, 1);
            }
            remove_from_queue(state, msg->id, msg->source);
            break;
            
        case MSG_ROUTE_REQUEST:
            process_route_request(state, msg);
            break;
            
        case MSG_ROUTE_RESPONSE:
            process_route_response(state, msg);
            break;
            
        case MSG_PING:
            if (msg->destination == state->node_id) {
                // Respond with pong
                if (DEBUG) {
                    char buffer[128];
                    snprintf(buffer, sizeof(buffer), 
                            "Received ping from node %d, sending pong", msg->source);
                    print_status_message(buffer, 0);
                }
                
                message_t pong;
                memset(&pong, 0, sizeof(message_t));
                
                pong.type = MSG_PONG;
                pong.source = state->node_id;
                pong.destination = msg->source;
                pong.id = state->next_msg_id++;
                pong.hop_count = 0;
                pong.ttl = MAX_HOPS;
                pong.priority = PRIORITY_LOW;
                pong.timestamp = state->hal.get_time_ms();
                
                // Copy ping payload to pong
                if (msg->payload_len > 0) {
                    memcpy(pong.payload, msg->payload, msg->payload_len);
                    pong.payload_len = msg->payload_len;
                }
                
                // Calculate MAC
                calculate_mac(state, &pong);
                
                // Send pong
                state->hal.send_packet((uint8_t*)&pong, sizeof(message_t), 1);
            } else {
                // Forward ping
                forward_message(state, msg);
            }
            break;
            
        case MSG_PONG:
            if (msg->destination == state->node_id) {
                // Pong response received
                // Calculate round-trip time
                uint32_t rtt = state->hal.get_time_ms() - msg->timestamp;
                
                print_header("PING RESPONSE");
                printf("Received pong from node %d, RTT: %d ms\n", msg->source, rtt);
                print_separator();
                
                // Update route quality based on RTT
                route_entry_t* route = find_route(state, msg->source);
                if (route) {
                    // Lower RTT = better quality (scale is implementation dependent)
                    signal_strength_t new_quality = (rtt < 100) ? 255 : (uint8_t)(25500 / rtt);
                    if (new_quality > route->signal_quality) {
                        route->signal_quality = new_quality;
                        route->last_updated = state->hal.get_time_ms();
                    }
                }
                
                // Update RTT in node info for distance estimation
                node_info_t* node = find_node(state, msg->source);
                if (node) {
                    node->rtt = rtt;
                    node->last_ping_time = state->hal.get_time_ms();
                    
                    // Update distance estimate based on new RTT
                    estimate_node_distance(state, msg->source);
                }
            } else {
                // Forward pong
                forward_message(state, msg);
            }
            break;
            
        case MSG_LOCATION:
            // Process location update
            process_location_message(state, msg);
            break;
            
        case MSG_MAP_REQUEST:
            // Process map tile request
            process_map_request(state, msg);
            break;
            
        case MSG_MAP_DATA:
            // Process received map tile data
            process_map_data(state, msg);
            break;
            
        case MSG_POI:
            // Process point of interest
            process_poi_message(state, msg);
            break;
            
        case MSG_ZONE:
            // Process rescue zone
            process_zone_message(state, msg);
            break;
    }
}

/**
 * Add a message to the outbound queue
 */
void enqueue_message(system_state_t* state, message_t* msg) {
    // Check if queue is full
    if (state->outbound_queue_size >= MAX_QUEUE_SIZE) {
        // Find lowest priority message to replace
        msg_queue_entry_t* lowest_entry = NULL;
        msg_queue_entry_t* prev_entry = NULL;
        msg_queue_entry_t* curr = state->outbound_queue;
        msg_queue_entry_t* prev = NULL;
        
        while (curr != NULL) {
            if (lowest_entry == NULL || curr->message.priority < lowest_entry->message.priority) {
                lowest_entry = curr;
                prev_entry = prev;
            }
            prev = curr;
            curr = curr->next;
        }
        
        // If new message is higher priority than lowest, replace it
        if (lowest_entry != NULL && msg->priority > lowest_entry->message.priority) {
            if (prev_entry == NULL) {
                // Lowest is at head of queue
                state->outbound_queue = lowest_entry->next;
            } else {
                prev_entry->next = lowest_entry->next;
            }
            free(lowest_entry);
            state->outbound_queue_size--;
        } else {
            // Queue is full and new message is not higher priority
            if (DEBUG) print_status_message("Queue full, dropping message", 2);
            return;
        }
    }
    
    // Allocate new queue entry
    msg_queue_entry_t* entry = (msg_queue_entry_t*)malloc(sizeof(msg_queue_entry_t));
    if (entry == NULL) {
        if (DEBUG) print_status_message("Out of memory, dropping message", 3);
        return; // Out of memory
    }
    
    // Copy message
    memcpy(&entry->message, msg, sizeof(message_t));
    entry->next_retry = state->hal.get_time_ms() + RETRY_INTERVAL_MS;
    entry->retry_count = 0;
    entry->awaiting_ack = (msg->type == MSG_DATA || msg->type == MSG_ALERT) && msg->destination != 0xFFFF;
    entry->next = NULL;
    
    if (DEBUG) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), 
                "Enqueuing message type %s to node %d (awaiting ACK: %d)", 
                get_message_type_name(msg->type), msg->destination, entry->awaiting_ack);
        print_status_message(buffer, 0);
    }
    
    // Add to queue
    if (state->outbound_queue == NULL) {
        state->outbound_queue = entry;
    } else {
        // Add based on priority
        if (entry->message.priority > state->outbound_queue->message.priority) {
            // New entry is higher priority than head
            entry->next = state->outbound_queue;
            state->outbound_queue = entry;
        } else {
            // Find insertion point
            msg_queue_entry_t* curr = state->outbound_queue;
            while (curr->next != NULL && curr->next->message.priority >= entry->message.priority) {
                curr = curr->next;
            }
            entry->next = curr->next;
            curr->next = entry;
        }
    }
    
    state->outbound_queue_size++;
}

/**
 * Process the outbound message queue
 */
void process_queue(system_state_t* state) {
    uint32_t current_time = state->hal.get_time_ms();
    
    msg_queue_entry_t* entry = state->outbound_queue;
    msg_queue_entry_t* prev = NULL;
    
    while (entry != NULL) {
        if (current_time >= entry->next_retry) {
            // Time to retry or send for the first time
            bool send_success = false;
            
            if (DEBUG) {
                char buffer[128];
                snprintf(buffer, sizeof(buffer), 
                        "Processing queued message type %s to node %d (retry: %d)", 
                        get_message_type_name(entry->message.type), 
                        entry->message.destination, entry->retry_count);
                print_status_message(buffer, 0);
            }
            
            // Check if we have a route for non-broadcast messages
            if (entry->message.destination == 0xFFFF) {
                // Broadcasts don't need routes
                send_success = true;
            } else {
                route_entry_t* route = find_route(state, entry->message.destination);
                if (route) {
                    send_success = true;
                } else if (find_node(state, entry->message.destination) != NULL) {
                    // Direct node
                    send_success = true;
                }
            }
            
            if (send_success) {
                // Determine power level based on priority and retry count
                uint8_t power_level = entry->message.priority;
                if (entry->retry_count > 0) {
                    power_level++; // Increase power for retries
                }
                if (power_level > 3) {
                    power_level = 3; // Max power level
                }
                
                // Send packet
                send_success = state->hal.send_packet((uint8_t*)&entry->message, 
                                                    sizeof(message_t), power_level);
                
                if (DEBUG) {
                    if (send_success) {
                        print_status_message("Queue message sent successfully", 1);
                    } else {
                        print_status_message("Failed to send queued message", 3);
                    }
                }
            }
            
            if (send_success) {
                if (!entry->awaiting_ack) {
                    // If not waiting for ACK, remove from queue
                    if (DEBUG) print_status_message("Message doesn't need ACK, removing from queue", 0);
                    
                    msg_queue_entry_t* to_remove = entry;
                    
                    if (prev == NULL) {
                        state->outbound_queue = entry->next;
                    } else {
                        prev->next = entry->next;
                    }
                    
                    entry = entry->next;
                    free(to_remove);
                    state->outbound_queue_size--;
                    continue;
                } else {
                    // Update retry info
                    entry->retry_count++;
                    entry->next_retry = current_time + RETRY_INTERVAL_MS * entry->retry_count;
                    
                    // If max retries reached, give up
                    if (entry->retry_count >= 5) {
                        if (DEBUG) print_status_message("Max retries reached, removing message from queue", 2);
                        
                        msg_queue_entry_t* to_remove = entry;
                        
                        if (prev == NULL) {
                            state->outbound_queue = entry->next;
                        } else {
                            prev->next = entry->next;
                        }
                        
                        entry = entry->next;
                        free(to_remove);
                        state->outbound_queue_size--;
                        continue;
                    }
                }
            } else {
                // Sending failed, may need to initiate route discovery
                if (entry->message.destination != 0xFFFF) { // Not for broadcasts
                    if (DEBUG) print_status_message("Sending failed, initiating route discovery", 2);
                    send_route_request(state, entry->message.destination);
                }
                
                // Set retry time
                entry->next_retry = current_time + RETRY_INTERVAL_MS;
            }
        }
        
        prev = entry;
        entry = entry->next;
    }
}

/**
 * Send an acknowledgment for a received message
 */
void acknowledge_message(system_state_t* state, message_t* msg) {
    message_t ack;
    memset(&ack, 0, sizeof(message_t));
    
    ack.type = MSG_ACK;
    ack.source = state->node_id;
    ack.destination = msg->source;
    ack.id = msg->id; // Use same ID as original message
    ack.hop_count = 0;
    ack.ttl = MAX_HOPS;
    ack.priority = PRIORITY_HIGH; // ACKs are high priority
    ack.timestamp = state->hal.get_time_ms();
    
    // No payload for ACKs
    ack.payload_len = 0;
    
    // Calculate MAC
    calculate_mac(state, &ack);
    
    if (DEBUG) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), 
                "Sending ACK for message %d to node %d", msg->id, msg->source);
        print_status_message(buffer, 0);
    }
    
    // Send ACK immediately
    if (state->hal.send_packet((uint8_t*)&ack, sizeof(message_t), 2)) {
        if (DEBUG) print_status_message("ACK sent successfully", 1);
    } else {
        if (DEBUG) print_status_message("Failed to send ACK", 3);
    }
}

/**
 * Remove a message from the queue after receiving an ACK
 */
void remove_from_queue(system_state_t* state, msg_id_t id, node_id_t source) {
    msg_queue_entry_t* entry = state->outbound_queue;
    msg_queue_entry_t* prev = NULL;
    
    while (entry != NULL) {
        if (entry->message.id == id && entry->message.destination == source) {
            // Found the message to remove
            if (DEBUG) {
                char buffer[128];
                snprintf(buffer, sizeof(buffer), 
                        "Removing message %d from queue (ACK received)", id);
                print_status_message(buffer, 1);
            }
            
            if (prev == NULL) {
                state->outbound_queue = entry->next;
            } else {
                prev->next = entry->next;
            }
            
            free(entry);
            state->outbound_queue_size--;
            return;
        }
        
        prev = entry;
        entry = entry->next;
    }
}

/**
 * Calculate MAC for message authentication
 */
void calculate_mac(system_state_t* state, message_t* msg) {
    if (msg == NULL) return;
    
    // Initialize MAC with values derived from key
    for (int i = 0; i < MAC_SIZE; i++) {
        msg->mac[i] = state->encryption_key[i % ENCRYPTION_KEY_SIZE];
    }
    
    // Mix in header fields individually
    
    // Mix in type
    msg->mac[0] ^= msg->type;
    
    // Mix in source and destination
    for (size_t i = 0; i < sizeof(node_id_t); i++) {
        msg->mac[1 + i % (MAC_SIZE-1)] ^= ((uint8_t*)&msg->source)[i];
        msg->mac[2 + i % (MAC_SIZE-2)] ^= ((uint8_t*)&msg->destination)[i];
    }
    
    // Mix in message ID
    msg->mac[3] ^= msg->id;
    
    // Mix in hop count and TTL
    msg->mac[4] ^= msg->hop_count;
    msg->mac[5] ^= msg->ttl;
    
    // Mix in priority
    msg->mac[6] ^= msg->priority;
    
    // Mix in timestamp
    for (size_t i = 0; i < sizeof(uint32_t); i++) {
        msg->mac[7 + i % (MAC_SIZE-7)] ^= ((uint8_t*)&msg->timestamp)[i];
    }
    
    // Mix in payload length
    for (size_t i = 0; i < sizeof(uint16_t); i++) {
        msg->mac[8 + i % (MAC_SIZE-8)] ^= ((uint8_t*)&msg->payload_len)[i];
    }
    
    // Mix in payload
    for (int i = 0; i < msg->payload_len && i < MAX_MSG_SIZE; i++) {
        msg->mac[i % MAC_SIZE] ^= msg->payload[i];
    }
    
    // Final scrambling round
    for (int i = 0; i < MAC_SIZE/2; i++) {
        uint8_t temp = msg->mac[i];
        msg->mac[i] ^= msg->mac[MAC_SIZE-1-i] ^ state->encryption_key[(i*3) % ENCRYPTION_KEY_SIZE];
        msg->mac[MAC_SIZE-1-i] ^= temp ^ state->encryption_key[(i*7) % ENCRYPTION_KEY_SIZE];
    }
}

/**
 * Verify message MAC
 */
bool verify_mac(system_state_t* state, message_t* msg) {
    if (msg == NULL) {
        return false;
    }
    
    // Save original MAC
    uint8_t original_mac[MAC_SIZE];
    memcpy(original_mac, msg->mac, MAC_SIZE);
    
    // Clear MAC field for calculation
    memset(msg->mac, 0, MAC_SIZE);
    
    // Calculate new MAC
    calculate_mac(state, msg);
    
    // Compare MACs
    bool result = true;
    for (int i = 0; i < MAC_SIZE; i++) {
        if (msg->mac[i] != original_mac[i]) {
            result = false;
            break;
        }
    }
    
    // Restore original MAC regardless of result
    memcpy(msg->mac, original_mac, MAC_SIZE);
    
    return result;
}

/**
 * Encrypt a message payload
 * 
 * Note: In a real implementation, this would use a proper encryption algorithm
 * like AES-GCM with a properly managed key.
 */
void encrypt_message(system_state_t* state, message_t* msg) {
    // Simple XOR encryption for demonstration purposes
    // In a real implementation, use a secure encryption algorithm
    for (int i = 0; i < msg->payload_len; i++) {
        msg->payload[i] ^= state->encryption_key[i % ENCRYPTION_KEY_SIZE];
    }
}

/**
 * Decrypt a message payload
 */
bool decrypt_message(system_state_t* state, message_t* msg) {
    // For XOR, encryption and decryption are the same operation
    encrypt_message(state, msg);
    return true;
}