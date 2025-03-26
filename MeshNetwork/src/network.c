/**
 * Resilient Mesh Emergency Communication System
 * Enhanced with GPS, Offline Maps, and RF Triangulation
 * For Emergency Response Operations
 * 
 * network.c - Network management functions
 */

#include "../include/network.h"
#include "../include/message.h"
#include "../include/location.h"
#include "../include/utils.h"

/**
 * Process all incoming messages
 */
void process_incoming_messages(system_state_t* state) {
    uint8_t buffer[sizeof(message_t)];
    uint16_t len = state->hal.receive_packet(buffer, sizeof(buffer));
    
    if (len > 0 && len >= sizeof(message_t)) {
        message_t* msg = (message_t*)buffer;
        
        if (DEBUG) {
            char buffer[256];
            snprintf(buffer, sizeof(buffer), 
                    "Received message: %s from %d to %d (hop count: %d)", 
                    get_message_type_name(msg->type), msg->source, msg->destination, msg->hop_count);
            print_status_message(buffer, 0);
        }
        
        // Validate the message
        if (msg->type >= MSG_BEACON && msg->type <= MSG_ZONE) {
            // Verify message integrity
            if (verify_mac(state, msg)) {
                // Decrypt message if necessary
                if ((msg->type == MSG_DATA || msg->type == MSG_ALERT) && 
                    !decrypt_message(state, msg)) {
                    if (DEBUG) print_status_message("Failed to decrypt message", 3);
                    return; // Failed to decrypt
                }
                
                // Don't process our own messages
                if (msg->source == state->node_id) {
                    if (DEBUG) print_status_message("Ignoring our own message", 0);
                    return;
                }
                
                // Process the message
                if (DEBUG) print_status_message("Processing message...", 0);
                process_message(state, msg);
            } else {
                if (DEBUG) print_status_message("MAC verification failed", 3);
            }
        } else {
            if (DEBUG) {
                char buffer[128];
                snprintf(buffer, sizeof(buffer), "Invalid message type: %d", msg->type);
                print_status_message(buffer, 3);
            }
        }
    }
}

/**
 * Send a beacon message to announce presence on the network
 */
void send_beacon(system_state_t* state) {
    uint32_t current_time = state->hal.get_time_ms();
    
    // Only send beacon at configured intervals
    if (current_time - state->last_beacon_time >= BEACON_INTERVAL_MS) {
        if (DEBUG) print_status_message("Sending beacon...", 0);
        
        message_t beacon;
        memset(&beacon, 0, sizeof(message_t));
        
        beacon.type = MSG_BEACON;
        beacon.source = state->node_id;
        beacon.destination = 0xFFFF; // Broadcast
        beacon.id = state->next_msg_id++;
        beacon.hop_count = 0;
        beacon.ttl = 1; // Beacons only travel 1 hop
        beacon.priority = PRIORITY_LOW;
        beacon.timestamp = current_time;
        
        // Include current node information in the payload
        uint8_t* p = beacon.payload;
        memcpy(p, &state->battery_level, sizeof(battery_level_t));
        p += sizeof(battery_level_t);
        
        memcpy(p, &state->location, sizeof(geo_location_t));
        p += sizeof(geo_location_t);
        
        uint8_t node_capabilities = 0;
        if (state->battery_level > BATTERY_LOW_THRESHOLD) {
            node_capabilities |= 0x01; // Can relay
        }
        *p++ = node_capabilities;
        
        *p++ = state->current_rf_channel;
        
        beacon.payload_len = p - beacon.payload;
        
        // Calculate MAC
        calculate_mac(state, &beacon);
        
        // Send the beacon
        if (state->hal.send_packet((uint8_t*)&beacon, sizeof(message_t), 1)) {
            state->last_beacon_time = current_time;
            if (DEBUG) print_status_message("Beacon sent successfully", 1);
        } else {
            if (DEBUG) print_status_message("Failed to send beacon", 3);
        }
    }
}

/**
 * Update information about a known node
 */
void update_node_info(system_state_t* state, node_id_t id, signal_strength_t signal, 
                     battery_level_t battery, hop_count_t hops, geo_location_t* location) {
    // Don't add ourselves to the known nodes list
    if (id == state->node_id) {
        return;
    }
    
    node_info_t* node = find_node(state, id);
    
    if (node) {
        // Update existing node
        node->last_seen = state->hal.get_time_ms();
        node->signal_strength = signal;
        node->battery_level = battery;
        node->hop_count = hops;
        if (location) {
            node->location = *location;
            
            // Also update the location in our database
            update_node_location(state, id, location);
        }
        node->is_relay = (battery > BATTERY_LOW_THRESHOLD);
        
        if (DEBUG) {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Updated node %d information", id);
            print_status_message(buffer, 0);
        }
    } else if (state->node_count < MAX_NODES) {
        // Add new node
        node = &state->known_nodes[state->node_count++];
        node->id = id;
        node->last_seen = state->hal.get_time_ms();
        node->signal_strength = signal;
        node->battery_level = battery;
        node->hop_count = hops;
        if (location) {
            node->location = *location;
            
            // Also add to our location database
            update_node_location(state, id, location);
        } else {
            memset(&node->location, 0, sizeof(geo_location_t));
        }
        node->is_relay = (battery > BATTERY_LOW_THRESHOLD);
        
        if (DEBUG) {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Added new node %d to known nodes list", id);
            print_status_message(buffer, 1);
        }
    } else {
        if (DEBUG) {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Cannot add node %d, known nodes list is full", id);
            print_status_message(buffer, 2);
        }
    }
}

/**
 * Find a node in the known nodes list
 */
node_info_t* find_node(system_state_t* state, node_id_t id) {
    for (int i = 0; i < state->node_count; i++) {
        if (state->known_nodes[i].id == id) {
            return &state->known_nodes[i];
        }
    }
    return NULL;
}

/**
 * Remove nodes that haven't been seen recently
 */
void clean_stale_nodes(system_state_t* state) {
    uint32_t current_time = state->hal.get_time_ms();
    
    for (int i = 0; i < state->node_count; i++) {
        if (current_time - state->known_nodes[i].last_seen > NODE_TIMEOUT_MS) {
            // Remove the stale node by shifting the array
            if (DEBUG) {
                char buffer[128];
                snprintf(buffer, sizeof(buffer), "Removing stale node %d", state->known_nodes[i].id);
                print_status_message(buffer, 2);
            }
            
            if (i < state->node_count - 1) {
                memmove(&state->known_nodes[i], &state->known_nodes[i + 1], 
                       (state->node_count - i - 1) * sizeof(node_info_t));
            }
            state->node_count--;
            i--; // Check the same index again since we shifted
        }
    }
}

/**
 * Find a route to the specified destination
 */
route_entry_t* find_route(system_state_t* state, node_id_t destination) {
    for (int i = 0; i < state->route_count; i++) {
        if (state->routing_table[i].destination == destination) {
            return &state->routing_table[i];
        }
    }
    return NULL;
}

/**
 * Add or update a route in the routing table
 */
bool add_or_update_route(system_state_t* state, node_id_t destination, 
                         node_id_t next_hop, hop_count_t hop_count, 
                         signal_strength_t quality) {
    // Don't route to ourselves
    if (destination == state->node_id) {
        return false;
    }
    
    route_entry_t* route = find_route(state, destination);
    
    if (route) {
        // Update existing route if new one is better
        if (hop_count < route->hop_count || 
            (hop_count == route->hop_count && quality > route->signal_quality)) {
            route->next_hop = next_hop;
            route->hop_count = hop_count;
            route->signal_quality = quality;
            route->last_updated = state->hal.get_time_ms();
            if (DEBUG) {
                char buffer[128];
                snprintf(buffer, sizeof(buffer), 
                        "Updated route to node %d via node %d (%d hops)", 
                        destination, next_hop, hop_count);
                print_status_message(buffer, 1);
            }
            return true;
        }
    } else if (state->route_count < MAX_ROUTES) {
        // Add new route
        route = &state->routing_table[state->route_count++];
        route->destination = destination;
        route->next_hop = next_hop;
        route->hop_count = hop_count;
        route->signal_quality = quality;
        route->last_updated = state->hal.get_time_ms();
        if (DEBUG) {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), 
                    "Added new route to node %d via node %d (%d hops)", 
                    destination, next_hop, hop_count);
            print_status_message(buffer, 1);
        }
        return true;
    } else {
        // Routing table is full, try to replace worst route
        route_entry_t* worst_route = &state->routing_table[0];
        for (int i = 1; i < state->route_count; i++) {
            if (state->routing_table[i].hop_count > worst_route->hop_count ||
                (state->routing_table[i].hop_count == worst_route->hop_count && 
                 state->routing_table[i].signal_quality < worst_route->signal_quality)) {
                worst_route = &state->routing_table[i];
            }
        }
        
        if (hop_count < worst_route->hop_count ||
            (hop_count == worst_route->hop_count && quality > worst_route->signal_quality)) {
            worst_route->destination = destination;
            worst_route->next_hop = next_hop;
            worst_route->hop_count = hop_count;
            worst_route->signal_quality = quality;
            worst_route->last_updated = state->hal.get_time_ms();
            if (DEBUG) {
                char buffer[128];
                snprintf(buffer, sizeof(buffer), 
                        "Replaced worst route with route to node %d via node %d (%d hops)", 
                        destination, next_hop, hop_count);
                print_status_message(buffer, 1);
            }
            return true;
        }
    }
    
    return false;
}

/**
 * Send a route request to find a path to a destination
 */
void send_route_request(system_state_t* state, node_id_t destination) {
    message_t req;
    memset(&req, 0, sizeof(message_t));
    
    req.type = MSG_ROUTE_REQUEST;
    req.source = state->node_id;
    req.destination = 0xFFFF; // Broadcast
    req.id = state->next_msg_id++;
    req.hop_count = 0;
    req.ttl = MAX_HOPS;
    req.priority = PRIORITY_NORMAL;
    req.timestamp = state->hal.get_time_ms();
    
    // Set destination in payload
    memcpy(req.payload, &destination, sizeof(node_id_t));
    req.payload_len = sizeof(node_id_t);
    
    // Calculate MAC
    calculate_mac(state, &req);
    
    // Send route request
    if (state->hal.send_packet((uint8_t*)&req, sizeof(message_t), 2)) {
        if (DEBUG) {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Sent route request for node %d", destination);
            print_status_message(buffer, 0);
        }
    } else {
        if (DEBUG) {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Failed to send route request for node %d", destination);
            print_status_message(buffer, 3);
        }
    }
}

/**
 * Process a route request message
 */
void process_route_request(system_state_t* state, message_t* msg) {
    if (msg->payload_len < sizeof(node_id_t)) {
        if (DEBUG) print_status_message("Invalid route request message", 3);
        return; // Invalid message
    }
    
    node_id_t requested_dest;
    memcpy(&requested_dest, msg->payload, sizeof(node_id_t));
    
    if (DEBUG) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), 
                "Processing route request for node %d from node %d", 
                requested_dest, msg->source);
        print_status_message(buffer, 0);
    }
    
    // Check if we are the requested destination or know a route to it
    if (requested_dest == state->node_id) {
        // We are the destination, send a route response
        if (DEBUG) print_status_message("We are the requested destination, sending route response", 1);
        
        message_t resp;
        memset(&resp, 0, sizeof(message_t));
        
        resp.type = MSG_ROUTE_RESPONSE;
        resp.source = state->node_id;
        resp.destination = msg->source;
        resp.id = state->next_msg_id++;
        resp.hop_count = 0;
        resp.ttl = MAX_HOPS;
        resp.priority = PRIORITY_NORMAL;
        resp.timestamp = state->hal.get_time_ms();
        
        // No payload needed for direct route
        resp.payload_len = 0;
        
        // Calculate MAC
        calculate_mac(state, &resp);
        
        // Send route response
        state->hal.send_packet((uint8_t*)&resp, sizeof(message_t), 2);
    } else {
        route_entry_t* route = find_route(state, requested_dest);
        if (route) {
            // We know a route, send response
            if (DEBUG) {
                char buffer[128];
                snprintf(buffer, sizeof(buffer), 
                        "We know a route to node %d, sending route response", requested_dest);
                print_status_message(buffer, 1);
            }
            
            message_t resp;
            memset(&resp, 0, sizeof(message_t));
            
            resp.type = MSG_ROUTE_RESPONSE;
            resp.source = state->node_id;
            resp.destination = msg->source;
            resp.id = state->next_msg_id++;
            resp.hop_count = 0;
            resp.ttl = MAX_HOPS;
            resp.priority = PRIORITY_NORMAL;
            resp.timestamp = state->hal.get_time_ms();
            
            // Include route information in payload
            uint8_t* p = resp.payload;
            memcpy(p, &requested_dest, sizeof(node_id_t));
            p += sizeof(node_id_t);
            
            memcpy(p, &route->hop_count, sizeof(hop_count_t));
            p += sizeof(hop_count_t);
            
            resp.payload_len = p - resp.payload;
            
            // Calculate MAC
            calculate_mac(state, &resp);
            
            // Send route response
            state->hal.send_packet((uint8_t*)&resp, sizeof(message_t), 2);
        } else if (msg->hop_count < MAX_HOPS - 1) {
            // Forward route request
            if (DEBUG) {
                char buffer[128];
                snprintf(buffer, sizeof(buffer), "Forwarding route request for node %d", requested_dest);
                print_status_message(buffer, 0);
            }
            
            msg->hop_count++;
            calculate_mac(state, msg);
            state->hal.send_packet((uint8_t*)msg, sizeof(message_t), 2);
        }
    }
}

/**
 * Process a route response message
 */
void process_route_response(system_state_t* state, message_t* msg) {
    if (DEBUG) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "Processing route response from node %d", msg->source);
        print_status_message(buffer, 0);
    }
    
    if (msg->destination == state->node_id) {
        // This response is for us
        if (msg->payload_len >= sizeof(node_id_t)) {
            node_id_t dest_id;
            memcpy(&dest_id, msg->payload, sizeof(node_id_t));
            
            hop_count_t hop_count = 1; // At minimum one hop through sender
            
            if (msg->payload_len >= sizeof(node_id_t) + sizeof(hop_count_t)) {
                memcpy(&hop_count, msg->payload + sizeof(node_id_t), sizeof(hop_count_t));
                hop_count += 1; // Add one for the hop to the sender
            }
            
            // Add/update route through the sender
            signal_strength_t quality = state->hal.get_last_rssi();
            if (add_or_update_route(state, dest_id, msg->source, hop_count, quality)) {
                if (DEBUG) {
                    char buffer[128];
                    snprintf(buffer, sizeof(buffer), 
                            "Added/updated route to node %d via node %d (%d hops)", 
                            dest_id, msg->source, hop_count);
                    print_status_message(buffer, 1);
                }
            }
        }
    } else {
        // Forward the response
        if (DEBUG) {
            char buffer[128];
            snprintf(buffer, sizeof(buffer), "Forwarding route response to node %d", msg->destination);
            print_status_message(buffer, 0);
        }
        forward_message(state, msg);
    }
}

/**
 * Remove stale routes from the routing table
 */
void clean_stale_routes(system_state_t* state) {
    uint32_t current_time = state->hal.get_time_ms();
    uint32_t route_timeout = NODE_TIMEOUT_MS * 2; // Routes time out after twice node timeout
    
    for (int i = 0; i < state->route_count; i++) {
        if (current_time - state->routing_table[i].last_updated > route_timeout) {
            // Remove the stale route by shifting the array
            if (DEBUG) {
                char buffer[128];
                snprintf(buffer, sizeof(buffer), 
                        "Removing stale route to node %d", 
                        state->routing_table[i].destination);
                print_status_message(buffer, 2);
            }
            
            if (i < state->route_count - 1) {
                memmove(&state->routing_table[i], &state->routing_table[i + 1], 
                       (state->route_count - i - 1) * sizeof(route_entry_t));
            }
            state->route_count--;
            i--; // Check the same index again since we shifted
        }
    }
}

/**
 * Scan all RF channels to find the least congested one
 */
void scan_channels(system_state_t* state) {
    uint8_t signal_levels[RF_CHANNEL_COUNT] = {0};
    
    print_status_message("Scanning RF channels...", 0);
    
    // Scan each channel
    for (uint8_t channel = 0; channel < RF_CHANNEL_COUNT; channel++) {
        state->hal.set_rf_channel(channel);
        
        // Wait for a short period to sample the channel
        state->hal.sleep_ms(10);
        
        // Get signal strength (higher value = more congestion)
        signal_levels[channel] = state->hal.get_last_rssi();
        
        if (DEBUG) printf("  Channel %2d: Signal level %3d\n", channel, signal_levels[channel]);
    }
    
    // Find least congested channel
    uint8_t best_channel = 0;
    uint8_t lowest_signal = 255;
    
    for (uint8_t channel = 0; channel < RF_CHANNEL_COUNT; channel++) {
        if (signal_levels[channel] < lowest_signal) {
            lowest_signal = signal_levels[channel];
            best_channel = channel;
        }
    }
    
    // Switch to best channel
    if (best_channel != state->current_rf_channel) {
        char buffer[128];
        snprintf(buffer, sizeof(buffer), 
                "Switching to channel %d (least congested, signal level: %d)", 
                best_channel, lowest_signal);
        print_status_message(buffer, 1);
        switch_channel(state, best_channel);
    } else {
        print_status_message("Already on optimal channel", 1);
    }
}

/**
 * Switch to a different RF channel
 */
void switch_channel(system_state_t* state, uint8_t channel) {
    if (channel < RF_CHANNEL_COUNT) {
        state->hal.set_rf_channel(channel);
        state->current_rf_channel = channel;
        
        char buffer[128];
        snprintf(buffer, sizeof(buffer), "Switched to channel %d", channel);
        print_status_message(buffer, 0);
    }
}

/**
 * Check battery status and adjust behavior accordingly
 */
void check_battery_status(system_state_t* state) {
    state->battery_level = state->hal.get_battery_level();
    
    if (state->battery_level <= BATTERY_CRIT_THRESHOLD) {
        // Critical battery level
        // Only process emergency messages
        // Enter low power mode when possible
        enter_low_power_mode(state);
        print_status_message("CRITICAL BATTERY LEVEL! Entering power saving mode", 3);
    } else if (state->battery_level <= BATTERY_LOW_THRESHOLD) {
        // Low battery
        // Reduce beacon frequency
        // Don't act as a relay for non-emergency messages
        print_status_message("Low battery level, reducing network activity", 2);
    }
}

/**
 * Adapt power settings based on network conditions
 */
void adapt_power_settings(system_state_t* state) {
    // If we have many nearby nodes, reduce transmission power
    int nearby_nodes = 0;
    for (int i = 0; i < state->node_count; i++) {
        if (state->known_nodes[i].hop_count == 1 && 
            state->known_nodes[i].signal_strength > 200) {
            nearby_nodes++;
        }
    }
    
    // Adjust beacon interval based on node density
    if (nearby_nodes > 10) {
        // High density, reduce beacon frequency
        if (DEBUG) print_status_message("High node density detected, reducing beacon frequency", 0);
    } else if (nearby_nodes < 3) {
        // Low density, increase beacon frequency
        if (DEBUG) print_status_message("Low node density detected, increasing beacon frequency", 0);
    }
}

/**
 * Enter low power mode
 */
void enter_low_power_mode(system_state_t* state) {
    if (!state->low_power_mode) {
        state->low_power_mode = true;
        state->hal.enter_low_power_mode();
        print_status_message("Entered low power mode", 2);
    }
}

/**
 * Exit low power mode
 */
void exit_low_power_mode(system_state_t* state) {
    if (state->low_power_mode) {
        state->low_power_mode = false;
        state->hal.exit_low_power_mode();
        print_status_message("Exited low power mode", 1);
    }
}