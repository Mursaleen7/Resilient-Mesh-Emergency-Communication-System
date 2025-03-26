# Resilient-Mesh-Emergency-Communication-System


This application is like a special walkie-talkie system that works when your phone doesn't.

When disasters like earthquakes or hurricanes knock out cell towers and internet, your phone stops working because it needs those networks to send messages. This system solves that problem by letting devices talk directly to each other instead.

Imagine dropping special devices throughout a city. Each one can send messages to any other device it can "see." If it can't see the destination directly, it passes the message to a nearby device, which passes it along until it reaches the right person - like a game of telephone. The system also:

- Shows where everyone is on a map
- Marks important places like shelters or medical stations
- Shares this information with everyone connected
- Keeps working even if some devices break or run out of power

The best part? It needs zero outside infrastructure to work. No cell towers, no internet, no power grid. This makes it perfect for emergency responders when normal communications fail.

---


This system implements a dynamically routed, decentralized mesh network optimized for emergency communications with the following technical specifications:

### Core Technologies
- **Language & Architecture**: Written in C with a modular architecture using separation of concerns
- **Hardware Abstraction Layer (HAL)**: Designed for cross-platform compatibility between simulation environments and Raspberry Pi hardware
- **Network Protocol**: Custom lightweight packet-based protocol with message authentication codes (MAC) for security
- **Persistence**: Local storage for map tiles and configuration using filesystem

### Networking Implementation
- **Routing Algorithm**: Distance vector routing with hop-count and signal quality metrics
- **Addressing**: 16-bit node addressing with unicast and broadcast capabilities
- **Topology**: Dynamic mesh with auto-discovery and route optimization
- **Reliability**: Acknowledgment-based message delivery with automatic retransmission
- **RF Communication**: Channel scanning and selection to avoid interference (real hardware mode)
- **Security**: XOR-based encryption with shared key authentication (could be extended to AES)

### Hardware Support
- **Simulation Mode**: Uses UDP sockets to simulate RF communication between virtual nodes
- **Raspberry Pi Implementation**: 
  - WiringPi for GPIO and hardware interface
  - SPI for RF module communication
  - UART for GPS module integration
  - Power management for battery conservation

### Location Services
- **Positioning**: GPS integration with NMEA sentence parsing
- **Fallback**: RF triangulation using signal strength and round-trip timing
- **Map System**: Offline map tile storage and sharing
- **Geographic Calculations**: Haversine formula for distance calculation between coordinates

### Emergency Features
- **Location Tracking**: Automatic position sharing and updating
- **Point of Interest System**: Structured data sharing of critical locations
- **Rescue Zone Management**: Operational area definition with status tracking
- **Automatic Power Adaptation**: Dynamic transmission power based on node density and battery status

The application employs a terminal-based interface for command input and visualization, with the ability to save and load persistent data. The modular design ensures easy extension with additional features while maintaining a small footprint suitable for deployment on resource-constrained devices.
