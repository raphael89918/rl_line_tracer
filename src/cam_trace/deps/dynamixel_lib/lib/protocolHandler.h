#ifndef PROTOCOLHANDLER_H
#define PROTOCOLHANDLER_H

#include <iostream>

#include "dynamixel_sdk.h"  // Uses Dynamixel SDK library

class ProtocolHandler {
   public:
    // constructor
    ProtocolHandler(std::string portName, bool show);
    ~ProtocolHandler();

    // memberfunction
    void write_bits(int protocol, int ID, int address, int value);
    void write_words(int protocol, int ID, int address, int value);
    void write_dwords(int protocol, int ID, int address, int value);

    int read_bits(int protocol, int ID, int address);
    int read_words(int protocol, int ID, int address);
    int read_dwords(int protocol, int ID, int address);

    int reset(int protocol, int ID);
    int reboot(int protocol, int ID);

   private:
    // member variable

    // protocol version handler
    dynamixel::PacketHandler *protocol1;
    dynamixel::PacketHandler *protocol2;

    // serialport handler
    dynamixel::PortHandler *portHandler;
};

void write(dynamixel::PortHandler *portHandler,
           dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr,
           uint16_t length, uint32_t value);
int read(dynamixel::PortHandler *portHandler,
         dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr,
         uint16_t length);

#endif  // PROTOCOLHANDLER_H
