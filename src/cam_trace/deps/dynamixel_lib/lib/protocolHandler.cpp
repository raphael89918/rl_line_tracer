#include <chrono>
#include <iostream>
#include <thread>

#include "protocolHandler.h"

using namespace std;

ProtocolHandler::ProtocolHandler(string portName, bool show) {
    this->protocol1 = dynamixel::PacketHandler::getPacketHandler(1.0);
    this->protocol2 = dynamixel::PacketHandler::getPacketHandler(2.0);

    portHandler = dynamixel::PortHandler::getPortHandler(portName.c_str());
    if (portHandler->openPort()) {
        if (show) {
            cout << "Succeeded to connect motor!" << endl;
            cout << " - Device Name : " << portName << endl;
            cout << " - Baudrate    : " << portHandler->getBaudRate() << endl;
        }
    } else {
        cout << "Failed to open the port! [" << portName << "]" << endl;
    }
}

ProtocolHandler::~ProtocolHandler() {}

void ProtocolHandler::write_bits(int protocol, int ID, int address, int value) {
    if (protocol == 1) {
        write(portHandler, this->protocol1, ID, address, 1, value);
    } else if (protocol == 2) {
        write(portHandler, this->protocol2, ID, address, 1, value);
    }
}
void ProtocolHandler::write_words(int protocol, int ID, int address,
                                  int value) {
    if (protocol == 1) {
        write(portHandler, this->protocol1, ID, address, 2, value);
    } else if (protocol == 2) {
        write(portHandler, this->protocol2, ID, address, 2, value);
    }
}
void ProtocolHandler::write_dwords(int protocol, int ID, int address,
                                   int value) {
    if (protocol == 1) {
        cerr << "protocol1 cannot write dword" << endl;
    } else if (protocol == 2) {
        write(portHandler, this->protocol2, ID, address, 4, value);
    }
}
int ProtocolHandler::read_bits(int protocol, int ID, int address) {
    int result = -1;

    if (protocol == 1) {
        result = read(portHandler, this->protocol1, ID, address, 1);
    } else if (protocol == 2) {
        result = read(portHandler, this->protocol2, ID, address, 1);
    }

    return result;
}
int ProtocolHandler::read_words(int protocol, int ID, int address) {
    int result = -1;

    if (protocol == 1) {
        result = read(portHandler, this->protocol1, ID, address, 2);
    } else if (protocol == 2) {
        result = read(portHandler, this->protocol2, ID, address, 2);
    }

    return result;
}
int ProtocolHandler::read_dwords(int protocol, int ID, int address) {
    int result = -1;

    if (protocol == 1) {
        cerr << "protocol1 cannot read dword" << endl;
    } else if (protocol == 2) {
        result = read(portHandler, this->protocol2, ID, address, 4);
    }

    return result;
}
int ProtocolHandler::reset(int protocol, int ID) {
    uint8_t dxl_error;
    if (protocol == 1) {
        int dxl_comm_result =
            protocol1->factoryReset(portHandler, ID, 0x00, &dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
            if (dxl_error != 0)
                printf("%s\n", protocol1->getRxPacketError(dxl_error));
            // fprintf(stderr, "\n Success to reset! \n\n");
            return 0;

        } else {
            printf("%s\n", protocol1->getTxRxResult(dxl_comm_result));
            fprintf(stderr, "\n Fail to reset! \n\n");
            return -1;
        }
    }

    else if (protocol == 2) {
        int dxl_comm_result =
            protocol2->factoryReset(portHandler, ID, 255, &dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
            if (dxl_error != 0)
                printf("%s\n", protocol2->getRxPacketError(dxl_error));
            fprintf(stderr, "\n Success to reset! \n\n");

        } else {
            printf("%s\n", protocol2->getTxRxResult(dxl_comm_result));
            fprintf(stderr, "\n Fail to reset! \n\n");
        }
    }
    cerr << "error status" << endl;
    return -1;
}
int ProtocolHandler::reboot(int protocol, int ID) {
    if (protocol == 1) {
        cerr << "protocol1 cannot reboot" << endl;
        return -1;
    }

    else if (protocol == 2) {
        uint8_t dxl_error;

        int dxl_comm_result =
            this->protocol2->reboot(portHandler, ID, &dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
            if (dxl_error != 0)
                printf("%s\n", this->protocol2->getRxPacketError(dxl_error));
            fprintf(stderr, "\n Success to reboot! \n\n");
            this_thread::sleep_for(chrono::milliseconds(100));
            return 0;

        } else {
            printf("%s\n", this->protocol2->getTxRxResult(dxl_comm_result));
            fprintf(stderr, "\n Fail to reboot! \n\n");
            return -1;
        }
    }
    cerr << "error status" << endl;
    return -1;
}

void write(dynamixel::PortHandler *portHandler,
           dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr,
           uint16_t length, uint32_t value) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    if (length == 1) {
        dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler, id, addr, (uint8_t)value, &dxl_error);
        this_thread::sleep_for(chrono::milliseconds(10));
    } else if (length == 2) {
        dxl_comm_result = packetHandler->write2ByteTxRx(
            portHandler, id, addr, (uint16_t)value, &dxl_error);
        this_thread::sleep_for(chrono::milliseconds(10));
    } else if (length == 4) {
        dxl_comm_result = packetHandler->write4ByteTxRx(
            portHandler, id, addr, (uint32_t)value, &dxl_error);
        this_thread::sleep_for(chrono::milliseconds(10));
    }

    if (dxl_comm_result == COMM_SUCCESS) {
        if (dxl_error != 0)
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        // fprintf(stderr, "\n Success to write\n\n");

    } else {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        fprintf(stderr, "\n Fail to write! \n\n");
    }
}

int read(dynamixel::PortHandler *portHandler,
         dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr,
         uint16_t length) {
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    int8_t value8 = 0;
    int16_t value16 = 0;
    int32_t value32 = 0;

    if (length == 1) {
        dxl_comm_result = packetHandler->read1ByteTxRx(
            portHandler, id, addr, (uint8_t *)&value8, &dxl_error);
        this_thread::sleep_for(chrono::milliseconds(10));
    } else if (length == 2) {
        dxl_comm_result = packetHandler->read2ByteTxRx(
            portHandler, id, addr, (uint16_t *)&value16, &dxl_error);
        this_thread::sleep_for(chrono::milliseconds(10));
    } else if (length == 4) {
        dxl_comm_result = packetHandler->read4ByteTxRx(
            portHandler, id, addr, (uint32_t *)&value32, &dxl_error);
        this_thread::sleep_for(chrono::milliseconds(10));
    }

    if (dxl_comm_result == COMM_SUCCESS) {
        if (dxl_error != 0) {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            return -1;
        }

        if (length == 1) {
            // fprintf(stderr, "\n READ VALUE : (UNSIGNED) %u , (SIGNED) %d
            // \n\n", (uint8_t)value8, value8);
            return ((int)value8);
        } else if (length == 2) {
            // fprintf(stderr, "\n READ VALUE : (UNSIGNED) %u , (SIGNED) %d
            // \n\n", (uint16_t)value16, value16);

            return ((int)value16);
        } else if (length == 4) {
            // fprintf(stderr, "\n READ VALUE : (UNSIGNED) %u , (SIGNED) %d
            // \n\n", (uint32_t)value32, value32);

            return ((int)value32);
        } else {
            cout << "error status!!";
            return -1;
        }
    }

    else {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        fprintf(stderr, "\n Fail to read! \n\n");
        return -1;
    }
}
