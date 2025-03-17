#ifndef MT25QL_DRIVER_H
#define MT25QL_DRIVER_H

#include <Arduino.h>
#include "wiring_private.h"
#include "mt25ql_definitions.h"

class MT25QL {
    public:
        MT25QL(int8_t sck, int8_t cs, int8_t io0, int8_t io1, int8_t io2, int8_t io3);
        void begin(void);
        void end(void);
    
        void runCommand(uint8_t command);
        void readData(uint8_t command, uint8_t* buffer, uint32_t len);
        void writeData(uint8_t command, const uint8_t* data, uint32_t len);
        void readCommand(uint8_t command, uint8_t *response, uint32_t len);
        void eraseSector(uint32_t address);
        void eraseBlock(uint32_t address);
        void eraseChip(void);
        
        void readBuffer(uint32_t address, uint8_t *buffer, uint32_t len);
        void writeBuffer(uint32_t address, const uint8_t *buffer, uint32_t len);
    
    private:
        int8_t _sck, _cs, _io0, _io1, _io2, _io3;
    
        void configureQSPI(void);
        void enableCache(void);
        void disableCache(void);
        void enableQuadMode(void);
        void waitForReady(void);
        void _run_instruction(uint8_t command, uint32_t iframe, uint32_t addr, uint8_t *buffer, uint32_t size);
    };
    
    #endif // MT25QL_DRIVER_H