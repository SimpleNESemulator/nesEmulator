#include "mainMemory.hpp"

uint8_t mainMemory[0xFFFF];

uint8_t memRead(uint16_t address){
    return mainMemory[address];
}

void memWrite(uint16_t address,uint8_t value){
    mainMemory[address] = value;
}

// returns a pointer to the value mainMemory[address]
// this is useful for loading the operand1 and operand2
uint8_t *memPoint(uint16_t address) {
    return (mainMemory + address);
}