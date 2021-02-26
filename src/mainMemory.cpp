#include "mainMemory.hpp"

uint8_t memory[0xFFFF];

uint8_t memRead(uint16_t address){
    return memory[address];
}

// returns a pointer to the value mainMemory[address]
uint8_t *memPoint(uint16_t address) {
    return (memory + address);
}

void memWrite(uint16_t address,uint8_t value){
    memory[address] = value;
}
