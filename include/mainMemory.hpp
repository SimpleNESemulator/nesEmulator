#ifndef MAIN_MEMORY_H
#define MAIN_MEMORY_H
#include <iostream>

// memory operations
uint8_t memRead(uint16_t address);
uint8_t* memPoint(uint16_t address);
void memWrite(uint16_t address,uint8_t value);

#endif