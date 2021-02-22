#ifndef MAIN_MEMORY
#define MAIN_MEMORY
#include <iostream>

//memory operations
uint8_t memRead(uint16_t address);
void memWrite(uint16_t address,uint8_t value);
uint8_t* memPoint(uint16_t address);

#endif