#include "mainMemory.hpp"

uint8_t mainMemory[0xFFFF];

uint8_t memRead(uint16_t address){
    return mainMemory[address];
}

void memWrite(uint16_t address,uint8_t value){

}