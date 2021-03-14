#include <iostream>
#include <iomanip>
#include <bitset>

#include "cpu.hpp"
#include "mainMemory.hpp"

using namespace std;

int main(){
    cpu c{};
    // uint8_t program[] = {
    //     0x69, 0xF0, // add 240
    //     0x69, 0x1F, // add 31
    //     0x90, 0xFD, // go back by 3 add 31 again
    // };


    uint8_t program[] = {
        0xA9, 0x05, // LDA #0x05
        0xC9, 0x04, // CMP #0x04
        0x90, 0xFD, // go back by 3 add 31 again
    };

    for (uint16_t i = 0; i < ((sizeof program) / (sizeof *program)); i++)
        memWrite(i, program[i]);

    cout << "===============================================CZIDBSVN===========" << endl;

    for (size_t i = 0; i < 20; i++){
        c.status &= 0;
        c.fetch();
        c.decode();
        c.execute();
        cout << "instrAddr:" << setfill('0') << setw(4) << c.instrAddress 
             << " |PC:" << setfill('0') << setw(4) << c.pc 
             << " |instr:" << (char*) opcodeStr[c.currentInstr.instrOpcode]
             << " |A:" << hex << setfill('0') << setw(2) << (int) c.ac 
             << " |flag:" << bitset<8>(c.status)
             << " |mode:" << setw(4) << addrModeStr[c.currentInstr.addressing] << endl;
    }

    return 0;
}