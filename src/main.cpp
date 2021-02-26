#include <iostream>
#include <bitset>

#include "cpu.hpp"
#include "mainMemory.hpp"

using namespace std;

int main(){
    cpu c{};
    uint8_t program[] = {
        0x69, 0xF0, // add 240
        0x69, 0x1F, // add 31
        0x90, 0xFD, // go back by 3 add 31 again
    };
    for (uint16_t i = 0; i < 6; i++)
        memWrite(i, program[i]);

    for (size_t i = 0; i < 20; i++){
        c.status &= 0;
        c.fetch();
        c.decode();
        c.execute();

        cout << "A: " << (int) c.ac << "\tstatus:\t" << bitset<8>(c.status) << endl;
    }

    return 0;
}