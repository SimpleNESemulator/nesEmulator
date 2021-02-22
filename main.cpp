#include <iostream>
#include "mainMemory.hpp"
#include "cpu.hpp"

using namespace std;

int main(){
    cpu c{};
    c.pc = 0;
    uint8_t program[] = {
        0x69, 0xFE, // add 254
        0x69, 0x0C // add 12 should print 10
    };
    for (size_t i = 0; i < 4; i++)
        memWrite(i, program[i]);

    c.fetch();
    c.decode();
    c.execute();

    c.fetch();
    c.decode();
    c.execute();


    cout << (int) c.ac << endl;
    return 0;
}