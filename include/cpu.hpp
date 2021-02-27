#ifndef CPU_H
#define CPU_H

#include <iostream>
#include "mainMemory.hpp"

class cpu;
typedef void (cpu::*instr_ptr) ();

//Addressing modes
enum addressingMode{
    XXX,    // illegal
    IMM,    // immediate
    ZPG,    // zeropage
    ZPGX,   // zeropage, x-indexed
    ZPGY,   // zeropage, y-indexed
    ABS,    // absolute
    ABSX,   // absolute, x-indexed
    ABSY,   // absolute, y-indexed
    IND,    // indirect
    XIND,   // X-indexted, indirect
    INDY,   // indirect, Y-indexted
    REL,    // relative
    IMPL,   // implied
    ACC,    // accumulator
};

enum flag{
    C = (1 << 0), // Carry
    Z = (1 << 1), // Zero
    I = (1 << 2), // Interrupt Disable 
    D = (1 << 3), // Decimal
    B = (1 << 4), // Break
    S = (1 << 5), // Ignored 
    V = (1 << 6), // Overflow
    N = (1 << 7), // Negative
};

typedef struct instruction {
    instr_ptr instr;
    addressingMode addressing;
    uint8_t size;
    uint8_t cycles;
    uint8_t pageCycles;
} instruction;

class cpu {
    public:
        uint16_t pc{0};    // program counter
        uint8_t ac;         // accumulator
        uint8_t x,y;        // x and y register
        uint16_t sp{0x1FF};         //stack pointer
        uint8_t opcode;     //instruction opcode
        uint8_t status;     // status register

        instruction currentInstr;   // current instruction
        uint8_t fetched;            // fetched byte from memory

        // operand is a pointer because some instructions
        // operate on either a value in memory or the accumulator.
        // operand can be pointed to whatever necessary and instructions
        // can simply work on the operand without knowing what it's pointing to
        uint8_t *operand;
        // target address for indirect addressing modes
        // This is used as placeholder mostly, only time 
        // the address is actually used is in JMP and JSR
        // other times I just pointed operand to the value in address
        uint16_t targetAddress;
        ulong cycles;

        // //stack operations
        void push(uint8_t);
        uint8_t pull();

        //cpu operations
        uint8_t fetch();
        void decode();
        void execute();
        void clock();
        
        // branching instructions follow the same structure
        // if condition is true:
        //      branch

        void branch();

        // instructions
        void ADC(); void AND(); void ASL(); void BCC(); 
        void BCS(); void BEQ(); void BIT(); void BMI(); 
        void BNE(); void BPL(); void BRK(); void BVC(); 
        void BVS(); void CLC(); void CLD(); void CLI(); 
        void CLV(); void CMP(); void CPX(); void CPY(); 
        void DEC(); void DEX(); void DEY(); void EOR(); 
        void INC(); void INX(); void INY(); void JMP(); 
        void JSR(); void LDA(); void LDX(); void LDY(); 
        void LSR(); void NOP(); void ORA(); void PHA(); 
        void PHP(); void PLA(); void PLP(); void ROL(); 
        void ROR(); void RIT(); void RTS(); void SBC(); 
        void SEC(); void SED(); void SEI(); void STA(); 
        void STX(); void STY(); void TAX(); void TAY(); 
        void TSX(); void TXA(); void TXS(); void TYA(); 

        // illegal opcodes
        void KIL(); void SLO(); void RLA(); void SRE(); 
        void RTI(); void RRA(); void SAX(); void AHX(); 
        void LAX(); void DCP(); void ISC(); void ANC(); 
        void ALR(); void ARR(); void XAA(); void SHY(); 
        void TAS(); void SHX(); void LAS(); void AXS(); 
};

#endif