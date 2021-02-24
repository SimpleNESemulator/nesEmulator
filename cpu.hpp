#ifndef CPU_H
#define CPU_H

#include <iostream>
#include "mainMemory.hpp"

//Addressing modes
enum modes{
    xxx, //illegal
    imm, //IMMediate
    zpg, //ZeroPaGe
    zpx, //ZeroPage,X
    zpy, //ZeroPage,Y
    abl, //ABsoLute
    abx, //ABsolute,X
    aby, //ABsolute,Y
    inx, //INdirect,X
    iny, //INdirect,Y
    rel, //RELative
    imp, //IMPlied
    acc, //ACCumulator
    ind, //INDirect
    ixt, //IndeXed indirecT
    itx, //IndirecT indeXed
};

enum {
    
};

<<<<<<< HEAD
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
        uint8_t sp;         //stack pointer
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
        // void push(uint8_t);
        // uint8_t pull();

        //cpu operations
        uint8_t fetch();
        void decode();
        void execute();
        void clock();
        
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
=======
struct cpuStruct{
    uint16_t pc; //program counter
    uint8_t ac; //accumulator
    uint8_t x,y; //x and y register
    uint8_t sp; //stack pointer

    uint8_t NVUBDIZC; //NVUBDIZC flag

    uint16_t ip; //instruction pointer
    uint8_t addressingMode; //addressing mode
    uint8_t opcode; //instruction opcode 
    uint8_t operand1; //first operand
    uint8_t operand2; //second operand
};

//Instructions
void ADC();
void AND();
void ASL();
void BCC();
void BCS();
void BEQ();
void BIT();
void BMI();
void BNE();
void BPL();
void BRK();
void BVC();
void BVS();
void CLC();
void CLD();
void CLI();
void CLV();
void CMP();
void CPX();
void CPY();
void DEC();
void DEX();
void DEY();
void EOR();
void INC();
void INX();
void INY();
void JMP();
void JSR();
void LDA();
void LDX();
void LDY();
void LSR();
void NOP();
void ORA();
void PHA();
void PHP();
void PLA();
void PLP();
void ROL();
void ROR();
void RIT();
void RTS();
void SBC();
void SEC();
void SED();
void SEI();
void STA();
void STX();
void STY();
void TAX();
void TAY();
void TSX();
void TXA();
void TXS();
void TYA();
//Illegal opcodes
void KIL();
void SLO();
void RLA();
void SRE();
void RTI();
void RRA();
void SAX();
void AHX();
void LAX();
void DCP();
void ISC();
void ANC();
void ALR();
void ARR();
void XAA();
void SHY();
void TAS();
void SHX();
void LAS();
void AXS();

//helper functions
//flag modify
void setFlag(uint8_t);

//stack operations
void push(uint8_t);
uint8_t pull();

void fetch();
void decode();
void execute(uint8_t);
>>>>>>> c74e091043b362ffa76f5dfa9d03816fb88a3967

#endif