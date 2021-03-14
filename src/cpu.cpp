#include "cpu.hpp"

void cpu::ADC(){
    // Affects Flags: N V Z C
    // A + M + C -> A, C

    uint res = ac + *operand + (status & C);
    bool carryOut = (res & 0x100);
    ac = res & 0xFF;

    // N - set bit 7 to the results bit 7 (0 indexed)
    status |= (ac & 0x80);

    // Z - set if result is zero
    // if res == 0 -> !(res ^ 0) << 1 == 0b00000010
    status |= (!(res ^ 0) << 1);

    // V - overflow only occurs when carry out and carry in
    // are different.
    status |= (carryOut ^ (status & C)) << 6;

    // C - set if there is a carry out
    status |= carryOut;
}
void cpu::AND(){
    // Affects Flags: N Z
    // A AND M -> A

    ac &= *operand;

    // Z
    status |= (!(ac ^ 0) << 1);
    // N
    status |= ac & 0x80;
}
void cpu::ASL(){
    // Affects Flags: N Z C 
    // C <- [76543210] <- 0

    bool carryOut = *operand & 0x80;
    *operand <<= 1;

    // N
    status |= *operand & 0x80;
    // Z
    status |= (!(*operand ^ 0) << 1);
    // C
    status |= carryOut;
};
void cpu::BCC(){
    // Branch on carry clear, C == 0

    // if status bit-0 isn't set
    if (!(status & C))
        branch();
};
void cpu::BCS(){
    // Branch on carry set, C == 1

    // if status bit-0 is set
    if (status & C)
       branch();
};
void cpu::BEQ(){
    // Branch on result zero, Z == 1

    // if status bit-1 is set
    if (status & Z)
        branch();
};
void cpu::BIT(){
    // bits 7 and 6 of operand are transfered to bits
    // 7 and 6 of status [N, V]
    // the Z flag is set to the result of operand AND accumulator
    // Affects Flags: N V Z 

    // mask status with 0011 1111 to clear bits 6 and 7
    status &= ~0xC0;
    // set bits 6 and 7 to operand's 6 and 7
    status |= *operand & 0xC0;

    // notice it's && not &
    status |= (*operand && ac) << 1;
};
void cpu::BMI(){
    // Branch on result minus, N == 1

    if (status & N)
        branch();
};
void cpu::BNE(){
    // Branch on result not zero, Z == 0

    if (!(status & Z))
        branch();
};
void cpu::BPL(){
    // Branch on result plus, N == 0

    if(!(status & N))
        branch();
};
void cpu::BRK(){
    // Force Break, interrupt
    // push PC+2, push SR

    // pc is already looking ahead
    // in cpu::fetch() {fetched = memRead(pc++);}
    pc++;

    // store PC(hi)
    push((pc & 0xFF00) >> 8);

    // store PC(lo)
    push(pc & 0x00FF);
    

    // not sure if we need to store the status
    // register with the interrupt flag set or
    // a copy of it where it's set but on the original
    // it's not

    //set interrupt flag
    status |= I;
    // store status register
    push(status);
};
void cpu::BVC(){
    // Branch on overflow clear, V == 0

    if (!(status & V))
        branch();
};
void cpu::BVS(){
    // Branch on overflow set, V == 1

    if (status & V)
        branch();
};
void cpu::CLC(){
    // Clear carry bit
    status &= ~C;
};
void cpu::CLD(){
    // Clear decimal bit
    status &= ~D;
};
void cpu::CLI(){
    // Clear interrupt disable bit
    status &= ~I;
};
void cpu::CLV(){
    // Clear overflow bit
    status &= ~V;
}
void cpu::CMP(){
    // Compare memory with accumulator
    // Affects Flags: N Z C

    // Check if A and the operand are equal
    uint diff = ac ^ *operand;
    
    // If A and the operand are equal then diff == 0
    // If they are then Z is set
    if (!diff)
        status |= 0x20;

    // Check if A is greater than the operand
    // if the leftmost 1 bit of res1 is futher left than
    // the leftmost 1 bit of res2 then ac is greater than the operand
    uint res1 = (ac & ~(*operand));
    uint res2 = (*operand & ~ac);

    // shift res1 and res2 until one reaches 0
    while(res1 != 0 && res2 != 0)
    {
        res1 << 1; 
        res2 << 1;
    }

    // check which result reached 0 first and set N if res1 reached 0 first
    // or set C if res2 reached 0 first
    if(res1 == 0)
        status |= 0x80; // set N
    else
        status |= 0x10; // Set C
};
void cpu::CPX(){};
void cpu::CPY(){};
void cpu::DEC(){};
void cpu::DEX(){};
void cpu::DEY(){};
void cpu::EOR(){};
void cpu::INC(){};
void cpu::INY(){};
void cpu::INX(){};
void cpu::JMP(){};
void cpu::JSR(){};
void cpu::LDA(){
    ac = *operand;

    // modify Z flag
    if(ac == 0)
        status |= 0x40; // Set Z
    else
        status &= 0xBF; // Clear Z

    //modify N flag
    if(((ac & 0x80) >> 7) == 1)
        status |= 0x1; // Set N
    else
        status &= 0xFE; // Clear N
};
void cpu::LDX(){};
void cpu::LDY(){};
void cpu::LSR(){};
void cpu::NOP(){};
void cpu::ORA(){}
void cpu::PHA(){};
void cpu::PHP(){};
void cpu::PLA(){};
void cpu::PLP(){};
void cpu::ROL(){};
void cpu::ROR(){};
void cpu::RIT(){};
void cpu::RTS(){};
void cpu::SBC(){};
void cpu::SEC(){};
void cpu::SED(){};
void cpu::SEI(){};
void cpu::STA(){};
void cpu::STX(){};
void cpu::STY(){};
void cpu::TAX(){};
void cpu::TAY(){};
void cpu::TSX(){};
void cpu::TXA(){};
void cpu::TXS(){};
void cpu::TYA(){};
//Illegal opcodes
void cpu::KIL(){};
void cpu::SLO(){};
void cpu::RLA(){};
void cpu::SRE(){};
void cpu::RTI(){};
void cpu::RRA(){};
void cpu::SAX(){};
void cpu::AHX(){};
void cpu::LAX(){};
void cpu::DCP(){};
void cpu::ISC(){};
void cpu::ANC(){};
void cpu::ALR(){};
void cpu::ARR(){};
void cpu::XAA(){};
void cpu::SHY(){};
void cpu::TAS(){};
void cpu::SHX(){};
void cpu::LAS(){};
void cpu::AXS(){};

void cpu::branch(){
    //since pc points to the next instruction we back up by one
    pc--;
    // branch taken adds one more cycle
    cycles++;
    // we cast operand's value to int8_t because
    // branching instructions interpret it as signed.
    uint16_t temp = pc + static_cast<int8_t>(*operand);
    pc = temp;

    if (!((pc & 0xFF00) ^ (temp & 0xFF00)))
        cycles++; // add one more cycle if the page boundary is crossed
}


// opcode string list defined here
char opcodeStr[][4] = {
    "BRK", "ORA", "KIL", "SLO", "NOP", "ASL", "PHP", "ANC", "BPL", "CLC", "JSR", "AND", "RLA", "BIT", "ROL", "PLP", "BMI", "SEC", "RTI", "EOR", "SRE", "LSR",
    "PHA", "ALR", "JMP", "BVC", "CLI", "RTS", "ADC", "RRA", "ROR", "PLA", "ARR", "BVS", "SEI", "STA", "SAX", "STY", "STX", "DEY", "TXA", "XAA", "BCC", "AHX",
    "TYA", "TXS", "TAS", "SHY", "SHX", "LDY", "LDA", "LDX", "LAX", "TAY", "TAX", "BCS", "CLV", "TSX", "LAS", "CPY", "CMP", "DCP", "DEC", "INY", "DEX", "AXS",
    "BNE", "CLD", "CPX", "SBC", "ISC", "INC", "INX", "BEQ", "SED",
};
// addressing mode list defined here
char addrModeStr[][5]{
    "XXX", "IMM", "ZPG", "ZPGX", "ZPGY", "ABS", "ABSX", "ABSY", "IND", "XIND", "INDY", "REL", "IMPL", "ACC",
};

/*
    instruction is a struct that contains:

    1. instr_ptr instr -> pointer to a member function of the cpu class (cpu::*ptr)
    2. instrOpcodeEnum instrOpcode-> enum for indexing the opcode string
    3. addressingMode addressing -> enum for different addressing modes
    4. uint8_t size -> byte size of the instruction
    5. uint8_t cycles -> amount of cycles the instruction takes the complete
    6. uint8_t pageCycles -> amount of extra cycles to be added if the page boundary of the address is crossed
*/
instruction instructions[] = {
	{&cpu::BRK,BRK,IMPL,1,7,0},	{&cpu::ORA,ORA,XIND,2,6,0},	{&cpu::KIL,KIL,XXX,1,2,0},	{&cpu::SLO,SLO,XXX,2,8,0},
	{&cpu::NOP,NOP,XXX,2,3,0},	{&cpu::ORA,ORA,ZPG,2,3,0},	{&cpu::ASL,ASL,ZPG,2,5,0},	{&cpu::SLO,SLO,XXX,2,5,0},
	{&cpu::PHP,PHP,IMPL,1,3,0},	{&cpu::ORA,ORA,IMM,2,2,0},	{&cpu::ASL,ASL,ACC,1,2,0},	{&cpu::ANC,ANC,XXX,2,2,0},
	{&cpu::NOP,NOP,XXX,3,4,0},	{&cpu::ORA,ORA,ABS,3,4,0},	{&cpu::ASL,ASL,ABS,3,6,0},	{&cpu::SLO,SLO,XXX,3,6,0},
	{&cpu::BPL,BPL,REL,2,2,1},	{&cpu::ORA,ORA,INDY,2,5,1},	{&cpu::KIL,KIL,XXX,1,2,0},	{&cpu::SLO,SLO,XXX,2,8,0},
	{&cpu::NOP,NOP,XXX,2,4,0},	{&cpu::ORA,ORA,ZPGX,2,4,0},	{&cpu::ASL,ASL,ZPGX,2,6,0},	{&cpu::SLO,SLO,XXX,2,6,0},
	{&cpu::CLC,CLC,IMPL,1,2,0},	{&cpu::ORA,ORA,ABSY,3,4,1},	{&cpu::NOP,NOP,XXX,1,2,0},	{&cpu::SLO,SLO,XXX,3,7,0},
	{&cpu::NOP,NOP,XXX,3,4,1},	{&cpu::ORA,ORA,ABSX,3,4,1},	{&cpu::ASL,ASL,ABSX,3,7,0},	{&cpu::SLO,SLO,XXX,3,7,0},
	{&cpu::JSR,JSR,ABS,3,6,0},	{&cpu::AND,AND,XIND,2,6,0},	{&cpu::KIL,KIL,XXX,1,2,0},	{&cpu::RLA,RLA,XXX,2,8,0},
	{&cpu::BIT,BIT,ZPG,2,3,0},	{&cpu::AND,AND,ZPG,2,3,0},	{&cpu::ROL,ROL,ZPG,2,5,0},	{&cpu::RLA,RLA,XXX,2,5,0},
	{&cpu::PLP,PLP,IMPL,1,4,0},	{&cpu::AND,AND,IMM,2,2,0},	{&cpu::ROL,ROL,ACC,1,2,0},	{&cpu::ANC,ANC,XXX,2,2,0},
	{&cpu::BIT,BIT,ABS,3,4,0},	{&cpu::AND,AND,ABS,3,4,0},	{&cpu::ROL,ROL,ABS,3,6,0},	{&cpu::RLA,RLA,XXX,3,6,0},
	{&cpu::BMI,BMI,REL,2,2,1},	{&cpu::AND,AND,INDY,2,5,1},	{&cpu::KIL,KIL,XXX,1,2,0},	{&cpu::RLA,RLA,XXX,2,8,0},
	{&cpu::NOP,NOP,XXX,2,4,0},	{&cpu::AND,AND,ZPGX,2,4,0},	{&cpu::ROL,ROL,ZPGX,2,6,0},	{&cpu::RLA,RLA,XXX,2,6,0},
	{&cpu::SEC,SEC,IMPL,1,2,0},	{&cpu::AND,AND,ABSY,3,4,1},	{&cpu::NOP,NOP,XXX,1,2,0},	{&cpu::RLA,RLA,XXX,3,7,0},
	{&cpu::NOP,NOP,XXX,3,4,1},	{&cpu::AND,AND,ABSX,3,4,1},	{&cpu::ROL,ROL,ABSX,3,7,0},	{&cpu::RLA,RLA,XXX,3,7,0},
	{&cpu::RTI,RTI,IMPL,1,6,0},	{&cpu::EOR,EOR,XIND,2,6,0},	{&cpu::KIL,KIL,XXX,1,2,0},	{&cpu::SRE,SRE,XXX,2,8,0},
	{&cpu::NOP,NOP,XXX,2,3,0},	{&cpu::EOR,EOR,ZPG,2,3,0},	{&cpu::LSR,LSR,ZPG,2,5,0},	{&cpu::SRE,SRE,XXX,2,5,0},
	{&cpu::PHA,PHA,IMPL,1,3,0},	{&cpu::EOR,EOR,IMM,2,2,0},	{&cpu::LSR,LSR,ACC,1,2,0},	{&cpu::ALR,ALR,XXX,2,2,0},
	{&cpu::JMP,JMP,ABS,3,3,0},	{&cpu::EOR,EOR,ABS,3,4,0},	{&cpu::LSR,LSR,ABS,3,6,0},	{&cpu::SRE,SRE,XXX,3,6,0},
	{&cpu::BVC,BVC,REL,2,2,1},	{&cpu::EOR,EOR,INDY,2,5,1},	{&cpu::KIL,KIL,XXX,1,2,0},	{&cpu::SRE,SRE,XXX,2,8,0},
	{&cpu::NOP,NOP,XXX,2,4,0},	{&cpu::EOR,EOR,ZPGX,2,4,0},	{&cpu::LSR,LSR,ZPGX,2,6,0},	{&cpu::SRE,SRE,XXX,2,6,0},
	{&cpu::CLI,CLI,IMPL,1,2,0},	{&cpu::EOR,EOR,ABSY,3,4,1},	{&cpu::NOP,NOP,XXX,1,2,0},	{&cpu::SRE,SRE,XXX,3,7,0},
	{&cpu::NOP,NOP,XXX,3,4,1},	{&cpu::EOR,EOR,ABSX,3,4,1},	{&cpu::LSR,LSR,ABSX,3,7,0},	{&cpu::SRE,SRE,XXX,3,7,0},
	{&cpu::RTS,RTS,IMPL,1,6,0},	{&cpu::ADC,ADC,XIND,2,6,0},	{&cpu::KIL,KIL,XXX,1,2,0},	{&cpu::RRA,RRA,XXX,2,8,0},
	{&cpu::NOP,NOP,XXX,2,3,0},	{&cpu::ADC,ADC,ZPG,2,3,0},	{&cpu::ROR,ROR,ZPG,2,5,0},	{&cpu::RRA,RRA,XXX,2,5,0},
	{&cpu::PLA,PLA,IMPL,1,4,0},	{&cpu::ADC,ADC,IMM,2,2,0},	{&cpu::ROR,ROR,ACC,1,2,0},	{&cpu::ARR,ARR,XXX,2,2,0},
	{&cpu::JMP,JMP,IND,3,5,0},	{&cpu::ADC,ADC,ABS,3,4,0},	{&cpu::ROR,ROR,ABS,3,6,0},	{&cpu::RRA,RRA,XXX,3,6,0},
	{&cpu::BVS,BVS,REL,2,2,1},	{&cpu::ADC,ADC,INDY,2,5,1},	{&cpu::KIL,KIL,XXX,1,2,0},	{&cpu::RRA,RRA,XXX,2,8,0},
	{&cpu::NOP,NOP,XXX,2,4,0},	{&cpu::ADC,ADC,ZPGX,2,4,0},	{&cpu::ROR,ROR,ZPGX,2,6,0},	{&cpu::RRA,RRA,XXX,2,6,0},
	{&cpu::SEI,SEI,IMPL,1,2,0},	{&cpu::ADC,ADC,ABSY,3,4,1},	{&cpu::NOP,NOP,XXX,1,2,0},	{&cpu::RRA,RRA,XXX,3,7,0},
	{&cpu::NOP,NOP,XXX,3,4,1},	{&cpu::ADC,ADC,ABSX,3,4,1},	{&cpu::ROR,ROR,ABSX,3,7,0},	{&cpu::RRA,RRA,XXX,3,7,0},
	{&cpu::NOP,NOP,XXX,2,2,0},	{&cpu::STA,STA,XIND,2,6,0},	{&cpu::NOP,NOP,XXX,2,2,0},	{&cpu::SAX,SAX,XXX,2,6,0},
	{&cpu::STY,STY,ZPG,2,3,0},	{&cpu::STA,STA,ZPG,2,3,0},	{&cpu::STX,STX,ZPG,2,3,0},	{&cpu::SAX,SAX,XXX,2,3,0},
	{&cpu::DEY,DEY,IMPL,1,2,0},	{&cpu::NOP,NOP,XXX,2,2,0},	{&cpu::TXA,TXA,IMPL,1,2,0},	{&cpu::XAA,XAA,XXX,2,2,0},
	{&cpu::STY,STY,ABS,3,4,0},	{&cpu::STA,STA,ABS,3,4,0},	{&cpu::STX,STX,ABS,3,4,0},	{&cpu::SAX,SAX,XXX,3,4,0},
	{&cpu::BCC,BCC,REL,2,2,1},	{&cpu::STA,STA,INDY,2,6,0},	{&cpu::KIL,KIL,XXX,1,2,0},	{&cpu::AHX,AHX,XXX,2,6,0},
	{&cpu::STY,STY,ZPGX,2,4,0},	{&cpu::STA,STA,ZPGX,2,4,0},	{&cpu::STX,STX,ZPGY,2,4,0},	{&cpu::SAX,SAX,XXX,2,4,0},
	{&cpu::TYA,TYA,IMPL,1,2,0},	{&cpu::STA,STA,ABSY,3,5,0},	{&cpu::TXS,TXS,IMPL,1,2,0},	{&cpu::TAS,TAS,XXX,3,5,0},
	{&cpu::SHY,SHY,XXX,3,5,0},	{&cpu::STA,STA,ABSX,3,5,0},	{&cpu::SHX,SHX,XXX,3,5,0},	{&cpu::AHX,AHX,XXX,3,5,0},
	{&cpu::LDY,LDY,IMM,2,2,0},	{&cpu::LDA,LDA,XIND,2,6,0},	{&cpu::LDX,LDX,IMM,2,2,0},	{&cpu::LAX,LAX,XXX,2,6,0},
	{&cpu::LDY,LDY,ZPG,2,3,0},	{&cpu::LDA,LDA,ZPG,2,3,0},	{&cpu::LDX,LDX,ZPG,2,3,0},	{&cpu::LAX,LAX,XXX,2,3,0},
	{&cpu::TAY,TAY,IMPL,1,2,0},	{&cpu::LDA,LDA,IMM,2,2,0},	{&cpu::TAX,TAX,IMPL,1,2,0},	{&cpu::LAX,LAX,XXX,2,2,0},
	{&cpu::LDY,LDY,ABS,3,4,0},	{&cpu::LDA,LDA,ABS,3,4,0},	{&cpu::LDX,LDX,ABS,3,4,0},	{&cpu::LAX,LAX,XXX,3,4,0},
	{&cpu::BCS,BCS,REL,2,2,1},	{&cpu::LDA,LDA,INDY,2,5,1},	{&cpu::KIL,KIL,XXX,1,2,0},	{&cpu::LAX,LAX,XXX,2,5,1},
	{&cpu::LDY,LDY,ZPGX,2,4,0},	{&cpu::LDA,LDA,ZPGX,2,4,0},	{&cpu::LDX,LDX,ZPGY,2,4,0},	{&cpu::LAX,LAX,XXX,2,4,0},
	{&cpu::CLV,CLV,IMPL,1,2,0},	{&cpu::LDA,LDA,ABSY,3,4,1},	{&cpu::TSX,TSX,IMPL,1,2,0},	{&cpu::LAS,LAS,XXX,3,4,1},
	{&cpu::LDY,LDY,ABSX,3,4,1},	{&cpu::LDA,LDA,ABSX,3,4,1},	{&cpu::LDX,LDX,ABSY,3,4,1},	{&cpu::LAX,LAX,XXX,3,4,1},
	{&cpu::CPY,CPY,IMM,2,2,0},	{&cpu::CMP,CMP,XIND,2,6,0},	{&cpu::NOP,NOP,XXX,2,2,0},	{&cpu::DCP,DCP,XXX,2,8,0},
	{&cpu::CPY,CPY,ZPG,2,3,0},	{&cpu::CMP,CMP,ZPG,2,3,0},	{&cpu::DEC,DEC,ZPG,2,5,0},	{&cpu::DCP,DCP,XXX,2,5,0},
	{&cpu::INY,INY,IMPL,1,2,0},	{&cpu::CMP,CMP,IMM,2,2,0},	{&cpu::DEX,DEX,IMPL,1,2,0},	{&cpu::AXS,AXS,XXX,2,2,0},
	{&cpu::CPY,CPY,ABS,3,4,0},	{&cpu::CMP,CMP,ABS,3,4,0},	{&cpu::DEC,DEC,ABS,3,6,0},	{&cpu::DCP,DCP,XXX,3,6,0},
	{&cpu::BNE,BNE,REL,2,2,1},	{&cpu::CMP,CMP,INDY,2,5,1},	{&cpu::KIL,KIL,XXX,1,2,0},	{&cpu::DCP,DCP,XXX,2,8,0},
	{&cpu::NOP,NOP,XXX,2,4,0},	{&cpu::CMP,CMP,ZPGX,2,4,0},	{&cpu::DEC,DEC,ZPGX,2,6,0},	{&cpu::DCP,DCP,XXX,2,6,0},
	{&cpu::CLD,CLD,IMPL,1,2,0},	{&cpu::CMP,CMP,ABSY,3,4,1},	{&cpu::NOP,NOP,XXX,1,2,0},	{&cpu::DCP,DCP,XXX,3,7,0},
	{&cpu::NOP,NOP,XXX,3,4,1},	{&cpu::CMP,CMP,ABSX,3,4,1},	{&cpu::DEC,DEC,ABSX,3,7,0},	{&cpu::DCP,DCP,XXX,3,7,0},
	{&cpu::CPX,CPX,IMM,2,2,0},	{&cpu::SBC,SBC,XIND,2,6,0},	{&cpu::NOP,NOP,XXX,2,2,0},	{&cpu::ISC,ISC,XXX,2,8,0},
	{&cpu::CPX,CPX,ZPG,2,3,0},	{&cpu::SBC,SBC,ZPG,2,3,0},	{&cpu::INC,INC,ZPG,2,5,0},	{&cpu::ISC,ISC,XXX,2,5,0},
	{&cpu::INX,INX,IMPL,1,2,0},	{&cpu::SBC,SBC,IMM,2,2,0},	{&cpu::NOP,NOP,IMPL,1,2,0},	{&cpu::SBC,SBC,XXX,2,2,0},
	{&cpu::CPX,CPX,ABS,3,4,0},	{&cpu::SBC,SBC,ABS,3,4,0},	{&cpu::INC,INC,ABS,3,6,0},	{&cpu::ISC,ISC,XXX,3,6,0},
	{&cpu::BEQ,BEQ,REL,2,2,1},	{&cpu::SBC,SBC,INDY,2,5,1},	{&cpu::KIL,KIL,XXX,1,2,0},	{&cpu::ISC,ISC,XXX,2,8,0},
	{&cpu::NOP,NOP,XXX,2,4,0},	{&cpu::SBC,SBC,ZPGX,2,4,0},	{&cpu::INC,INC,ZPGX,2,6,0},	{&cpu::ISC,ISC,XXX,2,6,0},
	{&cpu::SED,SED,IMPL,1,2,0},	{&cpu::SBC,SBC,ABSY,3,4,1},	{&cpu::NOP,NOP,XXX,1,2,0},	{&cpu::ISC,ISC,XXX,3,7,0},
	{&cpu::NOP,NOP,XXX,3,4,1},	{&cpu::SBC,SBC,ABSX,3,4,1},	{&cpu::INC,INC,ABSX,3,7,0},	{&cpu::ISC,ISC,XXX,3,7,0},
};


// stack operations
// 6502 uses the second page in memory for stack
// 0x1FF to 0x100 (it is iterated in reverse)

uint8_t cpu::pull(){
    // if it's out of bounds, wrap it around
    sp++;
    if (sp > 0x1FF)
        sp = 0x100 | (0xFF & sp);
    return memRead(sp);
}

void cpu::push(uint8_t n){
    if (sp < 0x100)
        sp = 0x100 | (0xFF & sp);
    memWrite(sp--, n);
}

uint8_t cpu::fetch(){
    // just fetch the current address
    fetched = memRead(pc++); 
    return fetched;
}

void cpu::decode(){     
    // once we have fetched we can assign our current instruction
    // we assign it to our currentInstr
    currentInstr = instructions[fetched];
    
    // address of the current instruction
    instrAddress = pc - 1;

    cycles = currentInstr.cycles;
    
    uint8_t lo, hi;     // memory for low and high bytes

    switch(currentInstr.addressing){
        case ACC:
            // operand is AC single byte
            operand = &ac; // operand points to accumulator
            break;
        case IMM:
            // operand is byte, directly read from the memory
            fetch();
            operand = &fetched;
            break;
        case ABS:
            // operand is address $HHLL
            lo = fetch();
            hi = fetch();

            // I use target address here because JMP
            // uses absolute addressing too.
            targetAddress = (hi << 8) | (lo);
            operand = memPoint(targetAddress);
            break;
        case ZPG:
            // operand is zero page address $00LL
            // just fetch the low-byte and assign the contents to operand
            operand = memPoint(fetch());
            break;
        case ZPGX:
            // operand is zeropage address $00LL
            // effective address is address incremented by X without carry
            // the address wraps around if it exceeds 0xFF
            targetAddress = (fetch() + x) & 0xFF;
            operand = memPoint(targetAddress);
            break;
        case ZPGY:
            // same as ZPGX but with Y register
            targetAddress = (fetch() + y) & 0xFF;
            operand = memPoint(targetAddress);
            break;
        case ABSX:
            // operand is address $HHLL + X + C;
            // effective address is adress incremented by X with carry
            // add 1 to the cycles if the page is crossed
            lo = fetch();
            hi = fetch();
            targetAddress = ((hi << 8) | lo) + x;

            // Checks if the page boundary is crossed.
            // In 16-bit addressing, first 8-bits is considered a page, and
            // the last are the index. If the original page read from the
            // memory (hi << 8) is not the same as the final address masked with
            // 0xFF00 that means the page boundary is crossed.
            // only some instructions add an extra cycle if the page boundary is
            // crossed so we and it with our instructions pageCycles
            cycles += (currentInstr.pageCycles) & ((hi << 8) ^ (targetAddress & 0xFF00));
            
            operand = memPoint(targetAddress);
            break;
        case ABSY:
            // Same as ABSX, but with Y register
            lo = fetch();
            hi = fetch();
            targetAddress = ((hi << 8) | lo) + y;

            
            cycles += (currentInstr.pageCycles) & ((hi << 8) ^ (targetAddress & 0xFF00));
            
            operand = memPoint(targetAddress);
            break;
        case IMPL:
            // operand is implied
            // every instruction that uses implied is 1-byte in size
            // except BRK Force Break
            break;
        case REL:
            // branch target is PC + signed offset BB
            // whether or not the branch offset is gonna be applied
            // depends on whether or not the condition in the instruction
            // is satisfied. operand's value should be casted to
            // int8_t before getting added to the pc because it's considered
            // a signed value. Also if a page transition occurs it adds one more cycle
            // to the instruction this could be handled inside the instruction itself.
            // Because it's not for certain that the branching is going to happen
            fetch();
            operand = &fetched;
            break;
        case IND:
            // operand is address $HHLL; 
            //effective address is contents of the word at address
            // this is only used for the JMP instruction

            // get address
            lo = fetch();
            hi = fetch();
            targetAddress = (hi << 8) | (lo);
            // read the values from address and address+1
            lo = memRead(targetAddress);
            hi = memRead(targetAddress + 0x0001);
            // which then is our IND address
            targetAddress = (hi << 8) | (lo);
            break;
        case XIND:
            // operand is zeropage address;
            // effective address is word in (LL + X, LL + X + 1),
            // inc. without carry: C.w($00LL + X)

            // fetch the zero page address
            lo = fetch();

            // add the contents of the X register to the
            // address and mask it with 0xFF, this causes a "wrap around"
            // if the (lo + x) is out of the zero page boundary
            targetAddress = (lo + x) & 0xFF;

            // low byte is read from memory (zeropage)
            lo = memRead(targetAddress);
            // high byte is he next address
            hi = memRead(targetAddress + 0x01);
            
            // this is our indirect address
            // operand points to the contents of that address
            targetAddress = ((hi << 8) | lo);
            operand = memPoint(targetAddress);
            break;
        case INDY:
            // operand is zeropage address; 
            // effective address is word in (LL, LL + 1)
            // incremented by Y with carry: C.w($00LL) + Y
            
            // fetch the zero page address to get the effective address.
            // low is the address that points to the low-bytes of the
            // effective address
            lo = fetch();

            // since address words are little endian
            // lo+1 points to the high-byte of the effective address
            // and contents of lo + y points to the low-byte of the effective address
            hi = memRead(lo+1);
            lo = (memRead(lo) + y);
            targetAddress = (hi << 8) + lo;
            cycles += (currentInstr.pageCycles) & ((hi << 8) ^ (targetAddress & 0xFF00));

            // now that we have our effective address we can point
            // our operand to the contents of that address.
            operand = memPoint(targetAddress);
            break;
    }
}

void cpu::execute(){
    // currentInstr.instr is a pointer to cpu::instruction()
    // member functions need an object to call them, in this case
    // the object is what "this" points to.
    (this->*currentInstr.instr)();
}

void cpu::clock(){ // this could be called from the main loop 
    if (cycles == 0){
        fetch();
        decode();
        execute();
    }
    cycles--;
}