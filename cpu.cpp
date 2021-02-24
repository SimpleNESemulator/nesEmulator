#include "cpu.hpp"

<<<<<<< HEAD
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
    if (!(status & C)){
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
};
void cpu::BCS(){};
void cpu::BEQ(){};
void cpu::BIT(){};
void cpu::BMI(){};
void cpu::BNE(){};
void cpu::BPL(){};
void cpu::BRK(){};
void cpu::BVC(){};
void cpu::BVS(){};
void cpu::CLC(){};
void cpu::CLD(){};
void cpu::CLI(){};
void cpu::CLV(){}
void cpu::CMP(){};
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
void cpu::LDA(){};
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


/*
    instruction is a struct that contains:
    instr_ptr instr -> pointer to a member function of the cpu class (cpu::*ptr)
    addressingMode addressing -> enum for different addressing modes
    uint8_t size -> byte size of the instruction
    uint8_t cycles -> amount of cycles the instruction takes the complete
    uint8_t pageCycles -> amount of extra cycles to be added if the page boundary of the address is crossed
*/
instruction instructions[] = {
	{&cpu::BRK,IMPL,1,7,0},	{&cpu::ORA,XIND,2,6,0},	{&cpu::KIL,XXX,1,2,0},	{&cpu::SLO,XXX,2,8,0},
	{&cpu::NOP,XXX,2,3,0},	{&cpu::ORA,ZPG,2,3,0},	{&cpu::ASL,ZPG,2,5,0},	{&cpu::SLO,XXX,2,5,0},
	{&cpu::PHP,IMPL,1,3,0},	{&cpu::ORA,IMM,2,2,0},	{&cpu::ASL,ACC,1,2,0},	{&cpu::ANC,XXX,2,2,0},
	{&cpu::NOP,XXX,3,4,0},	{&cpu::ORA,ABS,3,4,0},	{&cpu::ASL,ABS,3,6,0},	{&cpu::SLO,XXX,3,6,0},
	{&cpu::BPL,REL,2,2,1},	{&cpu::ORA,INDY,2,5,1},	{&cpu::KIL,XXX,1,2,0},	{&cpu::SLO,XXX,2,8,0},
	{&cpu::NOP,XXX,2,4,0},	{&cpu::ORA,ZPGX,2,4,0},	{&cpu::ASL,ZPGX,2,6,0},	{&cpu::SLO,XXX,2,6,0},
	{&cpu::CLC,IMPL,1,2,0},	{&cpu::ORA,ABSY,3,4,1},	{&cpu::NOP,XXX,1,2,0},	{&cpu::SLO,XXX,3,7,0},
	{&cpu::NOP,XXX,3,4,1},	{&cpu::ORA,ABSX,3,4,1},	{&cpu::ASL,ABSX,3,7,0},	{&cpu::SLO,XXX,3,7,0},
	{&cpu::JSR,ABS,3,6,0},	{&cpu::AND,XIND,2,6,0},	{&cpu::KIL,XXX,1,2,0},	{&cpu::RLA,XXX,2,8,0},
	{&cpu::BIT,ZPG,2,3,0},	{&cpu::AND,ZPG,2,3,0},	{&cpu::ROL,ZPG,2,5,0},	{&cpu::RLA,XXX,2,5,0},
	{&cpu::PLP,IMPL,1,4,0},	{&cpu::AND,IMM,2,2,0},	{&cpu::ROL,ACC,1,2,0},	{&cpu::ANC,XXX,2,2,0},
	{&cpu::BIT,ABS,3,4,0},	{&cpu::AND,ABS,3,4,0},	{&cpu::ROL,ABS,3,6,0},	{&cpu::RLA,XXX,3,6,0},
	{&cpu::BMI,REL,2,2,1},	{&cpu::AND,INDY,2,5,1},	{&cpu::KIL,XXX,1,2,0},	{&cpu::RLA,XXX,2,8,0},
	{&cpu::NOP,XXX,2,4,0},	{&cpu::AND,ZPGX,2,4,0},	{&cpu::ROL,ZPGX,2,6,0},	{&cpu::RLA,XXX,2,6,0},
	{&cpu::SEC,IMPL,1,2,0},	{&cpu::AND,ABSY,3,4,1},	{&cpu::NOP,XXX,1,2,0},	{&cpu::RLA,XXX,3,7,0},
	{&cpu::NOP,XXX,3,4,1},	{&cpu::AND,ABSX,3,4,1},	{&cpu::ROL,ABSX,3,7,0},	{&cpu::RLA,XXX,3,7,0},
	{&cpu::RTI,IMPL,1,6,0},	{&cpu::EOR,XIND,2,6,0},	{&cpu::KIL,XXX,1,2,0},	{&cpu::SRE,XXX,2,8,0},
	{&cpu::NOP,XXX,2,3,0},	{&cpu::EOR,ZPG,2,3,0},	{&cpu::LSR,ZPG,2,5,0},	{&cpu::SRE,XXX,2,5,0},
	{&cpu::PHA,IMPL,1,3,0},	{&cpu::EOR,IMM,2,2,0},	{&cpu::LSR,ACC,1,2,0},	{&cpu::ALR,XXX,2,2,0},
	{&cpu::JMP,ABS,3,3,0},	{&cpu::EOR,ABS,3,4,0},	{&cpu::LSR,ABS,3,6,0},	{&cpu::SRE,XXX,3,6,0},
	{&cpu::BVC,REL,2,2,1},	{&cpu::EOR,INDY,2,5,1},	{&cpu::KIL,XXX,1,2,0},	{&cpu::SRE,XXX,2,8,0},
	{&cpu::NOP,XXX,2,4,0},	{&cpu::EOR,ZPGX,2,4,0},	{&cpu::LSR,ZPGX,2,6,0},	{&cpu::SRE,XXX,2,6,0},
	{&cpu::CLI,IMPL,1,2,0},	{&cpu::EOR,ABSY,3,4,1},	{&cpu::NOP,XXX,1,2,0},	{&cpu::SRE,XXX,3,7,0},
	{&cpu::NOP,XXX,3,4,1},	{&cpu::EOR,ABSX,3,4,1},	{&cpu::LSR,ABSX,3,7,0},	{&cpu::SRE,XXX,3,7,0},
	{&cpu::RTS,IMPL,1,6,0},	{&cpu::ADC,XIND,2,6,0},	{&cpu::KIL,XXX,1,2,0},	{&cpu::RRA,XXX,2,8,0},
	{&cpu::NOP,XXX,2,3,0},	{&cpu::ADC,ZPG,2,3,0},	{&cpu::ROR,ZPG,2,5,0},	{&cpu::RRA,XXX,2,5,0},
	{&cpu::PLA,IMPL,1,4,0},	{&cpu::ADC,IMM,2,2,0},	{&cpu::ROR,ACC,1,2,0},	{&cpu::ARR,XXX,2,2,0},
	{&cpu::JMP,IND,3,5,0},	{&cpu::ADC,ABS,3,4,0},	{&cpu::ROR,ABS,3,6,0},	{&cpu::RRA,XXX,3,6,0},
	{&cpu::BVS,REL,2,2,1},	{&cpu::ADC,INDY,2,5,1},	{&cpu::KIL,XXX,1,2,0},	{&cpu::RRA,XXX,2,8,0},
	{&cpu::NOP,XXX,2,4,0},	{&cpu::ADC,ZPGX,2,4,0},	{&cpu::ROR,ZPGX,2,6,0},	{&cpu::RRA,XXX,2,6,0},
	{&cpu::SEI,IMPL,1,2,0},	{&cpu::ADC,ABSY,3,4,1},	{&cpu::NOP,XXX,1,2,0},	{&cpu::RRA,XXX,3,7,0},
	{&cpu::NOP,XXX,3,4,1},	{&cpu::ADC,ABSX,3,4,1},	{&cpu::ROR,ABSX,3,7,0},	{&cpu::RRA,XXX,3,7,0},
	{&cpu::NOP,XXX,2,2,0},	{&cpu::STA,XIND,2,6,0},	{&cpu::NOP,XXX,2,2,0},	{&cpu::SAX,XXX,2,6,0},
	{&cpu::STY,ZPG,2,3,0},	{&cpu::STA,ZPG,2,3,0},	{&cpu::STX,ZPG,2,3,0},	{&cpu::SAX,XXX,2,3,0},
	{&cpu::DEY,IMPL,1,2,0},	{&cpu::NOP,XXX,2,2,0},	{&cpu::TXA,IMPL,1,2,0},	{&cpu::XAA,XXX,2,2,0},
	{&cpu::STY,ABS,3,4,0},	{&cpu::STA,ABS,3,4,0},	{&cpu::STX,ABS,3,4,0},	{&cpu::SAX,XXX,3,4,0},
	{&cpu::BCC,REL,2,2,1},	{&cpu::STA,INDY,2,6,0},	{&cpu::KIL,XXX,1,2,0},	{&cpu::AHX,XXX,2,6,0},
	{&cpu::STY,ZPGX,2,4,0},	{&cpu::STA,ZPGX,2,4,0},	{&cpu::STX,ZPGY,2,4,0},	{&cpu::SAX,XXX,2,4,0},
	{&cpu::TYA,IMPL,1,2,0},	{&cpu::STA,ABSY,3,5,0},	{&cpu::TXS,IMPL,1,2,0},	{&cpu::TAS,XXX,3,5,0},
	{&cpu::SHY,XXX,3,5,0},	{&cpu::STA,ABSX,3,5,0},	{&cpu::SHX,XXX,3,5,0},	{&cpu::AHX,XXX,3,5,0},
	{&cpu::LDY,IMM,2,2,0},	{&cpu::LDA,XIND,2,6,0},	{&cpu::LDX,IMM,2,2,0},	{&cpu::LAX,XXX,2,6,0},
	{&cpu::LDY,ZPG,2,3,0},	{&cpu::LDA,ZPG,2,3,0},	{&cpu::LDX,ZPG,2,3,0},	{&cpu::LAX,XXX,2,3,0},
	{&cpu::TAY,IMPL,1,2,0},	{&cpu::LDA,IMM,2,2,0},	{&cpu::TAX,IMPL,1,2,0},	{&cpu::LAX,XXX,2,2,0},
	{&cpu::LDY,ABS,3,4,0},	{&cpu::LDA,ABS,3,4,0},	{&cpu::LDX,ABS,3,4,0},	{&cpu::LAX,XXX,3,4,0},
	{&cpu::BCS,REL,2,2,1},	{&cpu::LDA,INDY,2,5,1},	{&cpu::KIL,XXX,1,2,0},	{&cpu::LAX,XXX,2,5,1},
	{&cpu::LDY,ZPGX,2,4,0},	{&cpu::LDA,ZPGX,2,4,0},	{&cpu::LDX,ZPGY,2,4,0},	{&cpu::LAX,XXX,2,4,0},
	{&cpu::CLV,IMPL,1,2,0},	{&cpu::LDA,ABSY,3,4,1},	{&cpu::TSX,IMPL,1,2,0},	{&cpu::LAS,XXX,3,4,1},
	{&cpu::LDY,ABSX,3,4,1},	{&cpu::LDA,ABSX,3,4,1},	{&cpu::LDX,ABSY,3,4,1},	{&cpu::LAX,XXX,3,4,1},
	{&cpu::CPY,IMM,2,2,0},	{&cpu::CMP,XIND,2,6,0},	{&cpu::NOP,XXX,2,2,0},	{&cpu::DCP,XXX,2,8,0},
	{&cpu::CPY,ZPG,2,3,0},	{&cpu::CMP,ZPG,2,3,0},	{&cpu::DEC,ZPG,2,5,0},	{&cpu::DCP,XXX,2,5,0},
	{&cpu::INY,IMPL,1,2,0},	{&cpu::CMP,IMM,2,2,0},	{&cpu::DEX,IMPL,1,2,0},	{&cpu::AXS,XXX,2,2,0},
	{&cpu::CPY,ABS,3,4,0},	{&cpu::CMP,ABS,3,4,0},	{&cpu::DEC,ABS,3,6,0},	{&cpu::DCP,XXX,3,6,0},
	{&cpu::BNE,REL,2,2,1},	{&cpu::CMP,INDY,2,5,1},	{&cpu::KIL,XXX,1,2,0},	{&cpu::DCP,XXX,2,8,0},
	{&cpu::NOP,XXX,2,4,0},	{&cpu::CMP,ZPGX,2,4,0},	{&cpu::DEC,ZPGX,2,6,0},	{&cpu::DCP,XXX,2,6,0},
	{&cpu::CLD,IMPL,1,2,0},	{&cpu::CMP,ABSY,3,4,1},	{&cpu::NOP,XXX,1,2,0},	{&cpu::DCP,XXX,3,7,0},
	{&cpu::NOP,XXX,3,4,1},	{&cpu::CMP,ABSX,3,4,1},	{&cpu::DEC,ABSX,3,7,0},	{&cpu::DCP,XXX,3,7,0},
	{&cpu::CPX,IMM,2,2,0},	{&cpu::SBC,XIND,2,6,0},	{&cpu::NOP,XXX,2,2,0},	{&cpu::ISC,XXX,2,8,0},
	{&cpu::CPX,ZPG,2,3,0},	{&cpu::SBC,ZPG,2,3,0},	{&cpu::INC,ZPG,2,5,0},	{&cpu::ISC,XXX,2,5,0},
	{&cpu::INX,IMPL,1,2,0},	{&cpu::SBC,IMM,2,2,0},	{&cpu::NOP,IMPL,1,2,0},	{&cpu::SBC,XXX,2,2,0},
	{&cpu::CPX,ABS,3,4,0},	{&cpu::SBC,ABS,3,4,0},	{&cpu::INC,ABS,3,6,0},	{&cpu::ISC,XXX,3,6,0},
	{&cpu::BEQ,REL,2,2,1},	{&cpu::SBC,INDY,2,5,1},	{&cpu::KIL,XXX,1,2,0},	{&cpu::ISC,XXX,2,8,0},
	{&cpu::NOP,XXX,2,4,0},	{&cpu::SBC,ZPGX,2,4,0},	{&cpu::INC,ZPGX,2,6,0},	{&cpu::ISC,XXX,2,6,0},
	{&cpu::SED,IMPL,1,2,0},	{&cpu::SBC,ABSY,3,4,1},	{&cpu::NOP,XXX,1,2,0},	{&cpu::ISC,XXX,3,7,0},
	{&cpu::NOP,XXX,3,4,1},	{&cpu::SBC,ABSX,3,4,1},	{&cpu::INC,ABSX,3,7,0},	{&cpu::ISC,XXX,3,7,0},
};

uint8_t cpu::fetch(){
    // just fetch the current address
    fetched = memRead(pc++); 
    return fetched;
}

void cpu::decode(){     
    // once we have fetched we can assign our current instruction
    // we assign it to our currentInstr
    currentInstr = instructions[fetched];
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
            // 
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
=======
// instr_ptr is a pointer to a function that takes in no arguments
// and has the return type void
typedef void (*instr_ptr) ();
cpuStruct cpu;

void ADC(){};
void AND(){};
void ASL(){};
void BCC(){};
void BCS(){};
void BEQ(){};
void BIT(){};
void BMI(){};
void BNE(){};
void BPL(){};
void BRK(){std::cout<<"hello"<<std::endl;};
void BVC(){};
void BVS(){};
void CLC(){};
void CLD(){};
void CLI(){};
void CLV(){};
void CMP(){};
void CPX(){};
void CPY(){};
void DEC(){};
void DEX(){};
void DEY(){};
void EOR(){};
void INC(){};
void INX(){};
void INY(){};
void JMP(){};
void JSR(){};
void LDA(){};
void LDX(){};
void LDY(){};
void LSR(){};
void NOP(){};
void ORA(){};
void PHA(){};
void PHP(){};
void PLA(){};
void PLP(){};
void ROL(){};
void ROR(){};
void RIT(){};
void RTS(){};
void SBC(){};
void SEC(){};
void SED(){};
void SEI(){};
void STA(){};
void STX(){};
void STY(){};
void TAX(){};
void TAY(){};
void TSX(){};
void TXA(){};
void TXS(){};
void TYA(){};
//Illegal opcodes
void KIL(){};
void SLO(){};
void RLA(){};
void SRE(){};
void RTI(){};
void RRA(){};
void SAX(){};
void AHX(){};
void LAX(){};
void DCP(){};
void ISC(){};
void ANC(){};
void ALR(){};
void ARR(){};
void XAA(){};
void SHY(){};
void TAS(){};
void SHX(){};
void LAS(){};
void AXS(){};

// array of pointers to instruction functions
// by default function name without the paranthesis
// gives you the address.

void (*instrPtr[256]) () = {
    BRK,ORA,KIL,SLO,NOP,ORA,ASL,SLO,PHP,ORA,ASL,ANC,NOP,ORA,ASL,SLO,
    BPL,ORA,KIL,SLO,NOP,ORA,ASL,SLO,CLC,ORA,NOP,SLO,NOP,ORA,ASL,SLO,
    JSR,AND,KIL,RLA,BIT,AND,ROL,RLA,PLP,AND,ROL,ANC,BIT,AND,ROL,RLA,
    BMI,AND,KIL,RLA,NOP,AND,ROL,RLA,SEC,AND,NOP,RLA,NOP,AND,ROL,RLA,
    RTI,EOR,KIL,SRE,NOP,EOR,LSR,SRE,PHA,EOR,LSR,ALR,JMP,EOR,LSR,SRE,
    BVC,EOR,KIL,SRE,NOP,EOR,LSR,SRE,CLI,EOR,NOP,SRE,NOP,EOR,LSR,SRE,
    RTS,ADC,KIL,RRA,NOP,ADC,ROR,RRA,PLA,ADC,ROR,ARR,JMP,ADC,ROR,RRA,
    BVS,ADC,KIL,RRA,NOP,ADC,ROR,RRA,SEI,ADC,NOP,RRA,NOP,ADC,ROR,RRA,
    NOP,STA,NOP,SAX,STY,STA,STX,SAX,DEY,NOP,TXA,XAA,STY,STA,STX,SAX,
    BCC,STA,KIL,AHX,STY,STA,STX,SAX,TYA,STA,TXS,TAS,SHY,STA,SHX,AHX,
    LDY,LDA,LDX,LAX,LDY,LDA,LDX,LAX,TAY,LDA,TAX,LAX,LDY,LDA,LDX,LAX,
    BCS,LDA,KIL,LAX,LDY,LDA,LDX,LAX,CLV,LDA,TSX,LAS,LDY,LDA,LDX,LAX,
    CPY,CMP,NOP,DCP,CPY,CMP,DEC,DCP,INY,CMP,DEX,AXS,CPY,CMP,DEC,DCP,
    BNE,CMP,KIL,DCP,NOP,CMP,DEC,DCP,CLD,CMP,NOP,DCP,NOP,CMP,DEC,DCP,
    CPX,SBC,NOP,ISC,CPX,SBC,INC,ISC,INX,SBC,NOP,SBC,CPX,SBC,INC,ISC,
    BEQ,SBC,KIL,ISC,NOP,SBC,INC,ISC,SED,SBC,NOP,ISC,NOP,SBC,INC,ISC,
};

//instruction opcode array
char instrCode[256][4] = {
    "BRK","ORA","KIL","SLO","NOP","ORA","ASL","SLO","PHP","ORA","ASL","ANC","NOP","ORA","ASL","SLO",
    "BPL","ORA","KIL","SLO","NOP","ORA","ASL","SLO","CLC","ORA","NOP","SLO","NOP","ORA","ASL","SLO",
    "JSR","AND","KIL","RLA","BIT","AND","ROL","RLA","PLP","AND","ROL","ANC","BIT","AND","ROL","RLA",
    "BMI","AND","KIL","RLA","NOP","AND","ROL","RLA","SEC","AND","NOP","RLA","NOP","AND","ROL","RLA",
    "RTI","EOR","KIL","SRE","NOP","EOR","LSR","SRE","PHA","EOR","LSR","ALR","JMP","EOR","LSR","SRE",
    "BVC","EOR","KIL","SRE","NOP","EOR","LSR","SRE","CLI","EOR","NOP","SRE","NOP","EOR","LSR","SRE",
    "RTS","ADC","KIL","RRA","NOP","ADC","ROR","RRA","PLA","ADC","ROR","ARR","JMP","ADC","ROR","RRA",
    "BVS","ADC","KIL","RRA","NOP","ADC","ROR","RRA","SEI","ADC","NOP","RRA","NOP","ADC","ROR","RRA",
    "NOP","STA","NOP","SAX","STY","STA","STX","SAX","DEY","NOP","TXA","XAA","STY","STA","STX","SAX",
    "BCC","STA","KIL","AHX","STY","STA","STX","SAX","TYA","STA","TXS","TAS","SHY","STA","SHX","AHX",
    "LDY","LDA","LDX","LAX","LDY","LDA","LDX","LAX","TAY","LDA","TAX","LAX","LDY","LDA","LDX","LAX",
    "BCS","LDA","KIL","LAX","LDY","LDA","LDX","LAX","CLV","LDA","TSX","LAS","LDY","LDA","LDX","LAX",
    "CPY","CMP","NOP","DCP","CPY","CMP","DEC","DCP","INY","CMP","DEX","AXS","CPY","CMP","DEC","DCP",
    "BNE","CMP","KIL","DCP","NOP","CMP","DEC","DCP","CLD","CMP","NOP","DCP","NOP","CMP","DEC","DCP",
    "CPX","SBC","NOP","ISC","CPX","SBC","INC","ISC","INX","SBC","NOP","SBC","CPX","SBC","INC","ISC",
    "BEQ","SBC","KIL","ISC","NOP","SBC","INC","ISC","SED","SBC","NOP","ISC","NOP","SBC","INC","ISC",
};
>>>>>>> c74e091043b362ffa76f5dfa9d03816fb88a3967

//addressing mode array
int instrMode[256] = {
    imp,inx,xxx,xxx,xxx,zpg,zpg,xxx,imp,imm,acc,xxx,xxx,abl,abl,xxx,
    rel,iny,xxx,xxx,xxx,zpx,zpx,xxx,imp,aby,xxx,xxx,xxx,abx,abx,xxx,
    abl,inx,xxx,xxx,zpg,zpg,zpg,xxx,imp,imm,acc,xxx,abl,abl,abl,xxx,
    rel,iny,xxx,xxx,xxx,zpx,zpx,xxx,imp,aby,xxx,xxx,xxx,abx,abx,xxx,
    imp,inx,xxx,xxx,xxx,zpg,zpg,xxx,imp,imm,acc,xxx,abl,abl,abl,xxx,
    rel,iny,xxx,xxx,xxx,zpx,zpx,xxx,imp,aby,xxx,xxx,xxx,abx,abx,xxx,
    imp,inx,xxx,xxx,xxx,zpg,zpg,xxx,imp,imm,acc,xxx,ind,abl,abl,xxx,
    rel,iny,xxx,xxx,xxx,zpx,zpx,xxx,imp,aby,xxx,xxx,xxx,abx,abx,xxx,
    xxx,inx,xxx,xxx,zpg,zpg,zpg,xxx,imp,xxx,imp,xxx,abl,abl,abl,xxx,
    rel,iny,xxx,xxx,zpx,zpx,zpy,xxx,imp,aby,imp,xxx,xxx,abx,xxx,xxx,
    imm,inx,imm,xxx,zpg,zpg,zpg,xxx,imp,imm,imp,xxx,abl,abl,abl,xxx,
    rel,iny,xxx,xxx,zpx,zpx,zpy,xxx,imp,aby,imp,xxx,abx,abx,aby,xxx,
    imm,inx,xxx,xxx,zpg,zpg,zpg,xxx,imp,imm,imp,xxx,abl,abl,abl,xxx,
    rel,iny,xxx,xxx,xxx,zpx,zpx,xxx,imp,aby,xxx,xxx,xxx,abx,abx,xxx,
    imm,inx,xxx,xxx,zpg,zpg,zpg,xxx,imp,imm,imp,xxx,abl,abl,abl,xxx,
    rel,iny,xxx,xxx,xxx,zpx,zpx,xxx,imp,aby,xxx,xxx,xxx,abx,abx,xxx
};

//instr cycles
int instrCyc[256] = {
    7,6,2,8,3,3,5,5,3,2,2,2,4,4,6,6,
    2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
    6,6,2,8,3,3,5,5,4,2,2,2,4,4,6,6,
    2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
    6,6,2,8,3,3,5,5,3,2,2,2,3,4,6,6,
    2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
    6,6,2,8,3,3,5,5,4,2,2,2,5,4,6,6,
    2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
    2,6,2,6,3,3,3,3,2,2,2,2,4,4,4,4,
    2,6,2,6,4,4,4,4,2,5,2,5,5,5,5,5,
    2,6,2,6,3,3,3,3,2,2,2,2,4,4,4,4,
    2,5,2,5,4,4,4,4,2,4,2,4,4,4,4,4,
    2,6,2,8,3,3,5,5,2,2,2,2,4,4,6,6,
    2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
    2,6,2,8,3,3,5,5,2,2,2,2,4,4,6,6,
    2,5,2,8,4,4,6,6,2,4,2,7,4,4,7,7,
};

//instr cycles after crossing page boundary 
int instrPageCyc[256] = {
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    1,1,0,0,0,0,0,0,0,1,0,0,1,1,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    1,1,0,0,0,0,0,0,0,1,0,0,1,1,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    1,1,0,0,0,0,0,0,0,1,0,0,1,1,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    1,1,0,0,0,0,0,0,0,1,0,0,1,1,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    1,1,0,1,0,0,0,0,0,1,0,1,1,1,1,1,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    1,1,0,0,0,0,0,0,0,1,0,0,1,1,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    1,1,0,0,0,0,0,0,0,1,0,0,1,1,0,0,
};

int instrSize[256] = {
    1,2,1,2,2,2,2,2,1,2,1,2,3,3,3,3,
    2,2,1,2,2,2,2,2,1,3,1,3,3,3,3,3,
    3,2,1,2,2,2,2,2,1,2,1,2,3,3,3,3,
    2,2,1,2,2,2,2,2,1,3,1,3,3,3,3,3,
    1,2,1,2,2,2,2,2,1,2,1,2,3,3,3,3,
    2,2,1,2,2,2,2,2,1,3,1,3,3,3,3,3,
    1,2,1,2,2,2,2,2,1,2,1,2,3,3,3,3,
    2,2,1,2,2,2,2,2,1,3,1,3,3,3,3,3,
    2,2,2,2,2,2,2,2,1,2,1,2,3,3,3,3,
    2,2,1,2,2,2,2,2,1,3,1,3,3,3,3,3,
    2,2,2,2,2,2,2,2,1,2,1,2,3,3,3,3,
    2,2,1,2,2,2,2,2,1,3,1,3,3,3,3,3,
    2,2,2,2,2,2,2,2,1,2,1,2,3,3,3,3,
    2,2,1,2,2,2,2,2,1,3,1,3,3,3,3,3,
    2,2,2,2,2,2,2,2,1,2,1,2,3,3,3,3,
    2,2,1,2,2,2,2,2,1,3,1,3,3,3,3,3,
};


void fetch(){
    cpu.ip = cpu.pc; //instruction pointer points current instruction
    cpu.opcode = memRead(cpu.ip);
    cpu.pc += instrSize[cpu.opcode]; //program counter points next instruction
}
void decode(){
    cpu.addressingMode = instrMode[cpu.opcode]; //addressing mode
    cpu.operand1 = memRead(memRead(cpu.ip+1)); //operand 1 fetch
    cpu.operand2 = memRead(memRead(cpu.ip+2)); //operand 2 fetch
}
void execute(uint8_t opcode){
    instr_ptr instr = instrPtr[opcode];
    instr();
    
}