#include "cpu.hpp"
#include "mainMemory.hpp"


// instr_ptr is a pointer to a function that takes in no arguments
// and has the return type void
typedef void (*instr_ptr) ();
cpuStruct cpu;

void ADC(){
    uint add = cpu.ac + cpu.operand1 + cpu.C;
    bool carryGenerate = (cpu.ac>>7)&(cpu.operand1>>7); //test carry generation
    bool carryPropagate = (cpu.ac>>7)^(cpu.operand1>>7); //test carry propagation

    cpu.ac = (add & 0xFF);

    //set flag bits
    cpu.C = (carryGenerate || (carryPropagate && cpu.C));
    cpu.V = (carryGenerate != (add & 0x80)>>7);
    cpu.Z = ((add & 0xFF) == 0x00);
    cpu.N = (add & 0x80)>>7;
}
void AND(){

}
void ASL(){};
void BCC(){};
void BCS(){};
void BEQ(){};
void BIT(){};
void BMI(){};
void BNE(){};
void BPL(){};
void BRK(){}
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
uint8_t getFlag(){
    uint8_t flag = ((cpu.N<<7)|(cpu.V<<6)|(cpu.U<<5)|(cpu.B<<4)|(cpu.D<<3)|(cpu.I<<2)|(cpu.Z<<1)|(cpu.C<<0));
    return flag;
}
void cycleAdder(){
    cpu.cycle += instrCyc[cpu.opcode];
    //add cycles if branch taken
    if(cpu.branchFlag){
        cpu.cycle += 1;
    }
    //add cycles if page boundary crossed
    if(cpu.pageBoundaryFlag){
        cpu.cycle += instrPageCyc[cpu.opcode];
    }
    cpu.pageBoundaryFlag = false;
    cpu.branchFlag = false;
}
void fetchInstr(){
    cpu.ip = cpu.pc; //instruction pointer points current instruction
    cpu.opcode = memRead(cpu.ip);
    cpu.pc += instrSize[cpu.opcode]; //program counter points next instruction
}
void decode(){
    cpu.addressingMode = instrMode[cpu.opcode]; //addressing mode
    //read the next two bytes after the instruction
    uint8_t secondByte = memRead(cpu.ip+1);
    uint8_t thirdByte = memRead(cpu.ip+2);
    uint8_t lowByte, highByte;
    uint8_t lowPointer,highPointer;
    switch(cpu.addressingMode){
        case imm:
        {
            cpu.operand1 = secondByte;
            break;
        }
        case zpg:
        {
            cpu.operand1 = memRead(0x00FF&secondByte);
            break;
        }
        case zpx:
        {
            cpu.operand1 = memRead(0x00FF&(secondByte+cpu.x));
            break;
        }
        case zpy:
        {
            cpu.operand1 = memRead(0x00FF&(secondByte+cpu.y));
            break;
        }
        case abl:
        {
            lowByte = secondByte;
            highByte = thirdByte;
            cpu.operand1 = memRead((highByte<<4)|lowByte);
            break;
        }
        case abx:
        {
            lowByte = secondByte;
            highByte = thirdByte;
            cpu.operand1 = memRead(((highByte<<4)|lowByte)+cpu.x);
            break;
        }
        case aby:
        {
            lowByte = secondByte;
            highByte = thirdByte;
            cpu.operand1 = memRead(((highByte<<4)|lowByte)+cpu.y);
            break;
        }
        case ind:
        {
            lowByte = secondByte;
            highByte = thirdByte;
            lowPointer = memRead((highByte<<4)|lowByte);
            highPointer = memRead((highByte<<4)|((lowByte+1)%0x100));
            cpu.targetAddress = (highPointer<<4)|lowPointer; //16 bits, address
            break;
        }
        case inx:
        {
            uint8_t zeroPageOffset = (secondByte+cpu.x)%0x100;
            lowPointer = memRead(0x00FF&zeroPageOffset);
            highPointer = memRead(0x00FF&((zeroPageOffset+1)%0x100));
            cpu.targetAddress = (highPointer<<4)|lowPointer; //16 bits, address
            break;
        }
        case iny:
        {
            lowPointer = memRead(0x00FF&secondByte);
            highPointer = memRead(0x00FF&((secondByte+1)%0x100));
            cpu.targetAddress = ((highPointer<<4)|lowPointer)+cpu.y;
            break;
        }
        case rel:
        {
            int8_t offset = secondByte;
            cpu.targetAddress = cpu.pc+offset;
            break;
        }
    }
}
void execute(uint8_t opcode){
    instr_ptr instr = instrPtr[opcode];
    instr();
    cycleAdder();
}
