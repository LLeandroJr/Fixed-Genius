#include "hw_types.h"
#include "soc_AM335x.h"
#include "CONTROL_MODULE_Registers.h"

void controlModuleSetPad(CONTROL_MODULE offset, unsigned int value){
    // volatile unsigned* addr = (unsigned*)(base+offset);
    // *addr = value;
    HWREG(SOC_CONTROL_REGS+offset) |= value; 
}

unsigned int controlModuleGetPad(CONTROL_MODULE offset){
    // volatile unsigned int* addr = (unsigned int*)(base+offset);
    // return *addr;
    return HWREG(SOC_CONTROL_REGS+offset);
}