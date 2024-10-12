#include "hw_types.h"
#include "soc_AM335x.h"
#include "CLOCK_MODULE_Registers.h"

void ckmSetCLKModuleRegister(CLK_MODULE module, CLK_MODULE_REG reg, unsigned int value){
    HWREG(module+reg) |= value;
}

unsigned int ckmGetCLKModuleRegister(CLK_MODULE module, CLK_MODULE_REG reg){
    return HWREG(module+reg);
}

void ckmVerifySetCLKModuleRegister(CLK_MODULE module, CLK_MODULE_REG reg, unsigned int value){
/*
    if((CM_PER <= module) && (module <= 0x44E003FF))
    {
        HWREG(SOC_CM_PER_REGS+module) |= value;  
    }

    if((CM_WKUP <= module) && (module <= 0x44E004FF))
    {
        HWREG(CM_WKUP+module) |= value;  
    }

    if((CM_DPLL <= module) && (module <= 0x44E005FF))
    {
        HWREG(CM_DPLL+module) |= value;  
    }
        
    if((CM_MPU <= module) && (module <= 0x44E006FF))
    {
        HWREG(CM_MPU+module) |= value;  
    }

    if((CM_DEVICE <= module) && (module <= 0x44E007FF))
    {
        HWREG(CM_DEVICE+module) |= value;  
    }

    if((CM_RTC <= module) && (module <= 0x44E008FF))
    {
        HWREG(CM_RTC+module) |= value;  
    }

    if((CM_GFX <= module) && (module <= 0x44E009FF))
    {
        HWREG(CM_GFX+module) |= value;  
    }

    if((CM_CEFUSE <= module) && (module <= 0x44E00AFF))
    {
        HWREG(CM_CEFUSE+module) |= value;  
    }
 */

}