#ifndef _ePWM_Registers_
#define _ePWM_Registers_

#pragma region EPWM REGISTERS

#define EPWM_TBCTL                           (0x0u)  
#define EPWM_TBSTS                           (0x2u)  
#define EPWM_TBPHSHR                         (0x4u)  
#define EPWM_TBPHS                           (0x6u)  
#define EPWM_TBCNT                           (0x8u)  
#define EPWM_TBPRD                           (0xAu)  
#define EPWM_CMPCTL                          (0xEu)  
#define EPWM_CMPAHR                          (0x10u) 
#define EPWM_CMPA                            (0x12u) 
#define EPWM_CMPB                            (0x14u) 
#define EPWM_AQCTLA                          (0x16u) 
#define EPWM_AQCTLB                          (0x18u) 
#define EPWM_AQSFRC                          (0x1Au) 
#define EPWM_AQCSFRC                         (0x1Cu) 
#define EPWM_DBCTL                           (0x1Eu) 
#define EPWM_DBRED                           (0x20u) 
#define EPWM_DBFED                           (0x22u) 
#define EPWM_TZSEL                           (0x24u) 
#define EPWM_TZCTL                           (0x28u) 
#define EPWM_TZEINT                          (0x2Au) 
#define EPWM_TZFLG                           (0x2Cu) 
#define EPWM_TZCLR                           (0x2Eu) 
#define EPWM_TZFRC                           (0x30u) 
#define EPWM_ETSEL                           (0x32u) 
#define EPWM_ETPS                            (0x34u) 
#define EPWM_ETFLG                           (0x36u) 
#define EPWM_ETCLR                           (0x38u) 
#define EPWM_ETFRC                           (0x3Au) 
#define EPWM_PCCTL                           (0x3Cu) 
#define EPWM_HRCNFG                          (0xC0u) 

#pragma endregion EPWM REGISTERS

#endif