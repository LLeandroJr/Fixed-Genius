#include "hw_types.h"
#include "soc_AM335x.h"
#include "CLOCK_MODULE_Registers.h"
#include "CONTROL_MODULE_Registers.h"
#include "GPIO_Registers.h"
#include "INTERRUPT_SERVICE_ROUTINE.h"
#include "TIMERS_Registers.h"
#include "WATCHDOG_TIMER_Registers.h"
#include "uart.h"
#include "ePWM_Registers.h"


#include "types.h"

#define USR0							(21)	// INTERNAL LED gpio1_21
#define USR1							(22)	// INTERNAL LED gpio1_22
#define USR2							(23)	// INTERNAL LED gpio1_23
#define USR3							(24)	// INTERNAL LED gpio1_24

#define _typesel_PULLUP					(1<<4)		// 1: Pullup selected
#define _typesel_PULLDOWN				(~(1<<4))	// 0: Pulldown selected
#define _puden_enable					(~(1<<3))	// 0: Pullup/pulldown enabled
#define _puden_disable					(1<<3)		// 1: Pullup/pulldown disabled

#define clearTerminal					0, "\033[H\033[J\r", 8


// pwmx.c
//
// pwm test application - 50Hz varying duty cycle for RC-servo control
// activate ehrpwm0A (pin P9_31), ehrpwm0B (pin P9_29), 
// ehrpwm1A (pin P9_14), ehrpwm1B (pin P9_16)
// ehrpwm2A (pin P8_19), ehrpwm2B (pin P8_13)
// also blinks LED USR2 - GPIO1[23]
//
#include "bbbpwm.h"
//
// local functions
//
void GPIOModuleEnab(unsigned int baseAdd) {
	if(baseAdd == SOC_GPIO_1_REGS) {
		// write to MODULEMODE field of CM_PER_GPIO1_CLKCTRL reg
		HWREG(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) |= CM_PER_GPIO1_CLKCTRL_MODULEMODE_ENABLE;
		// wait for MODULEMODE field to attain value
		while(CM_PER_GPIO1_CLKCTRL_MODULEMODE_ENABLE != (HWREG(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) & CM_PER_GPIO1_CLKCTRL_MODULEMODE));

		// write to OPTFCLKEN_GPIO_1_GDBCLK bit in CM_PER_GPIO1_CLKCTRL
		HWREG(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) |= CM_PER_GPIO1_CLKCTRL_OPTFCLKEN_GPIO_1_GDBCLK;
		// wait for OPTFCLKEN_GPIO_1_GDBCLK bit to attain value
		while(CM_PER_GPIO1_CLKCTRL_OPTFCLKEN_GPIO_1_GDBCLK != (HWREG(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) & CM_PER_GPIO1_CLKCTRL_OPTFCLKEN_GPIO_1_GDBCLK));

		// wait for IDLEST field in CM_PER_GPIO1_CLKCTRL register to attain value
		while((CM_PER_GPIO1_CLKCTRL_IDLEST_FUNC << CM_PER_GPIO1_CLKCTRL_IDLEST_SHIFT) != (HWREG(SOC_CM_PER_REGS + CM_PER_GPIO1_CLKCTRL) & CM_PER_GPIO1_CLKCTRL_IDLEST));
		// wait for CLKACTIVITY_GPIO_1_GDBCLK bit in CM_PER_L4LS_CLKSTCTRL reg to attain value
		while(CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_GPIO_1_GDBCLK != (HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) & CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_GPIO_1_GDBCLK));
	}
	// clear DISABLEMODULE bit in CTRL register (enab the module)
	HWREG(baseAdd + GPIO_CTRL) &= ~(GPIO_CTRL_DISABLEMODULE);
	// set SOFTRESET bit in System Configuration reg
	HWREG(baseAdd + GPIO_SYSCONFIG) |= (GPIO_SYSCONFIG_SOFTRESET);
	// wait until GPIO Module is reset
	while(!(HWREG(baseAdd + GPIO_SYSSTATUS) & GPIO_SYSSTATUS_RESETDONE));
}

void GPIODirectionSet(unsigned int baseAdd, unsigned int pinNo, unsigned int pinDir) {
	// check if pin is to be an output
	if(GPIO_DIR_OUTPUT == pinDir) {
		HWREG(baseAdd + GPIO_OE) &= ~(1 << pinNo);
	} else {
		HWREG(baseAdd + GPIO_OE) |= (1 << pinNo);
	}
}

void GPIOPinWrite(unsigned int baseAdd, unsigned int pinNo, unsigned int pinValue) {
	if(pinValue == 1) {
		HWREG(baseAdd + GPIO_SETDATAOUT) = (1 << pinNo);
	} else {
		HWREG(baseAdd + GPIO_CLEARDATAOUT) = (1 << pinNo);
	}
}
//
// simple PinMux setup
//
// GPIOaddr - GPIO address from pin_mux.h (and Mode7 column of Header Table in BBB_SRM)
// modeValue - select mode 0 to 7
//
void GPIOPinMuxSetup(unsigned int GPIOaddr, unsigned int modeValue) {
	HWREG(SOC_CONTROL_REGS + GPIOaddr) = (modeValue);
}
//
// set clock divider of time-based clock
//
// see TBCTL register in TRM
//
// prescale - time-based clock prescale value 
// choose a value = 14 * X (X = 1,2,4,8...128)
//
void PWMTBClkDiv(unsigned int baseAddr, unsigned int prescale) {
	unsigned int clkDiv = prescale;
	unsigned int hspClkDiv;
	unsigned int lspClkDiv, lspClkDivSetting = 0;

	if(clkDiv > EHRPWM_TBCTL_HSPCLKDIV_14) { // 0xE
		hspClkDiv = EHRPWM_TBCTL_HSPCLKDIV_DIVBY14; // 0x7
		lspClkDiv = clkDiv/EHRPWM_TBCTL_HSPCLKDIV_14; 
		while(lspClkDiv > 1) {
			lspClkDiv = lspClkDiv >> 1;
			lspClkDivSetting++;
		}
	} else {
		hspClkDiv = clkDiv/2;
		lspClkDivSetting = EHRPWM_TBCTL_HSPCLKDIV_DIVBY1; // divide by 1
	}

	HWREGH(baseAddr + EHRPWM_TBCTL) = (HWREGH(baseAddr + EHRPWM_TBCTL) & (~EHRPWM_TBCTL_CLKDIV)) | ((lspClkDivSetting << EHRPWM_TBCTL_CLKDIV_SHIFT) & EHRPWM_TBCTL_CLKDIV);
	
	HWREGH(baseAddr + EHRPWM_TBCTL) = (HWREGH(baseAddr + EHRPWM_TBCTL) & (~EHRPWM_TBCTL_HSPCLKDIV)) | ((hspClkDiv << EHRPWM_TBCTL_HSPCLKDIV_SHIFT) & EHRPWM_TBCTL_HSPCLKDIV);
}
//
// set PWM output period (time-based clock period)
//
// see TBPRD register in TRM
//
// pwmPeriod - desired output pwm period in milliseconds 
// eg pwmPeriod = 20 will yield an output frequency of 50Hz
//
void PWMPeriodSet(unsigned int baseAddr, unsigned int pwmPeriod) {
	unsigned int tbPeriod = pwmPeriod*TICKS_PER_MS;
	unsigned int counterDir = EHRPWM_COUNT_UP;
	char enableShadowWrite = EHRPWM_SHADOW_WRITE_ENABLE;
	
	HWREGH(baseAddr + EHRPWM_TBCTL) = 
	(HWREGH(baseAddr + EHRPWM_TBCTL) & (~EHRPWM_PRD_LOAD_SHADOW_MASK))
	| ((enableShadowWrite << EHRPWM_TBCTL_PRDLD_SHIFT) & EHRPWM_PRD_LOAD_SHADOW_MASK);

	HWREGH(baseAddr + EHRPWM_TBCTL) = 
	(HWREGH(baseAddr + EHRPWM_TBCTL) & (~EHRPWM_COUNTER_MODE_MASK)) 
	| ((counterDir <<  EHRPWM_TBCTL_CTRMODE_SHIFT) & EHRPWM_COUNTER_MODE_MASK);
	
	if(EHRPWM_COUNT_UP_DOWN == counterDir) {
		HWREGH(baseAddr + EHRPWM_TBPRD) = (unsigned short)tbPeriod/2;
	} else {
		HWREGH(baseAddr + EHRPWM_TBPRD) = (unsigned short)tbPeriod;
	}
}
//
// load counter compare register A
//
// CMPAVal must be less than TBPRD
// see Counter-Compare Submodule in TRM
//
// this will hang the mpu if u try to load a 
// pwm module that has NOT been initialized
//
char PWMLoadCMPA(unsigned int baseAddr, unsigned int CMPAVal) {
	char enableShadowWrite = EHRPWM_SHADOW_WRITE_ENABLE;
	unsigned int ShadowToActiveLoadTrigger = EHRPWM_COMPA_LOAD_COUNT_EQUAL_ZERO_OR_PERIOD;
	char OverwriteShadowFull = EHRPWM_CMPCTL_OVERWR_SH_FL;
	char status = FALSE;
	
	if((OverwriteShadowFull) || ((HWREGH(baseAddr+EHRPWM_CMPCTL) & EHRPWM_CMPCTL_SHDWAFULL) 
		== EHRPWM_SHADOW_A_EMPTY))
	{

		HWREGH(baseAddr + EHRPWM_CMPCTL) = (HWREGH(baseAddr + EHRPWM_CMPCTL) & (~EHRPWM_CMPCTL_SHDWAMODE))
		| ((enableShadowWrite << EHRPWM_CMPCTL_SHDWAMODE_SHIFT) & EHRPWM_CMPCTL_SHDWAMODE);
		
		HWREGH(baseAddr + EHRPWM_CMPCTL) = (HWREGH(baseAddr + EHRPWM_CMPCTL) & (~EHRPWM_COMPA_LOAD_MASK))
		|((ShadowToActiveLoadTrigger << EHRPWM_CMPCTL_LOADAMODE_SHIFT) & EHRPWM_COMPA_LOAD_MASK);

		HWREGH(baseAddr + EHRPWM_CMPA) = CMPAVal & EHRPWM_CMPA_CMPA;
		status = TRUE;
	}

	return status;
}
//
// load counter compare register B
//
// CMPBVal must be less than TBPRD
// see Counter-Compare Submodule in TRM
//
char PWMLoadCMPB(unsigned int baseAddr, unsigned int CMPBVal) {
	char enableShadowWrite = EHRPWM_SHADOW_WRITE_ENABLE;
	unsigned int ShadowToActiveLoadTrigger = EHRPWM_COMPB_LOAD_COUNT_EQUAL_ZERO_OR_PERIOD;
	char OverwriteShadowFull = EHRPWM_CMPCTL_OVERWR_SH_FL;
	char status = FALSE;
	
	if((OverwriteShadowFull) || ((HWREGH(baseAddr+EHRPWM_CMPCTL) & EHRPWM_CMPCTL_SHDWBFULL) == EHRPWM_SHADOW_B_EMPTY))
	{
		HWREGH(baseAddr + EHRPWM_CMPCTL) = (HWREGH(baseAddr + EHRPWM_CMPCTL) & (~EHRPWM_CMPCTL_SHDWBMODE))
		| ((enableShadowWrite << EHRPWM_CMPCTL_SHDWBMODE_SHIFT) & EHRPWM_CMPCTL_SHDWBMODE);

		HWREGH(baseAddr + EHRPWM_CMPCTL) = (HWREGH(baseAddr + EHRPWM_CMPCTL) & (~EHRPWM_COMPB_LOAD_MASK)) 
		| ((ShadowToActiveLoadTrigger << EHRPWM_CMPCTL_LOADBMODE_SHIFT) & EHRPWM_COMPB_LOAD_MASK);

		HWREGH(baseAddr + EHRPWM_CMPB) = CMPBVal & EHRPWM_CMPB_CMPB;
		status = TRUE;
	}

	return status;
}

void PWMconfigAQActionOnA(unsigned int baseAddr,
	unsigned int zero,
	unsigned int period,
	unsigned int CAUp,
	unsigned int CADown,
	unsigned int CBUp,
	unsigned int CBDown,
	unsigned int SWForced) {
		
	HWREGH(baseAddr + EHRPWM_AQCTLA) =
	((CBDown << EHRPWM_AQCTLA_CBD_SHIFT) & EHRPWM_AQCTLA_CBD) |
	((CBUp << EHRPWM_AQCTLA_CBU_SHIFT) & EHRPWM_AQCTLA_CBU) |
	((CADown << EHRPWM_AQCTLA_CAD_SHIFT) & EHRPWM_AQCTLA_CAD) |
	((CAUp << EHRPWM_AQCTLA_CAU_SHIFT) & EHRPWM_AQCTLA_CAU) |
	((period << EHRPWM_AQCTLA_PRD_SHIFT) & EHRPWM_AQCTLA_PRD) |
	((zero << EHRPWM_AQCTLA_ZRO_SHIFT) & EHRPWM_AQCTLA_ZRO);

	HWREGH(baseAddr + EHRPWM_AQSFRC) = (HWREGH(baseAddr + EHRPWM_AQSFRC) & (~EHRPWM_AQSFRC_ACTSFA)) 
	| ((SWForced << EHRPWM_AQSFRC_ACTSFA_SHIFT) & EHRPWM_AQSFRC_ACTSFA);
}

void PWMconfigAQActionOnB(unsigned int baseAddr,
	unsigned int zero,
	unsigned int period,
	unsigned int CAUp,
	unsigned int CADown,
	unsigned int CBUp,
	unsigned int CBDown,
	unsigned int SWForced) {

	HWREGH(baseAddr + EHRPWM_AQCTLB) =
	((CBDown << EHRPWM_AQCTLB_CBD_SHIFT) & EHRPWM_AQCTLB_CBD) |
	((CBUp << EHRPWM_AQCTLB_CBU_SHIFT) & EHRPWM_AQCTLB_CBU) |
	((CADown << EHRPWM_AQCTLB_CAD_SHIFT) & EHRPWM_AQCTLB_CAD) |
	((CAUp << EHRPWM_AQCTLB_CAU_SHIFT) & EHRPWM_AQCTLB_CAU) |
	((period << EHRPWM_AQCTLB_PRD_SHIFT) & EHRPWM_AQCTLB_PRD) |
	((zero << EHRPWM_AQCTLB_ZRO_SHIFT) & EHRPWM_AQCTLB_ZRO);
	HWREGH(baseAddr + EHRPWM_AQSFRC) =
	(HWREGH(baseAddr + EHRPWM_AQSFRC) & (~EHRPWM_AQSFRC_ACTSFB))
	| ((SWForced << EHRPWM_AQSFRC_ACTSFB_SHIFT) & EHRPWM_AQSFRC_ACTSFB);
}

void meu_PWMconfigAQActionOnB(_uint32 ePWMx,
	_uint32 SWForced,
	_uint32 CBDown,
	_uint32 CBUp,
	_uint32 CADown,
	_uint32 CAUp,
	_uint32 period,
	_uint32 zero) {

	HWREGH(ePWMx + EHRPWM_AQCTLB) =
	((CBDown << EHRPWM_AQCTLB_CBD_SHIFT) & EHRPWM_AQCTLB_CBD) |
	((CBUp << EHRPWM_AQCTLB_CBU_SHIFT) & EHRPWM_AQCTLB_CBU) |
	((CADown << EHRPWM_AQCTLB_CAD_SHIFT) & EHRPWM_AQCTLB_CAD) |
	((CAUp << EHRPWM_AQCTLB_CAU_SHIFT) & EHRPWM_AQCTLB_CAU) |
	((period << EHRPWM_AQCTLB_PRD_SHIFT) & EHRPWM_AQCTLB_PRD) |
	((zero << EHRPWM_AQCTLB_ZRO_SHIFT) & EHRPWM_AQCTLB_ZRO);

	HWREGH(ePWMx + AQSFRC) =
	(HWREGH(ePWMx + EHRPWM_AQSFRC) & (~EHRPWM_AQSFRC_ACTSFB))
	| ((SWForced << EHRPWM_AQSFRC_ACTSFB_SHIFT) & EHRPWM_AQSFRC_ACTSFB);
}

//
// heavily hacked config code from rodrigo in brazil
//
// pwmx: bit1-pwm0, bit2-pwm1, bit3-pwm2 (0-disable or 1-enable PWM module X)
// pwmPeriod: period of pwm output in msec
//

void PWMconfig(unsigned int pwmx, unsigned int pwmPeriod) {
	// config L3_PER and L4_PER clocks
	HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) |= CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP;
	while((HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) & 
	CM_PER_L3S_CLKSTCTRL_CLKTRCTRL) != CM_PER_L3S_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

	HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) |= CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;
	while((HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) & 
	CM_PER_L3_CLKSTCTRL_CLKTRCTRL) != CM_PER_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

	HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) |= CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE;
	while((HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) & 
	CM_PER_L3_INSTR_CLKCTRL_MODULEMODE) != CM_PER_L3_INSTR_CLKCTRL_MODULEMODE_ENABLE);

	HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) |= CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE;
	while((HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) & 
	CM_PER_L3_CLKCTRL_MODULEMODE) != CM_PER_L3_CLKCTRL_MODULEMODE_ENABLE);
	
	HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) |= CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP;
	while((HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) & 
	CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL) != CM_PER_OCPWP_L3_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

	HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) |= CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_WKUP;
	while((HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) & 
	CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL) != CM_PER_L4LS_CLKSTCTRL_CLKTRCTRL_SW_WKUP);

	HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKCTRL) |= CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE;
	while((HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKCTRL) & 
	CM_PER_L4LS_CLKCTRL_MODULEMODE) != CM_PER_L4LS_CLKCTRL_MODULEMODE_ENABLE);
	
	if(pwmx & 0x1) { // config EPWMSS0 clock
		HWREG(SOC_CM_PER_REGS + CM_PER_EPWMSS0_CLKCTRL) |= CM_PER_EPWMSS0_CLKCTRL_MODULEMODE_ENABLE;

		while(CM_PER_EPWMSS0_CLKCTRL_MODULEMODE_ENABLE !=
		(HWREG(SOC_CM_PER_REGS + CM_PER_EPWMSS0_CLKCTRL) & CM_PER_EPWMSS0_CLKCTRL_MODULEMODE));
		while((CM_PER_EPWMSS0_CLKCTRL_IDLEST_FUNC << CM_PER_EPWMSS0_CLKCTRL_IDLEST_SHIFT) !=
		(HWREG(SOC_CM_PER_REGS + CM_PER_EPWMSS0_CLKCTRL) & CM_PER_EPWMSS0_CLKCTRL_IDLEST));
	}

	if(pwmx & 0x2) { // config EPWMSS1 clock
		HWREG(SOC_CM_PER_REGS + CM_PER_EPWMSS1_CLKCTRL) |= CM_PER_EPWMSS1_CLKCTRL_MODULEMODE_ENABLE;

		while(CM_PER_EPWMSS1_CLKCTRL_MODULEMODE_ENABLE != (HWREG(SOC_CM_PER_REGS + CM_PER_EPWMSS1_CLKCTRL) & CM_PER_EPWMSS1_CLKCTRL_MODULEMODE));

		while((CM_PER_EPWMSS1_CLKCTRL_IDLEST_FUNC << CM_PER_EPWMSS1_CLKCTRL_IDLEST_SHIFT) !=
		(HWREG(SOC_CM_PER_REGS + CM_PER_EPWMSS1_CLKCTRL) & CM_PER_EPWMSS1_CLKCTRL_IDLEST));
	}
	
	if(pwmx & 0x4) { // config EPWMSS2 clock
		HWREG(SOC_CM_PER_REGS + CM_PER_EPWMSS2_CLKCTRL) |= CM_PER_EPWMSS2_CLKCTRL_MODULEMODE_ENABLE;

		while(CM_PER_EPWMSS2_CLKCTRL_MODULEMODE_ENABLE != (HWREG(SOC_CM_PER_REGS + CM_PER_EPWMSS2_CLKCTRL) & CM_PER_EPWMSS2_CLKCTRL_MODULEMODE));
		while((CM_PER_EPWMSS2_CLKCTRL_IDLEST_FUNC << CM_PER_EPWMSS2_CLKCTRL_IDLEST_SHIFT) !=
		(HWREG(SOC_CM_PER_REGS + CM_PER_EPWMSS2_CLKCTRL) & CM_PER_EPWMSS2_CLKCTRL_IDLEST));
	}

	// wait on L3 & L4 clock activity
	while(!(HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) & CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK));
	while(!(HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) & CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK));
	while(!(HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) & (CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK | CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L4_GCLK)));
	while(!(HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) & (CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_L4LS_GCLK )));

	if(pwmx & 0x1) { // PWM0
		HWREG(SOC_PWMSS0_REGS + PWMSS_CLKCONFIG) |= PWMSS_EHRPWM_CLK_EN_ACK; // enable PWMSSx clocks
		HWREG(SOC_CONTROL_REGS + CONTROL_PWMSS_CTRL) |= CONTROL_PWMSS_CTRL_PWMSS0_TBCLKEN; // enable Timer Base Module Clock in Control Module
		PWMTBClkDiv(SOC_EPWM_0_REGS, PWM_PRESCALE); // config time-base clock
		PWMPeriodSet(SOC_EPWM_0_REGS, pwmPeriod); // config PWM period
		PWMconfigAQActionOnA(SOC_EPWM_0_REGS, // config Action Qualifiers for PWM0A
		EHRPWM_AQCTLA_ZRO_EPWMXAHIGH, // high when CTR = 0
		EHRPWM_AQCTLA_PRD_DONOTHING,
		EHRPWM_AQCTLA_CAU_EPWMXALOW, // low when CTR = CMPA
		EHRPWM_AQCTLA_CAD_DONOTHING,
		EHRPWM_AQCTLA_CBU_DONOTHING,
		EHRPWM_AQCTLA_CBD_DONOTHING,
		EHRPWM_AQSFRC_ACTSFA_DONOTHING);
		PWMconfigAQActionOnB(SOC_EPWM_0_REGS, // PWM0B
		EHRPWM_AQCTLB_ZRO_EPWMXBHIGH, // high when CTR = 0
		EHRPWM_AQCTLB_PRD_DONOTHING,
		EHRPWM_AQCTLB_CAU_DONOTHING,
		EHRPWM_AQCTLB_CAD_DONOTHING,
		EHRPWM_AQCTLB_CBU_EPWMXBLOW, // low when CTR = CMPB
		EHRPWM_AQCTLB_CBD_DONOTHING,
		EHRPWM_AQSFRC_ACTSFB_DONOTHING);
	}
	if(pwmx & 0x2) { // PWM1
		HWREG(SOC_PWMSS1_REGS + PWMSS_CLKCONFIG) |= PWMSS_EHRPWM_CLK_EN_ACK;
		HWREG(SOC_CONTROL_REGS + CONTROL_PWMSS_CTRL) |= CONTROL_PWMSS_CTRL_PWMSS1_TBCLKEN;
		PWMTBClkDiv(SOC_EPWM_1_REGS, PWM_PRESCALE);
		PWMPeriodSet(SOC_EPWM_1_REGS, pwmPeriod);
		PWMconfigAQActionOnA(SOC_EPWM_1_REGS, // PWM1A
		EHRPWM_AQCTLA_ZRO_EPWMXAHIGH, // high when CTR = 0
		EHRPWM_AQCTLA_PRD_DONOTHING,
		EHRPWM_AQCTLA_CAU_EPWMXALOW, // low when CTR = CMPA
		EHRPWM_AQCTLA_CAD_DONOTHING,
		EHRPWM_AQCTLA_CBU_DONOTHING,
		EHRPWM_AQCTLA_CBD_DONOTHING,
		EHRPWM_AQSFRC_ACTSFA_DONOTHING);
		PWMconfigAQActionOnB(SOC_EPWM_1_REGS, // PWM1B
		EHRPWM_AQCTLB_ZRO_EPWMXBHIGH, // high when CTR = 0
		EHRPWM_AQCTLB_PRD_DONOTHING,
		EHRPWM_AQCTLB_CAU_DONOTHING,
		EHRPWM_AQCTLB_CAD_DONOTHING,
		EHRPWM_AQCTLB_CBU_EPWMXBLOW, // low when CTR = CMPB
		EHRPWM_AQCTLB_CBD_DONOTHING,
		EHRPWM_AQSFRC_ACTSFB_DONOTHING);
	}
	if(pwmx & 0x4) { // PWM2
		HWREG(SOC_PWMSS2_REGS + PWMSS_CLKCONFIG) |= PWMSS_EHRPWM_CLK_EN_ACK;
		HWREG(SOC_CONTROL_REGS + CONTROL_PWMSS_CTRL) |= CONTROL_PWMSS_CTRL_PWMSS2_TBCLKEN;
		PWMTBClkDiv(SOC_EPWM_2_REGS, PWM_PRESCALE);
		PWMPeriodSet(SOC_EPWM_2_REGS, pwmPeriod);
		PWMconfigAQActionOnA(SOC_EPWM_2_REGS, // PWM2A
		EHRPWM_AQCTLA_ZRO_EPWMXAHIGH, // high when CTR = 0
		EHRPWM_AQCTLA_PRD_DONOTHING,
		EHRPWM_AQCTLA_CAU_EPWMXALOW, // low when CTR = CMPA
		EHRPWM_AQCTLA_CAD_DONOTHING,
		EHRPWM_AQCTLA_CBU_DONOTHING,
		EHRPWM_AQCTLA_CBD_DONOTHING,
		EHRPWM_AQSFRC_ACTSFA_DONOTHING);
		PWMconfigAQActionOnB(SOC_EPWM_2_REGS, // PWM2B
		EHRPWM_AQCTLB_ZRO_EPWMXBHIGH, // high when CTR = 0
		EHRPWM_AQCTLB_PRD_DONOTHING,
		EHRPWM_AQCTLB_CAU_DONOTHING,
		EHRPWM_AQCTLB_CAD_DONOTHING,
		EHRPWM_AQCTLB_CBU_EPWMXBLOW, // low when CTR = CMPB
		EHRPWM_AQCTLB_CBD_DONOTHING,
		EHRPWM_AQSFRC_ACTSFB_DONOTHING);
	}

	return;
}
//
// spin your wheels
//
void Delay(volatile unsigned int count) {
	while(count--);
}
//
// main program
//
// move a RC-servo back and forth and watch das-blinken-light!
//
void main() {
	unsigned int i, delta, base, direction, tbprd;

	GPIOModuleEnab(GPIO_ADDR); // enable GPIO module
	GPIODirectionSet(GPIO_ADDR, LED_USR2, GPIO_DIR_OUTPUT); // set GPIO pin as output
	GPIOPinMuxSetup(GPIO_1_23, 7); // muxout LED USR2
	GPIOPinMuxSetup(GPIO_0_22, 4); // pin P8_19 ehrpwm2A
	GPIOPinMuxSetup(GPIO_0_23, 4); // pin P8_13 ehrpwm2B
	GPIOPinMuxSetup(GPIO_1_18, 6); // pin P9_14 ehrpwm1A
	GPIOPinMuxSetup(GPIO_1_19, 6); // pin P9_16 ehrpwm1B
	GPIOPinMuxSetup(GPIO_3_14, 1); // pin P9_31 ehrpwm0A
	GPIOPinMuxSetup(GPIO_3_15, 1); // pin P9_29 ehrpwm0B
	PWMconfig(0x7, PWM_PERIOD_MS); // config PWM 0, 1, 2
	base = (TICKS_PER_MS * PWM_PERIOD_MS)/20; // 1ms base pulse width - RC Servo
	delta = base/10; // .1ms pulse width increment
	i = 4;
	direction = 1;

	while(1) {
		if(direction == 1) i++;
		if(direction == 0) i--;
		tbprd = base + (delta * i);
		
		PWMLoadCMPA(SOC_EPWM_0_REGS, tbprd);
		PWMLoadCMPB(SOC_EPWM_0_REGS, tbprd); 
		PWMLoadCMPA(SOC_EPWM_1_REGS, tbprd);
		PWMLoadCMPB(SOC_EPWM_1_REGS, tbprd); 
		PWMLoadCMPA(SOC_EPWM_2_REGS, tbprd);
		PWMLoadCMPB(SOC_EPWM_2_REGS, tbprd); 

		if(i <= 0) direction = 1;
		if(i >= 10) direction = 0;

		GPIOPinWrite(GPIO_ADDR, LED_USR2, 1);
		Delay(0x2FFFF);
		GPIOPinWrite(GPIO_ADDR, LED_USR2, 0);
		Delay(0x3FFFF);
		GPIOPinWrite(GPIO_ADDR, LED_USR2, 1);
		Delay(0x1FFFF);
		GPIOPinWrite(GPIO_ADDR, LED_USR2, 0);
		Delay(0xAFFFF);
	}
} 

and the header file bbbpwm.h :

//
// bbbpwm.h - pwm library header file
//
//#include "soc_AM335x.h"
#define SOC_GPIO_1_REGS (0x4804C000)
#define SOC_ECAP_REGS (0x00000100) // ecap0 not
#define SOC_ECAP_0_REGS (SOC_PWMSS0_REGS + SOC_ECAP_REGS) // implemented
#define SOC_CONTROL_REGS (0x44E10000)

//#include "beaglebone.h"
//#include "gpio_v2.h"
#define GPIO_DIR_OUTPUT (GPIO_OE_OUTPUTEN_ENABLED)
#define GPIO_OE_OUTPUTEN_ENABLED (0x0u)
#define GPIO_CTRL (0x130)
#define GPIO_CTRL_DISABLEMODULE (0x00000001u)
#define GPIO_SYSCONFIG (0x10)
#define GPIO_SYSCONFIG_SOFTRESET (0x00000002u)
#define GPIO_SYSSTATUS (0x114)
#define GPIO_SYSSTATUS_RESETDONE (0x00000001u)
#define GPIO_OE (0x134)
#define GPIO_CLEARDATAOUT (0x190)
#define GPIO_SETDATAOUT (0x194)

//#include "hw_control_AM335x.h"
#define CONTROL_PWMSS_CTRL (0x664)
#define CONTROL_PWMSS_CTRL_PWMSS0_TBCLKEN (0x00000001u)
#define CONTROL_PWMSS_CTRL_PWMSS1_TBCLKEN (0x00000002u)
#define CONTROL_PWMSS_CTRL_PWMSS2_TBCLKEN (0x00000004u)

#include "ehrpwm.h" // do not remove, too many defines needed here

#define PWMSS_EHRPWM_CLK_EN_ACK 0x100

//#include "evmAM335x.h"

#include "hw_cm_per.h" // do not remove, too many defines needed here

//#include "interrupt.h"

//#include "hw_types.h"
#define HWREG(x) (*((volatile unsigned int *)(x)))
#define HWREGH(x) (*((volatile unsigned short *)(x)))
#define TRUE 1
#define FALSE 0

//#include "pin_mux.h"
#define GPIO_0_22 (0x0820)
#define GPIO_0_23 (0x0824)
#define GPIO_1_18 (0x0848)
#define GPIO_1_19 (0x084c)
#define GPIO_3_14 (0x0990)
#define GPIO_3_15 (0x0994)
#define GPIO_1_23 (0x085c) // spoof LED USR2
//
// internal macros
//
#define GPIO_ADDR SOC_GPIO_1_REGS // for LED USR2
#define LED_USR2 23 // LED USR2
#define SOC_EHRPWM_1_MODULE_FREQ 100000000 // SYSCLKOUT [10 ns/tick]
#define PWM_PRESCALE 224 // pwm clk divider - TBCLK
#define TICKS_PER_MS 446 // ticks per msec pwm output period
#define PWM_PERIOD_MS 20 // desired pwm output period in ms
// PWM_PRESCALE of 224 = 14 * X (X = 1, 2, 4...128), see TBCTL Register
// TICKS_PER_MS * PWM_PERIOD_MS = period of pwm output in ticks = TBPRD (only 16 bits)
// FINITO


