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
#include "PWMSS_Registers.h"


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

// set clock divider of time-based clock
//
// see TBCTL register in TRM
//
// prescale - time-based clock prescale value 
// choose a value = 14 * X (X = 1,2,4,8...128)
//
void PWMTBClkDiv(_uint32 ePWMx, _uint32 prescale) {
	_uint32 clkDiv = prescale;
	_uint32 hspClkDiv;
	_uint32 lspClkDiv, lspClkDivSetting = 0;
	// como a prescale é maior que o maior hight-speed permitido 0x7
	if(clkDiv > 14) { // 0xE
		hspClkDiv = 0x7; // 0x7
		lspClkDiv = clkDiv/14; 
		while(lspClkDiv > 1) {
			lspClkDiv = lspClkDiv >> 1;
			lspClkDivSetting++;
		}
	} else {
		hspClkDiv = clkDiv/2;	
		lspClkDivSetting = 0; // divide by 1
	}

	HWREGH(ePWMx + EPWM_TBCTL) = (HWREGH(ePWMx + EPWM_TBCTL) & (~(7<<10))) | ((lspClkDivSetting & 0x7) << 10);
	
	HWREGH(ePWMx + EPWM_TBCTL) = (HWREGH(ePWMx + EPWM_TBCTL) & (~(7<<7))) | ((hspClkDiv & 0x7) << 7);
}
//
// set PWM output period (time-based clock period)
//
// see TBPRD register in TRM
//
// pwmPeriod - desired output pwm period in milliseconds 
// eg pwmPeriod = 20 will yield an output frequency of 50Hz
//

#define Up_count_mode						(0x0u)
#define Down_count_mode						(0x1u) 
#define CTRMODE_Up_down_count				(0x2u)
#define Stop_freeze_counter					(0x3u)

void PWMPeriodSet(_uint32 ePWMx, _uint32 pwmPeriod) {
	_uint32 tbPeriod = pwmPeriod*TICKS_PER_MS;
	// 	System clock, SYSCLKOUT and TBCLK = 100 MHz, 10 ns
	// For a PWM Period register value of 20 counts, PWM Period = 20 × 10 ns = 200 ns, PWM frequency = 1/200 ns = 1.25 MHz
	// Assumed MEP step size for the above example = 180 ps
	_uint32 CTRMODE_counterDir = 0;
	_uint8 PRDLD_enableShadow = 0; 

	HWREGH(ePWMx + EPWM_TBCTL) = (HWREGH(ePWMx + EPWM_TBCTL) & (~(1<<3))) | ((PRDLD_enableShadow << 3) & (1<<3));

	// HWREGH(ePWMx + EPWM_TBCTL) = 
	// (HWREGH(ePWMx + EPWM_TBCTL) & (~EHRPWM_PRD_LOAD_SHADOW_MASK))
	// | ((enableShadowWrite << 3) & EHRPWM_PRD_LOAD_SHADOW_MASK);

	HWREGH(ePWMx + EPWM_TBCTL) = (HWREGH(ePWMx + EPWM_TBCTL) & (~(1<<0))) | ((CTRMODE_counterDir <<  0) & (1<<0));
	
	if(CTRMODE_Up_down_count == CTRMODE_counterDir) {
		HWREGH(ePWMx + EPWM_TBPRD) = (_uint16)tbPeriod/2;
	} else {
		HWREGH(ePWMx + EPWM_TBPRD) = (_uint16)tbPeriod;
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
// load counter compare register B
//
// CMPBVal must be less than TBPRD
// see Counter-Compare Submodule in TRM
//

_uint8 PWMLoadCMP(_uint32 ePWMx, _uint8 CMPx, _uint32 CMPBVal) {
	_uint8 enableShadowWrite = 0;
	_uint8 ShadowToActiveLoadTrigger = 2;
	_uint8 OverwriteShadowFull = 1;
	_uint8 status = FALSE;
	
	if(CMPx == 'A'){
		if((OverwriteShadowFull) || ((HWREGH(ePWMx + EPWM_CMPCTL) & (1<<9)) == (0<<9)))
		{	
			HWREGH(ePWMx + EPWM_CMPCTL) = (HWREGH(ePWMx + EPWM_CMPCTL) & (~(0x1<<6)))
			| ((enableShadowWrite & 0x1) << 6);

			HWREGH(ePWMx + EPWM_CMPCTL) = (HWREGH(ePWMx + EPWM_CMPCTL) & (~(0x3<<2))) 
			| ((ShadowToActiveLoadTrigger & 0x3) << 2);

			HWREGH(ePWMx + EPWM_CMPB) = CMPBVal & 0xFFFF;
			status = TRUE;
		}
	}
	
	if(CMPx == 'B'){
		if((OverwriteShadowFull) || ((HWREGH(ePWMx + EPWM_CMPCTL) & (1<<9)) == (0<<9)))
		{	
			HWREGH(ePWMx + EPWM_CMPCTL) = (HWREGH(ePWMx + EPWM_CMPCTL) & (~(0x1<<6)))
			| ((enableShadowWrite & 0x1) << 6);

			HWREGH(ePWMx + EPWM_CMPCTL) = (HWREGH(ePWMx + EPWM_CMPCTL) & (~(0x3<<2))) 
			| ((ShadowToActiveLoadTrigger & 0x3) << 2);

			HWREGH(ePWMx + EPWM_CMPB) = CMPBVal & 0xFFFF;
			status = TRUE;
		}
	}

	return status;
}

void PWMconfigAQAction(_uint32 ePWMx, _uint8 AQCTLx, _uint8 CBD,
													 _uint8 CBU,
													 _uint8 CAD,
													 _uint8 CAU,
													 _uint8 PRD,
													 _uint8 ZRO,
													 _uint8 SWForced) {
	
	_uint32 setting = 
				   |  ((CBD & 0x3) << 10)
				   |  ((CBU & 0x3) << 8)
				   |  ((CAD & 0x3) << 6)
				   |  ((CAU & 0x3) << 4)
				   |  ((PRD & 0x3) << 2)
				   |  ((ZRO & 0x3) << 0);
	
	if(AQCTLx == 'A'){
		HWREGH(ePWMx + EPWM_AQCTLA) = setting;
		setting = (HWREGH(ePWMx + EPWM_AQSFRC) & (~(0x3<<0))) | ((SWForced & 0x3) << 0);
	}

	if(AQCTLx == 'B'){
		HWREGH(ePWMx + EPWM_AQCTLB) = setting;
		setting = (HWREGH(ePWMx + EPWM_AQSFRC) & (~(0x3<<3))) | ((SWForced & 0x3) << 3);
	}

	HWREGH(ePWMx + EPWM_AQSFRC) = setting;
}

//
// heavily hacked config code from rodrigo in brazil
//
// pwmx: bit1-pwm0, bit2-pwm1, bit3-pwm2 (0-disable or 1-enable PWM module X)
// pwmPeriod: period of pwm output in msec
//

#define actionDisable				0
#define clearLow					1
#define setHight					2
#define toggleLHHL					3


#define EHRPWM_AQCTLA_ZRO_EPWMXAHIGH				(0)// high when CTR = 0
#define EHRPWM_AQCTLA_PRD_DONOTHING					(0)
#define EHRPWM_AQCTLA_CAU_EPWMXALOW					(0)// low when CTR = CMPA
#define EHRPWM_AQCTLA_CAD_DONOTHING					(0)
#define EHRPWM_AQCTLA_CBU_DONOTHING					(0)
#define EHRPWM_AQCTLA_CBD_DONOTHING					(0)
#define EHRPWM_AQSFRC_ACTSFA_DONOTHING				(0)

#define EHRPWM_AQCTLB_ZRO_EPWMXBHIGH				(0)// high when CTR = 0
#define EHRPWM_AQCTLB_PRD_DONOTHING					(0)
#define EHRPWM_AQCTLB_CAU_DONOTHING					(0)
#define EHRPWM_AQCTLB_CAD_DONOTHING					(0)
#define EHRPWM_AQCTLB_CBU_EPWMXBLOW					(0)
#define EHRPWM_AQCTLB_CBD_DONOTHING					(0)
#define EHRPWM_AQSFRC_ACTSFB_DONOTHING				(0)

#define CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK					(0x00000008u)
#define CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK						(0x00000010u)
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK			(0x00000010u)
#define CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_L4LS_GCLK					(0x00000100u)
#define CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L4_GCLK   (0x00000020u)

#define ePWMCLK_EN									(1<<8)

#define clkctrl_Idlest_field								(0x3u<<16)
#define clkctrl_ModuleMode_field							(0x3u)
#define clkstctrl_CLKTRCTRL_field							(0x3u)

#define clkctrl_IdlestFunc									(0x0u)
#define clkctrl_IdlestTrans									(0x1u)
#define clkctrl_IdlestIadle									(0x3u)
#define clkctrl_IdlestDisable								(0x3u)
#define clkctrl_ModuleModeEnable							(0x2u)

#define clkstctrl_CLKTRCTRL_SW_WKUP							(0x2u)

void Init_BusInterface(){
	// config L3_PER and L4_PER clocks
	HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) |= clkstctrl_CLKTRCTRL_SW_WKUP;
	while((HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) & 	clkstctrl_CLKTRCTRL_field) != clkstctrl_CLKTRCTRL_SW_WKUP);

	HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) |= clkstctrl_CLKTRCTRL_SW_WKUP;
	while((HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) & clkstctrl_CLKTRCTRL_field) != clkstctrl_CLKTRCTRL_SW_WKUP);

	HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) |= clkctrl_ModuleModeEnable;
	while((HWREG(SOC_CM_PER_REGS + CM_PER_L3_INSTR_CLKCTRL) & clkctrl_IdlestDisable) != clkctrl_ModuleModeEnable);

	HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) |= clkctrl_ModuleModeEnable;
	while((HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKCTRL) & clkctrl_IdlestDisable) != clkctrl_ModuleModeEnable);
	
	HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) |= clkstctrl_CLKTRCTRL_SW_WKUP;
	while((HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) & clkstctrl_CLKTRCTRL_field) != clkstctrl_CLKTRCTRL_SW_WKUP);

	HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) |= clkstctrl_CLKTRCTRL_SW_WKUP;
	while((HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) & clkstctrl_CLKTRCTRL_field) != clkstctrl_CLKTRCTRL_SW_WKUP);

	HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKCTRL) |= clkctrl_ModuleModeEnable;
	while((HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKCTRL) & clkctrl_IdlestDisable) != clkctrl_ModuleModeEnable);	

	// wait on L3 & L4 clock activity
	while(!(HWREG(SOC_CM_PER_REGS + CM_PER_L3S_CLKSTCTRL) & CM_PER_L3S_CLKSTCTRL_CLKACTIVITY_L3S_GCLK));
	while(!(HWREG(SOC_CM_PER_REGS + CM_PER_L3_CLKSTCTRL) & CM_PER_L3_CLKSTCTRL_CLKACTIVITY_L3_GCLK));
	while(!(HWREG(SOC_CM_PER_REGS + CM_PER_OCPWP_L3_CLKSTCTRL) & (CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L3_GCLK | CM_PER_OCPWP_L3_CLKSTCTRL_CLKACTIVITY_OCPWP_L4_GCLK)));
	while(!(HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) & (CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_L4LS_GCLK )));
}


#define PWM_Subsystem0 0
#define PWM_Subsystem1 1
#define PWM_Subsystem2 2

// PWMSS_x:1:1:1 = 0x7	active all 
// PWMSS_0:0:0:1 = 0x1  CM_PER_EPWMSS0_CLKCTRL
// PWMSS_1:0:1:0 = 0x2 	CM_PER_EPWMSS1_CLKCTRL
// PWMSS_2:1:0:0 = 0x4 	CM_PER_EPWMSS2_CLKCTRL
void Init_PWMSS(_uint8 pwmx){
	putString(0,"Inicializando clock_PWMSS: wait\n\r",34);
	_uint8 EPWMSSx_CLKCTRL;
	if(pwmx & 0x1) { // config EPWMSS0 clock
		EPWMSSx_CLKCTRL = CM_PER_EPWMSS0_CLKCTRL;
	}

	if(pwmx & 0x2) { // config EPWMSS1 clock
		EPWMSSx_CLKCTRL = CM_PER_EPWMSS1_CLKCTRL;
	}
	
	if(pwmx & 0x4) { // config EPWMSS2 clock
		EPWMSSx_CLKCTRL = CM_PER_EPWMSS2_CLKCTRL;
	}

	HWREG(SOC_CM_PER_REGS + EPWMSSx_CLKCTRL) |= clkctrl_ModuleModeEnable;
	
	while((HWREG(SOC_CM_PER_REGS + EPWMSSx_CLKCTRL) & clkctrl_ModuleMode_field) != clkctrl_ModuleModeEnable);

	while((HWREG(SOC_CM_PER_REGS + EPWMSSx_CLKCTRL) & clkctrl_Idlest_field) != (clkctrl_IdlestFunc << 16));
	putString(0,"Inicializando clock_PWMSS: enable\n\r",36);
}

_uint32 Init_eCAP(){
	return 0;
}

_uint32 Init_eQEP(){
	return 0;
}


// ePWM_x:1:1:1 = 0x7	active all 
// ePWM_0:0:0:1 = 0x1	pwmss0_tbclken
// ePWM_1:0:1:0 = 0x2 	pwmss1_tbclken
// ePWM_2:1:0:0 = 0x4 	pwmss2_tbclken
_uint32 Init_ePWM(_uint8 pwmx){
	putString(0,"Inicializando clock_ePWM: wait\n\r",33);
	_uint32 PWM_SubsystemX;
	_uint8 pwmss_num;
	_uint32 ePWMx;
	if(pwmx & 0x1) { // PWM0
		PWM_SubsystemX = SOC_PWMSS0_REGS;
		pwmss_num = 1<<0;
		ePWMx = SOC_EPWM_0_REGS;
	}
	if(pwmx & 0x2) { // PWM1
		PWM_SubsystemX = SOC_PWMSS1_REGS;
		pwmss_num = 1<<1;
		ePWMx = SOC_EPWM_1_REGS;
	}
	if(pwmx & 0x4) { // PWM2
		PWM_SubsystemX = SOC_PWMSS2_REGS;
		pwmss_num = 1<<2;
		ePWMx = SOC_EPWM_2_REGS;
	}

	HWREG(PWM_SubsystemX + PWMSS_CLKCONFIG) |= ePWMCLK_EN;	// enable PWMSSx clocks
	HWREG(SOC_CONTROL_REGS + pwmss_ctrl) |= pwmss_num;		// enable Timer Base Module Clock in Control Module
	putString(0,"Inicializando clock_ePWM: enable\n\r",35);

	return ePWMx;
}

void PWMconfig(unsigned int pwmx, unsigned int pwmPeriod) {
	putString(0,"Inicializando config_PWM: wait\n\r",33);
	Init_BusInterface();
	Init_PWMSS(pwmx);

	_uint32 ePWMx = Init_ePWM(pwmx);

	PWMTBClkDiv(ePWMx, PWM_PRESCALE); // config time-base clock
	PWMPeriodSet(ePWMx, pwmPeriod); // config PWM period

	PWMconfigAQAction(ePWMx,'A', // config Action Qualifiers for PWM0A
		EHRPWM_AQCTLA_CBD_DONOTHING,
		EHRPWM_AQCTLA_CBU_DONOTHING,
		EHRPWM_AQCTLA_CAD_DONOTHING,
		EHRPWM_AQCTLA_CAU_EPWMXALOW, // low when CTR = CMPA
		EHRPWM_AQCTLA_PRD_DONOTHING,
		EHRPWM_AQCTLA_ZRO_EPWMXAHIGH, // high when CTR = 0
		EHRPWM_AQSFRC_ACTSFA_DONOTHING);

	PWMconfigAQAction(ePWMx, 'B', // PWM0B
		EHRPWM_AQCTLB_CBD_DONOTHING,
		EHRPWM_AQCTLB_CBU_EPWMXBLOW, // low when CTR = CMPB
		EHRPWM_AQCTLB_CAD_DONOTHING,
		EHRPWM_AQCTLB_CAU_DONOTHING,
		EHRPWM_AQCTLB_PRD_DONOTHING,
		EHRPWM_AQCTLB_ZRO_EPWMXBHIGH, // high when CTR = 0
		EHRPWM_AQSFRC_ACTSFB_DONOTHING);
	putString(0,"Inicializando config_PWM: enable\n\r",35);

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

	GPIOModuleEnab(SOC_GPIO_1_REGS); // enable GPIO module
	GPIODirectionSet(SOC_GPIO_1_REGS, LED_USR2, GPIO_DIR_OUTPUT); // set GPIO pin as output
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

		GPIOPinWrite(SOC_GPIO_1_REGS, LED_USR2, 1);
		Delay(0x2FFFF);
		GPIOPinWrite(SOC_GPIO_1_REGS, LED_USR2, 0);
		Delay(0x3FFFF);
		GPIOPinWrite(SOC_GPIO_1_REGS, LED_USR2, 1);
		Delay(0x1FFFF);
		GPIOPinWrite(SOC_GPIO_1_REGS, LED_USR2, 0);
		Delay(0xAFFFF);
	}
} 

// and the header file bbbpwm.h :

//
// bbbpwm.h - pwm library header file
//
#define SOC_ECAP_REGS (0x00000100) // ecap0 not
#define SOC_ECAP_0_REGS (SOC_PWMSS0_REGS + SOC_ECAP_REGS) // implemented

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

#include "hw_cm_per.h" // do not remove, too many defines needed here

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
#define LED_USR2 23 // LED USR2
#define SOC_EHRPWM_1_MODULE_FREQ 100000000 // SYSCLKOUT [10 ns/tick]
#define PWM_PRESCALE 224 // pwm clk divider - TBCLK
#define TICKS_PER_MS 446 // ticks per msec pwm output period
#define PWM_PERIOD_MS 20 // desired pwm output period in ms

// 0x7 * 0x4
// HSPCLKDIV*CLKDIV
// 14*16


// PWM_PRESCALE of 224 = 14 * X (X = 1, 2, 4...128), see TBCTL Register
// TICKS_PER_MS * PWM_PERIOD_MS = period of pwm output in ticks = TBPRD (only 16 bits)
// FINITO


