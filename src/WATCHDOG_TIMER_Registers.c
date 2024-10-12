#include "hw_types.h"
#include "soc_AM335x.h"
#include "WATCHDOG_TIMER_Registers.h"
#include "INTERRUPT_CONTROLLER_Registers.h"
#include "uart.h"

#define WAIT_WRITE_WSPR(W_PEND_WX_0_5) \
	while ((WDT1+WDT_WWPS) & (W_PEND_WX_0_5))

void disableWDT(){
	HWREG(WDT1+WDT_WSPR) = XXXX_AAAA;

	while (HWREG(WDT1+WDT_WWPS) & (1<<4));

	HWREG(WDT1+WDT_WSPR) = XXXX_5555;

	while (HWREG(WDT1+WDT_WWPS) & (1<<4));
}/* end of function disableWDT */

void enableWDT(){
	HWREG(WDT1+WDT_WSPR) = XXXX_BBBB;

	while (HWREG(WDT1+WDT_WWPS) & (1<<4));

	HWREG(WDT1+WDT_WSPR) = XXXX_4444;
	
    while (HWREG(WDT1+WDT_WWPS) & (1<<4));
}/* end of function	enableWDT */



void inicializationWDT(){

	putString(0,"Inicializando config_WatchDog: wait\n\r",38);

	// HWREG(WDT1+WDT_WDSC) &= ~(0b11<<3);
	// HWREG(WDT1+WDT_WDSC) |= (0b0<<3);

	// HWREG(CM_WKUP+CM_WKUP_WDT1_CLKCTRL) |= 2<<0;
	// HWREG(CM_DPLL+CLKSEL_WDT1_CLK) |= 1<<0;

	putString(0,"Habilitando interrupcoes\n\r",27);

	// HWREG(SOC_AINTC_REGS+INTC_ILR_(91)) &= ~(1<<0) & ~(0x3F);

	// HWREG(SOC_AINTC_REGS+INTC_ILR91) &= ~(1<<0) & ~(0x3F<<2);

	// HWREG(SOC_AINTC_REGS+INTC_MIR_CLEAR2) |= 1<<(91 & 0x1F);

	// HWREG(SOC_WDT_1_REGS+WDT_WIRQSTAT) = (1<<1);	//clear dly
	// HWREG(SOC_WDT_1_REGS+WDT_WIRQSTAT) = (1<<0);	//clear ovf

	// delay event
	HWREG(WDT1+WDT_WIRQENSET) |= (1<<1);		//enable 
	// HWREG(WDT1+WDT_WIRQENCLR) |= (1<<1);		//disable

	// overflow event
	// HWREG(WDT1+WDT_WIRQENSET) |=  (1<<0);	//enable
	// HWREG(WDT1+WDT_WIRQENCLR) |=  (1<<0);	//disable


	disableWDT();
	HWREG(WDT1+WDT_WCLR) &= ~(7<<2);
	while(HWREG(SOC_WDT_1_REGS+WDT_WWPS) & (1<WDT_WWPS_W_PEND_WCLR));

	HWREG(WDT1+WDT_WCLR) |= 1<<2;
	while(HWREG(SOC_WDT_1_REGS+WDT_WWPS) & (1<WDT_WWPS_W_PEND_WCLR));

	HWREG(WDT1+WDT_WCLR) |= 1<<5;
	while(HWREG(SOC_WDT_1_REGS+WDT_WWPS) & (1<WDT_WWPS_W_PEND_WCLR));

	HWREG(WDT1+WDT_WTGR) &= 0b10;
	while(HWREG(SOC_WDT_1_REGS+WDT_WWPS) & (1<WDT_WWPS_W_PEND_WTGR));

	unsigned timeCount=HWREG(SOC_WDT_1_REGS+WDT_WLDR);

	// HWREG(WDT1+WDT_WDLY) = timeCount+ (0xFFFFFFFF-timeCount) - 0x10;
	HWREG(WDT1+WDT_WDLY) = timeCount + 0x10000;
	while(HWREG(SOC_WDT_1_REGS+WDT_WWPS) & (1<WDT_WWPS_W_PEND_WDLY));

	HWREG(WDT1+WDT_WCRR)=timeCount;
	while(HWREG(SOC_WDT_1_REGS+WDT_WWPS) & (1<WDT_WWPS_W_PEND_WCRR));

	putString(0,"Inicializando config_WatchDog: enable\n\r",40);
	enableWDT();
}

unsigned int get_timeWDT(){
	return (HWREG(WDT1+WDT_WCRR));
}

void SoftResetWDT(){
	HWREG(HWREG(WDT1+WDT_WDSC)) |= (1<<1);
}