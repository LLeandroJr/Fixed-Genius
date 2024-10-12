#include "hw_types.h"
#include "soc_AM335x.h"
#include "CONTROL_MODULE_Registers.h"
#include "CLOCK_MODULE_Registers.h"
#include "GPIO_Registers.h"
#include "INTERRUPT_SERVICE_ROUTINE.h"
#include "TIMERS_Registers.h"
#include "uart.h"
#include "WATCHDOG_TIMER_Registers.h"

#include "flags.h"
#include "types.h"

extern unsigned int TIME_DELAY;
extern _uintptr8 prt_array_levels_;
extern _uint32 _levels_;
extern _uint32 _current_levels_;
extern _uint32 _press_buffer_;
extern _uint8 _reset_status_;

int interruptServiceRoutine_Setup(unsigned int inter){

}

void interrupts(unsigned char interruptIntNumber, unsigned char Priority, unsigned char FIQnIRQ){
	// HWREG(SOC_AINTC_REGS+INTC_SYSCONFIG);
	// HWREG(SOC_AINTC_REGS+INTC_IDLE);

	unsigned int MIR_CLEARn = (interruptIntNumber>>5);
	
	if(MIR_CLEARn == 0){
		MIR_CLEARn = INTC_MIR_CLEAR0;
	}

	if(MIR_CLEARn == 1){
		MIR_CLEARn = INTC_MIR_CLEAR1;
	}

	if(MIR_CLEARn == 2){
		MIR_CLEARn = INTC_MIR_CLEAR2;
	}

	if(MIR_CLEARn == 3){
		MIR_CLEARn = INTC_MIR_CLEAR3;
	}

	// bit 0 IntIRQ : Interrupt is routed to IRQ.
	// bit 7-2 Interrupt priority 0x0
	// (by default, interrupts are mapped to IRQ and priority is 0x0 [highest]).
	// HWREG(SOC_AINTC_REGS+INTC_ILR_(interruptIntNumber)) &= ~(FIQnIRQ) & ~((Priority)<<2);

	interruptIntNumber = interruptIntNumber & 0x1F;
	HWREG(SOC_AINTC_REGS+MIR_CLEARn) |= 1<<(interruptIntNumber);
}

void interruptConfigGPIO(unsigned int GPIOregisters, unsigned char POINTRPEND,unsigned gpio_pin){
	if(POINTRPEND == 1)
	{
		HWREG((GPIOregisters)+GPIO_IRQSTATUS_SET_0) |= (1 << gpio_pin);
	}

	if(POINTRPEND == 2)
	{
		HWREG((GPIOregisters)+GPIO_IRQSTATUS_SET_1) |= (1 << gpio_pin);
	}

	HWREG((GPIOregisters)+GPIO_RISINGDETECT) |= (1 << gpio_pin);
	// HWREG((GPIOregisters)+GPIO_DEBOUNCENABLE) |= (1 << gpio_pin);
}

#pragma region Handler
void IRQ_Number0   (void){};
void IRQ_Number1   (void){};
void IRQ_Number2   (void){};
void IRQ_Number3   (void){};
void IRQ_Number4   (void){};
void IRQ_Number5   (void){};
void IRQ_Number6   (void){};
void IRQ_Number7   (void){};
void IRQ_Number8   (void){};
void IRQ_Number9   (void){};
void IRQ_Number10  (void){};
void IRQ_Number11  (void){};
void IRQ_Number12  (void){};
void IRQ_Number13  (void){};
void IRQ_Number14  (void){};
void IRQ_Number15  (void){};
void IRQ_Number16  (void){};
void IRQ_Number17  (void){};
void IRQ_Number18  (void){};
void IRQ_Number19  (void){};
void IRQ_Number20  (void){};
void IRQ_Number21  (void){};
void IRQ_Number22  (void){};
void IRQ_Number23  (void){};
void IRQ_Number24  (void){};
void IRQ_Number25  (void){};
void IRQ_Number26  (void){};
void IRQ_Number27  (void){};
void IRQ_Number28  (void){};
void IRQ_Number29  (void){};
void IRQ_Number30  (void){};
void IRQ_Number31  (void){};

void IRQ_Number32   (void){};
void IRQ_Number33   (void){};
void IRQ_Number34   (void){};
void IRQ_Number35   (void){};
void IRQ_Number36   (void){};
void IRQ_Number37   (void){};
void IRQ_Number38   (void){};
void IRQ_Number39   (void){};
void IRQ_Number40   (void){};
void IRQ_Number41   (void){};
void IRQ_Number42   (void){};
void IRQ_Number43   (void){};
void IRQ_Number44   (void){};
void IRQ_Number45   (void){};
void IRQ_Number46   (void){};
void IRQ_Number47   (void){};
void IRQ_Number48   (void){};
void IRQ_Number49   (void){};
void IRQ_Number50   (void){};
void IRQ_Number51   (void){};
void IRQ_Number52   (void){};
void IRQ_Number53   (void){};
void IRQ_Number54   (void){};
void IRQ_Number55   (void){};
void IRQ_Number56   (void){};
void IRQ_Number57   (void){};
void IRQ_Number58   (void){};
void IRQ_Number59   (void){};
void IRQ_Number60   (void){};
void IRQ_Number61   (void){};
void IRQ_Number62   (void){};
void IRQ_Number63   (void){};

void IRQ_Number64   (void){};
void IRQ_Number65   (void){};
void IRQ_Number66   (void){};
void IRQ_Number67   (void){};
void IRQ_Number68   (void){};
void IRQ_Number69   (void){};
void IRQ_Number70   (void){};
void IRQ_Number71   (void){};
void IRQ_Number72   (void){};
void IRQ_Number73   (void){};
void IRQ_Number74   (void){};
void IRQ_Number75   (void){};
void IRQ_Number76   (void){};
void IRQ_Number77   (void){};
void IRQ_Number78   (void){};
void IRQ_Number79   (void){};
void IRQ_Number80   (void){};
void IRQ_Number81   (void){};
void IRQ_Number82   (void){};
void IRQ_Number83   (void){};
void IRQ_Number84   (void){};
void IRQ_Number85   (void){};
void IRQ_Number86   (void){};
void IRQ_Number87   (void){};
void IRQ_Number88   (void){};
void IRQ_Number89   (void){};
void IRQ_Number90   (void){};
void WDT1INT_IRQHandler91(void);
void IRQ_Number92   (void){};
void IRQ_Number93   (void){};
void IRQ_Number94   (void){};
//DMTIMER7
void TINT7_IRQHandler95(void);
void IRQ_Number96   (void){};
void IRQ_Number97   (void){};
//GPIO1A
void GPIOINT1A_IRQHandler98(void);
//GPIO1B
void GPIOINT1B_IRQHandler99(void){}
void IRQ_Number100  (void){};
void IRQ_Number101  (void){};
void IRQ_Number102  (void){};
void IRQ_Number103  (void){};
void IRQ_Number104  (void){};
void IRQ_Number105  (void){};
void IRQ_Number106  (void){};
void IRQ_Number107  (void){};
void IRQ_Number108  (void){};
void IRQ_Number109  (void){};
void IRQ_Number110  (void){};
void IRQ_Number111  (void){};
void IRQ_Number112  (void){};
void IRQ_Number113  (void){};
void IRQ_Number114  (void){};
void IRQ_Number115  (void){};
void IRQ_Number116  (void){};
void IRQ_Number117  (void){};
void IRQ_Number118  (void){};
void IRQ_Number119  (void){};
void IRQ_Number120  (void){};
void IRQ_Number121  (void){};
void IRQ_Number122  (void){};
void IRQ_Number123  (void){};
void IRQ_Number124  (void){};
void IRQ_Number125  (void){};
void IRQ_Number126  (void){};
void IRQ_Number127  (void){};
#pragma endregion Handler

static void Reserverd(void);

static void (*IRQ_table[128])(void)={
	IRQ_Number0,
	IRQ_Number1,
	IRQ_Number2,
	IRQ_Number3,
	IRQ_Number4,
	Reserverd,
	Reserverd,
	IRQ_Number7,
	Reserverd,
	IRQ_Number9,
	IRQ_Number10,
	IRQ_Number11,
	IRQ_Number12,
	IRQ_Number13,
	IRQ_Number14,
	Reserverd,
	IRQ_Number16,
	IRQ_Number17,
	IRQ_Number18,
	IRQ_Number19,
	IRQ_Number20,
	IRQ_Number21,
	IRQ_Number22,
	IRQ_Number23,
	IRQ_Number24,
	IRQ_Number25,
	IRQ_Number26,
	IRQ_Number27,
	IRQ_Number28,
	IRQ_Number29,
	IRQ_Number30,
	IRQ_Number31,

	IRQ_Number32,
	IRQ_Number33,
	IRQ_Number34,
	Reserverd,
	IRQ_Number36,
	IRQ_Number37,
	Reserverd,
	IRQ_Number39,
	IRQ_Number40,
	IRQ_Number41,
	IRQ_Number42,
	IRQ_Number43,
	IRQ_Number44,
	IRQ_Number45,
	IRQ_Number46,
	IRQ_Number47,
	Reserverd,
	Reserverd,
	Reserverd,
	Reserverd,
	IRQ_Number52,
	IRQ_Number53,
	IRQ_Number54,
	IRQ_Number55,
	IRQ_Number56,
	IRQ_Number57,
	IRQ_Number58,
	IRQ_Number59,
	IRQ_Number60,
	IRQ_Number61,
	IRQ_Number62,
	IRQ_Number63,

	IRQ_Number64,
	IRQ_Number65,
	IRQ_Number66,
	IRQ_Number67,
	IRQ_Number68,
	IRQ_Number69,
	IRQ_Number70,
	IRQ_Number71,
	IRQ_Number72,
	IRQ_Number73,
	IRQ_Number74,
	IRQ_Number75,
	IRQ_Number76,
	IRQ_Number77,
	IRQ_Number78,
	IRQ_Number79,
	IRQ_Number80,
	IRQ_Number81,
	IRQ_Number82,
	IRQ_Number83,
	Reserverd,
	Reserverd,
	IRQ_Number86,
	IRQ_Number87,
	IRQ_Number88,
	IRQ_Number89,
	IRQ_Number90,
	WDT1INT_IRQHandler91,
	IRQ_Number92,
	IRQ_Number93,
	IRQ_Number94,
	TINT7_IRQHandler95,

	IRQ_Number96,
	IRQ_Number97,
	GPIOINT1A_IRQHandler98,
	GPIOINT1B_IRQHandler99,
	IRQ_Number100,
	IRQ_Number101,
	Reserverd,
	Reserverd,
	Reserverd,
	Reserverd,
	Reserverd,
	Reserverd,
	Reserverd,
	Reserverd,
	Reserverd,
	Reserverd,
	IRQ_Number112,
	IRQ_Number113,
	IRQ_Number114,
	IRQ_Number115,
	Reserverd,	 // IRQ_Number116
	Reserverd,	 // IRQ_Number117
	Reserverd,	 // IRQ_Number118
	Reserverd,	 // IRQ_Number119
	IRQ_Number120,
	IRQ_Number121,
	IRQ_Number122,
	IRQ_Number123,
	IRQ_Number124,
	IRQ_Number125,
	Reserverd,	// IRQ_Number126
	Reserverd	// IRQ_Number127
};

void inToString2(unsigned number){
	char valor[15]={'0','0','0','0','0','0','0','0','0','0','0','0','0','0','0'};
	int i=15;

	while(number>0){
		valor[--i]=(number%10)+0x30;

		number/=10;
	}

	putString(0,valor,15);
	putString(0,"\n\r",3);
}

void ISR_Handler(void){
	unsigned active_IRQ_Number = HWREG(SOC_AINTC_REGS+INTC_SIR_IRQ) & 0x7F;

	typedef void (*IntVector)(void);
	IntVector vect = IRQ_table[active_IRQ_Number];
	(*vect)();

	HWREG(SOC_AINTC_REGS+INTC_CONTROL) = (1<<0);	// NewIRQAgr
}

void WDT1INT_IRQHandler91(){
	putString(0,"intWATCHDOG\n\r", 14);

	HWREG(SOC_WDT_1_REGS+WDT_WTGR) ^= 0b10;

	while(HWREG(SOC_WDT_1_REGS+WDT_WWPS) & (1<WDT_WWPS_W_PEND_WTGR));

	HWREG(SOC_WDT_1_REGS+WDT_WIRQSTAT) = (1<<1);	//clear dly
	// HWREG(SOC_WDT_1_REGS+WDT_WIRQSTAT) = (1<<0);	//clear ovf

}

void TINT7_IRQHandler95(){

	if(HWREG(SOC_DMTIMER_7_REGS+IRQSTATUS_RAW) & (1<<1)){
		// putString(0,"---------------DELAY---------------\n\r", 38);
		set_flag_timer(1);
		timerStop();	// Stop the DMTimer7

		HWREG(SOC_DMTIMER_7_REGS+IRQSTATUS) = 0x2;
	}

	if(HWREG(SOC_DMTIMER_7_REGS+IRQSTATUS_RAW) & (1<<0)){
		// putString(0,"---------------DELAY---------------\n\r", 38);
		set_flag_timer(1);
		timerStop();	// Stop the DMTimer7

		HWREG(SOC_DMTIMER_7_REGS+IRQSTATUS) = (1<<0);
	}
}

void GPIOINT1A_IRQHandler98(){

	if((HWREG(SOC_GPIO_1_REGS+GPIO_IRQSTATUS_RAW_0) & (1<<16)))
	{	
		putString(0,"botao1\n\r", 9);
		_press_buffer_ = (1);
		
		HWREG(SOC_GPIO_1_REGS+GPIO_IRQSTATUS_0) |= (1<<16);
	}	

	if(HWREG(SOC_GPIO_1_REGS+GPIO_IRQSTATUS_RAW_0) & (1<<28))
	{
		putString(0,"botao2\n\r", 9);
		_press_buffer_ = (2);

		HWREG(SOC_GPIO_1_REGS+GPIO_IRQSTATUS_0) |= (1<<28);
	}

	if(HWREG(SOC_GPIO_1_REGS+GPIO_IRQSTATUS_RAW_0) & (1<<14))
	{
		putString(0,"botao3\n\r", 9);

		_press_buffer_ = (3);

		HWREG(SOC_GPIO_1_REGS+GPIO_IRQSTATUS_0) |= (1<<14);
	}

	if(HWREG(SOC_GPIO_1_REGS+GPIO_IRQSTATUS_RAW_0) & (1<<15))
	{
		putString(0,"botao4\n\r", 9);

		_press_buffer_ = (4);

		HWREG(SOC_GPIO_1_REGS+GPIO_IRQSTATUS_0) |= (1<<15);
	}

	if(_reset_status_ & WAIT_BUTTON){
		_reset_status_ &= ~(WAIT_BUTTON);
		return;
	}

	--_levels_;

	if(_levels_ < 0 || _levels_ > 20){
		// putString(0,"APERTE MAIS SEU BAITOLA\n\r", 26);
		return;
	}

	if(prt_array_levels_[_levels_] != _press_buffer_){
		_current_levels_ = -1;	// reset level
		_press_buffer_ = INVALID;
		_reset_status_ |= RESTART_GAME;
		return;
	}

}

static void Reserverd(){
	while(1);
}