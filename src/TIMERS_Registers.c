#include "hw_types.h"
#include "soc_AM335x.h"
#include "CLOCK_MODULE_Registers.h"
#include "TIMERS_Registers.h"

// #define verifyPendentWrite(reg) \
//      if(HWREG(DMTIMER_TSICR) & (1U << 2)) \
//           while(HWREG(SOC_DMTIMER_7_REGS+TWPS) & (1U << reg))

static unsigned char activeTimer7;

void set_flag_timer(unsigned val){
     activeTimer7 = val;
}

unsigned char get_flag_timer(){
     return activeTimer7;
}

// Wait for pending write to complete
void verifyPendentWrite(pendent_register reg) {
     // If posted mode is active wait for pending write to reg to complete
     while(HWREG(SOC_DMTIMER_7_REGS+TWPS) & (1U << reg));
}

void timerSetup(void) {
    HWREG(SOC_CM_PER_REGS+CM_PER_TIMER7_CLKCTRL) |= (0b10U << 0);

    while((HWREG(SOC_CM_PER_REGS+CM_PER_TIMER7_CLKCTRL) & 0b11U) != (0b10U << 0));

    HWREG(SOC_DMTIMER_7_REGS+TCLR) |= (1U << 1);
}

void timerStart(unsigned int ms) {
     verifyPendentWrite(PEND_TCRR);
     HWREG(SOC_DMTIMER_7_REGS+TCRR) = ms;

     verifyPendentWrite(PEND_TLDR);
     HWREG(SOC_DMTIMER_7_REGS+TLDR) = ms;

     timerStartCount();
}

void timerStartCount(void) {
     verifyPendentWrite(PEND_TCLR);

     HWREG(SOC_DMTIMER_7_REGS+TCLR) |= (1U << 0);
}

void timerStop(void) {
     verifyPendentWrite(PEND_TCLR);

     HWREG(SOC_DMTIMER_7_REGS+TCLR) &= ~(1U << 0);
}


void delay_ms(unsigned int ms){
     while(ms){
          timerStart(0);
          while(HWREG(SOC_DMTIMER_7_REGS+TCRR) < TIMER_1MS_COUNT);
          timerStop();
          ms--;
     }
}

void delayInterrupt_ms(unsigned int ms){
     unsigned int countVal = TIMER_OVERFLOW - (ms * TIMER_1MS_COUNT);

     // Wait for previous write to complete
     verifyPendentWrite(TCRR);

     // Load the register with the re-load value
     HWREG(SOC_DMTIMER_7_REGS+TCRR) = countVal;

     activeTimer7 = 0;

     // Enable the DMTimer OVF interrupts
     HWREG(SOC_DMTIMER_7_REGS+IRQSTATUS_SET) = 0x2;

     // Enable the DMTimer MAT interrupts
     // HWREG(SOC_DMTIMER_7_REGS+IRQSTATUS_SET) |= (1<<0);    

     // Start the DMTimer
     timerStartCount();

     while(activeTimer7 == 0){
          
     }

     // Disable the DMTimer OVF interrupts
     HWREG(SOC_DMTIMER_7_REGS+IRQSTATUS_CLR) = 0x2;

     // Disable the DMTimer MAT interrupts
     // HWREG(SOC_DMTIMER_7_REGS+IRQSTATUS_CLR) |= (1<<0);
}

void pseudo_delay(unsigned int TIME){
	volatile unsigned int ra;
	for(ra = 0; ra < TIME; ra ++);
}/*	end of function	delay */