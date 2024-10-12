#ifndef _TIMERS_REGISTERS_
#define _TIMERS_REGISTERS_

#pragma region TIMERS REGISTERS

#define TIMER_1MS_COUNT                     (0x5DC0u)
#define TIMER_OVERFLOW                      (0xFFFFFFFFu)

#define TIDR                                (0x00u)  // Register
#define TIOCP_CFG                           (0x10u)  // OCP Configuration Register
#define IRQ_EOI                             (0x20u)  // IRQ End-Of-Interrupt Register
#define IRQSTATUS_RAW                       (0x24u)  // IRQSTATUS Raw Register
#define IRQSTATUS                           (0x28u)  // IRQSTATUS Register
#define IRQSTATUS_SET                       (0x2Cu)  // IRQENABLE Set Register
#define IRQSTATUS_CLR                       (0x30u)  // IRQENABLE Clear Register
#define IRQWAKEEN                           (0x34u)  // IRQ Wakeup Enable Register
#define TCLR                                (0x38u)  // Control Register
#define TCRR                                (0x3Cu)  // Counter Register
#define TLDR                                (0x40u)  // Load Register
#define TTGR                                (0x44u)  // Trigger Register
#define TWPS                                (0x48u)  // Write Posted Status Register
#define TMAR                                (0x4Cu)  // Match Register
#define TCAR1                               (0x50u)  // Capture Register
#define TSICR                               (0x54u)  // Synchronous Interface Control Register
#define TCAR2                               (0x58u)  // Capture Register

#pragma endregion TIMERS REGISTERS

/* The Input clock is selected as 24MHz. So for 1ms set the count to 0x5DC0. 
 *If the input clock is changed to different source this value has to be updated 
 *accordingly.	
*/
#define TIMER7_CLKCTRL					        0x7C
#define TIMER6_CLKCTRL                          0xF0
#define TIMER5_CLKCTRL                          0xEC
#define TIMER4_CLKCTRL                          0x88
#define TIMER3_CLKCTRL                          0x84
#define TIMER2_CLKCTRL                          0x80     

/* RTC */
#define RTC_BASE           						0x44E3E000
#define SECONDS_REG        						0x44E3E000
#define MINUTES_REG        						0x44E3E004
#define HOURS_REG          						0x44E3E008
#define RTC_CTRL_REG       						0x44E3E040
#define RTC_STATUS_REG     						0x44E3E044
#define RTC_INTERRUPTS_REG 						0x44E3E048
#define RTC_OSC_REG        						0x44E3E054
#define KICK0R             						0x44E3E06C
#define KICK1R             						0x44E3E070

typedef enum Timer{
    TIMER2 = 2,
    TIMER3,
    TIMER4,
    TIMER5,
    TIMER6,
    TIMER7
}Timer;

typedef enum {
     PEND_TCLR,
     PEND_TCRR,
     PEND_TLDR,
     PEND_TTGR,
     PEND_TMAR
}pendent_register;

void set_flag_timer(unsigned);
unsigned char get_flag_timer(void);
void timerSetup(void);
void timerStart(unsigned int);
void timerStartCount(void);
void timerStop(void);
void delay_ms(unsigned int);
void delayInterrupt_ms(unsigned int);
void pseudo_delay(unsigned int);

#endif