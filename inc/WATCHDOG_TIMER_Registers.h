#ifndef _WATCHDOG_TIMER_REGISTERS_
#define _WATCHDOG_TIMER_REGISTERS_

#pragma region WATCHDOG TIMER REGISTERS

#define WDT1							(0x44E35000u)
#define WDT_WIDR                        (0x0u) 
#define WDT_WDSC                        (0x10u) 
#define WDT_WDST                        (0x14u) 
#define WDT_WISR                        (0x18u) 
#define WDT_WIER                        (0x1Cu) 
#define WDT_WCLR                        (0x24u) 
#define WDT_WCRR                        (0x28u) 
#define WDT_WLDR                        (0x2Cu) 
#define WDT_WTGR                        (0x30u)
	#ifndef WDT_BASE
#define WDT_WWPS                        (0x34)
	#endif 
#define WDT_WDLY                        (0x44)
	#ifndef WDT_BASE
#define WDT_WSPR                        (0x48)
	#endif
#define WDT_WIRQSTATRAW                 (0x54u) 
#define WDT_WIRQSTAT                    (0x58u) 
#define WDT_WIRQENSET                   (0x5Cu) 
#define WDT_WIRQENCLR                   (0x60u) 

#pragma endregion WATCHDOG TIMER REGISTERS

#define XXXX_AAAA 						(0x0000AAAA)
#define XXXX_5555						(0x00005555)			
#define XXXX_BBBB						(0x0000BBBB)
#define XXXX_4444						(0x00004444)

#define WDT_WWPS_W_PEND_WCLR			(1<<0)
#define WDT_WWPS_W_PEND_WCRR			(1<<1)
#define WDT_WWPS_W_PEND_WLDR			(1<<2)
#define WDT_WWPS_W_PEND_WTGR			(1<<3)
#define WDT_WWPS_W_PEND_WSPR			(1<<4)
#define WDT_WWPS_W_PEND_WDLY			(1<<5)


void disableWDT();
void enableWDT();

void inicializationWDT();
unsigned int get_timeWDT();
void SoftResetWDT();

#endif