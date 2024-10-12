#ifndef _INTERRUPT_SERVICE_ROUTINE_
#define _INTERRUPT_SERVICE_ROUTINE_

#pragma region INTERRUPT CONTROLLER REGISTERS
#include "INTERRUPT_CONTROLLER_Registers.h"
#pragma endregion INTERRUPT CONTROLLER REGISTERS

/* The Input clock is selected as 24MHz. So for 1ms set the count to 0x5DC0. 
 *If the input clock is changed to different source this value has to be updated 
 *accordingly.	
 */

#define IRL_priorityMax					(0x3Fu)
#define IRL_priorityMin					(0x0)
#define GPIOINT0A 						(96) 
#define	GPIOINT0B 						(97)
#define GPIOINT1A 						(98) 
#define	GPIOINT1B 						(99)
#define GPIOINT2A 						(32) 
#define	GPIOINT2B 						(33)
#define GPIOINT3A 						(62)
#define	GPIOINT3B 						(63)

#define TINT7							(95)
#define WDT1INT							(91)

typedef enum{
	FIQ,
	IRQ
}IRQnFIQ;

int interruptServiceRoutine_Setup(unsigned int inter);
void interrupts(unsigned char, unsigned char, unsigned char);
void interruptConfigGPIO(unsigned int, unsigned char, unsigned int);

#endif