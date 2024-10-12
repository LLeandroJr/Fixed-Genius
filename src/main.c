/*
 =====================================================================================
 *
 *	Filename:  main.c
 *
 *	Description:  
 *
 *	Version:  a.0
 *	Created:  dd/mm/aa hh:mm:ss
 *	Revision:  none
 *	Compiler:  arm-none-eabi-gcc
 *
 *	Author:  Diogo Bandeira, user.email@gmail.com
 *	Author:  Leandro Rodrigues da Silva Júnior, leandro.jr369@gmail.com
 *
 * =====================================================================================
 */

#include "hw_types.h"
#include "soc_AM335x.h"
#include "CLOCK_MODULE_Registers.h"
#include "CONTROL_MODULE_Registers.h"
#include "GPIO_Registers.h"
#include "INTERRUPT_SERVICE_ROUTINE.h"
#include "TIMERS_Registers.h"
#include "WATCHDOG_TIMER_Registers.h"
#include "uart.h"

#include "flags.h"
#include "random.h"
#include "types.h"

// #define DEBUG

#define initialization_for_array(array, size, ini_val)	\
	for( int i= 0; i< size; ++i)						\
		array[i] = ini_val;

#define USR0							(21)	// INTERNAL LED gpio1_21
#define USR1							(22)	// INTERNAL LED gpio1_22
#define USR2							(23)	// INTERNAL LED gpio1_23
#define USR3							(24)	// INTERNAL LED gpio1_24

#define TIMER5							(5)		// EXTERNAL PIN gpio2_3 BUZZER
#define TIMER7							(3)		// EXTERNAL PIN gpio2_3
#define TIMER6							(4)		// EXTERNAL PIN gpio2_4
#define GPIO2_1							(1)		// EXTERNAL PIN gpio2_1
#define GPIO2_2							(2)		// EXTERNAL PIN gpio2_2

#define BUTTON1 (16)
#define BUTTON2 (28)
#define BUTTON3 (14)
#define BUTTON4 (15)

#define _typesel_PULLUP					(1<<4)		// 1: Pullup selected
#define _typesel_PULLDOWN				(~(1<<4))	// 0: Pulldown selected
#define _puden_enable					(~(1<<3))	// 0: Pullup/pulldown enabled
#define _puden_disable					(1<<3)		// 1: Pullup/pulldown disabled

#define clearTerminal					0, "\033[H\033[J\r", 8

#define _active_ 						(1)
#define _inactive_						(0)

#define _max_levels_ 21

_uint32 TIME_DELAY;

_uint8 _array_levels_[_max_levels_];
_uintptr8 prt_array_levels_ = _array_levels_;
_uint32 _levels_;	
_uint32 _current_levels_;
_uint32 _press_buffer_;
_uint8 _reset_status_;

static void sysconfig(void);

void inToString(unsigned number);
void LEDS(_uint8 option);
void LEDS_OFF();
void Win_or_Not();

int _main(void){
	// Config do componentes
	initialization_for_array(_array_levels_, _max_levels_, INVALID);
	_levels_ = _max_levels_;
	_current_levels_ = _max_levels_ -2;
	_press_buffer_ = INVALID;
	_reset_status_ = RESTART_GAME;

	_uint8 _show_sequence = _active_;
	
	TIME_DELAY=4000;

	// Config do ambiente
	putString(clearTerminal);
	inicializationWDT();
  	sysconfig();
	timerSetup();	
	
  	while (1){
		HWREG(SOC_GPIO_1_REGS+GPIO_SETDATAOUT) = (1<<USR3);
		// INICIO DA IMPLEMENTACAO GENIUS
		LEDS_OFF();

		if(_reset_status_ & RESTART_GAME){
			putString(0,"while\n\r", 8);
			
			_reset_status_ |= WAIT_BUTTON;
			while(_reset_status_ & WAIT_BUTTON){};
			// Poderia desabilitar as interrupcoes

			initialization_for_array(_array_levels_, _max_levels_, INVALID);
			_current_levels_ = _max_levels_ - 1;
		}

		// HWREG(SOC_GPIO_2_REGS+GPIO_IRQSTATUS_CLR_0) |= (1<<TIMER7) | (1<<TIMER6) | (1<<GPIO2_1) | (1<<TIMER5);

		_levels_ = _max_levels_;			// quantidade fixa de fases
											// inicia sempre da primeira fase
		_press_buffer_ = INVALID;
		_reset_status_ &= ~(RESTART_GAME);

		random_seed = get_timeWDT();									// inToString(random_seed);		// DEBUG

		while((_levels_ > _current_levels_ )){
			random_value = xorshift32(&random_seed);					// inToString(random_value);	// DEBUG	
			--_levels_;													// inToString(_levels_);		// DEBUG

			if(_array_levels_[_levels_] == INVALID){							// inToString(random_value);	// DEBUG
				_array_levels_[_levels_] = (_uint8)random_value; // _array_levels_[_levels_] = _aleatoriador();
			}

			// exibe a sequencia
			LEDS(_array_levels_[_levels_]);		delayInterrupt_ms(300);
			LEDS_OFF();							delayInterrupt_ms(300);
			// blink led

		}

		// HWREG(SOC_GPIO_2_REGS+GPIO_IRQSTATUS_SET_0) |= (1<<TIMER7) | (1<<TIMER6) | (1<<GPIO2_1) | (1<<TIMER5);

		_levels_ = _max_levels_;			// quantidade fixa de fases
											// inicia sempre da primeira fase
		
		// valida a sequencia
		while(_levels_ > _current_levels_){
			LEDS_OFF();
			_press_buffer_ = WAIT_BUTTON;	//
			while(_press_buffer_ == WAIT_BUTTON){}; // enquanto a interrupcao for chamada 
			
			LEDS(_array_levels_[_levels_]);
			putString(0, "valor correto\n\r", 16);
			delayInterrupt_ms(300);

			// HWREG(SOC_GPIO_1_REGS+GPIO_SETDATAOUT) |= (1<<_press_buffer_);

		};

		--_current_levels_;			// quantidade de fases aumentou 1

		if(_current_levels_< 0){
			_uint32 i = 0;
			for(i = _max_levels_-1 ; i >= 0 ; --i){
				LEDS(_array_levels_[i]);
				delayInterrupt_ms(350);
				LEDS_OFF();
				delayInterrupt_ms(350);
			}
			// reiniciar o jogo
			// resetar level
			// resetar current_level
			_reset_status_ &= ~(RESTART_GAME);
			_reset_status_ |= RESTART_GAME; 
		}

		// FIM DA IMPLEMENTACAO DO GENIUS		
		HWREG(SOC_GPIO_1_REGS+GPIO_CLEARDATAOUT) = (1<<USR3);
	}

	return(0);
}/* end of function main */

void sysconfig( ){

	interrupts(WDT1INT, IRL_priorityMax, IRQ);
	
	interrupts(TINT7, IRL_priorityMax, IRQ);
    interrupts(GPIOINT1B, IRL_priorityMax, IRQ);
    interrupts(GPIOINT1A, IRL_priorityMax, IRQ);

    // Configuração de GPIOs
    Init_module_gpio(GPIO1);
    Init_module_gpio(GPIO2);

    Init_pin_gpio(GPIO1, OUTPUT, USR0);
    Init_pin_gpio(GPIO1, OUTPUT, USR1);
    Init_pin_gpio(GPIO1, OUTPUT, USR2);
    Init_pin_gpio(GPIO1, OUTPUT, USR3);
    Init_pin_gpio(GPIO2, OUTPUT, TIMER7);
	Init_pin_gpio(GPIO2, OUTPUT, TIMER6);
	Init_pin_gpio(GPIO2, OUTPUT, GPIO2_1);
	Init_pin_gpio(GPIO2, OUTPUT, GPIO2_2);
    Init_pin_gpio(GPIO2, OUTPUT, TIMER5);

    Init_pin_gpio(GPIO1, INPUT, BUTTON1);
    Init_pin_gpio(GPIO1, INPUT, BUTTON2);
    Init_pin_gpio(GPIO1, INPUT, BUTTON3);
    Init_pin_gpio(GPIO1, INPUT, BUTTON4);

    // Configurar interrupções GPIO
    interruptConfigGPIO(SOC_GPIO_1_REGS, 1, BUTTON1);
    interruptConfigGPIO(SOC_GPIO_1_REGS, 1, BUTTON2);
    interruptConfigGPIO(SOC_GPIO_1_REGS, 1, BUTTON3);
    interruptConfigGPIO(SOC_GPIO_1_REGS, 1, BUTTON4);

	HWREG(SOC_GPIO_1_REGS+GPIO_DEBOUNCENABLE) |= (1 << BUTTON1);
	HWREG(SOC_GPIO_1_REGS+GPIO_DEBOUNCENABLE) |= (1 << BUTTON2);
	HWREG(SOC_GPIO_1_REGS+GPIO_DEBOUNCENABLE) |= (1 << BUTTON3);
	HWREG(SOC_GPIO_1_REGS+GPIO_DEBOUNCENABLE) |= (1 << BUTTON4);
	HWREG(SOC_GPIO_1_REGS+GPIO_DEBOUNCINGTIME) = 32;

	// Init_pin_gpio(GPIO1, OUTPUT, 28);

}/*	end of function ledInit */


void inToString(unsigned number){
	char valor[15]={'0','0','0','0','0','0','0','0','0','0','0','0','0','0','0'};
	int i=15;

	while(number>0){
		valor[--i]=(number%10)+0x30;

		number/=10;
	}

	putString(0,valor,15);
	putString(0,"\n\r",3);
	// uartClearBuffer(0);
}

void LEDS(_uint8 option){
	switch (option)
	{
	case 1:
		HWREG(SOC_GPIO_2_REGS+GPIO_SETDATAOUT) |= (1<<TIMER7);
		break;

	case 2:
		HWREG(SOC_GPIO_2_REGS+GPIO_SETDATAOUT) |= (1<<TIMER6);
		break;

	case 3:
		HWREG(SOC_GPIO_2_REGS+GPIO_SETDATAOUT) |= (1<<GPIO2_1);
		break;

	case 4:
		HWREG(SOC_GPIO_2_REGS+GPIO_SETDATAOUT) |= (1<<TIMER5);
		break;
	default:
		break;
	}
}

void LEDS_OFF(){
		HWREG(SOC_GPIO_2_REGS+GPIO_CLEARDATAOUT) = (1<<TIMER7);
		HWREG(SOC_GPIO_2_REGS+GPIO_CLEARDATAOUT) = (1<<TIMER6);
		HWREG(SOC_GPIO_2_REGS+GPIO_CLEARDATAOUT) = (1<<GPIO2_1);
		HWREG(SOC_GPIO_2_REGS+GPIO_CLEARDATAOUT) = (1<<TIMER5);
}

void Win_or_Not(){

}
