#ifndef _GPIO_REGISTERS_
#define _GPIO_REGISTERS_

#pragma region GPIO REGISTERS

#define GPIO_REVISION                   (0x0u) 
#define GPIO_SYSCONFIG                  (0x10u) 
#define GPIO_EOI                        (0x20u) 
#define GPIO_IRQSTATUS_RAW_0            (0x24u) 
#define GPIO_IRQSTATUS_RAW_1            (0x28u) 
#define GPIO_IRQSTATUS_0                (0x2Cu) 
#define GPIO_IRQSTATUS_1                (0x30u) 
#define GPIO_IRQSTATUS_SET_0            (0x34u) 
#define GPIO_IRQSTATUS_SET_1            (0x38u) 
#define GPIO_IRQSTATUS_CLR_0            (0x3Cu) 
#define GPIO_IRQSTATUS_CLR_1            (0x40u)
#define GPIO_IRQWAKEN_0                 (0x44u) 
#define GPIO_IRQWAKEN_1                 (0x48u) 
#define GPIO_SYSSTATUS                  (0x114u) 
#define GPIO_CTRL                       (0x130u) 
#define GPIO_OE                         (0x134u) 
#define GPIO_DATAIN                     (0x138u) 
#define GPIO_DATAOUT                    (0x13Cu) 
#define GPIO_LEVELDETECT0               (0x140u) 
#define GPIO_LEVELDETECT1               (0x144u) 
#define GPIO_RISINGDETECT               (0x148u) 
#define GPIO_FALLINGDETECT              (0x14Cu) 
#define GPIO_DEBOUNCENABLE              (0x150u) 
#define GPIO_DEBOUNCINGTIME             (0x154u) 
#define GPIO_CLEARDATAOUT               (0x190u) 
#define GPIO_SETDATAOUT                 (0x194u) 

#pragma endregion GPIO REGISTERS

/**
 * @brief GPIO pin number (0-31)
 */
typedef unsigned char ucPinNumber;

typedef enum GroupInterrup{
	type0 = 0,
	type1
}GroupInterrup;

/**
 * @brief Logic HIGH (1) or logic LOW (0).
 */
typedef enum Level{
	LOW,
	HIGH
}Level;

/**
 * @brief GPIO port number (GPIO0-GPIO3).
 **/
typedef enum gpioMod{
	GPIO0,
	GPIO1,
	GPIO2,
	GPIO3
}gpioMod;

/**
 * @brief GPIO pin direction (INPUT/OUTPUT).
 **/
typedef enum Direction{
	OUTPUT,
	INPUT
}Direction;

/**
 * @fn void GPIO_initPort(GPIO_t port)
 * @brief      Initialize port.
 * @param[in]  port  Port to initialize.
 * @return     void
 **/
void Init_module_gpio(gpioMod);

/**
 * @fn void GPIO_initPin(GPIO_t port, pin_t pin)
 * @brief      Initialize port.pin.
 * @param[in]  port  Parent port of pin.
 * @param[in]  pin   Pin of port to initialize.
 * @return     void
 **/
void Init_pin_gpio(gpioMod , Direction, unsigned char);

/**
 * @fn void GPIO_setDirection(GPIO_t port, pin_t pin, pin_direction_t dir)
 * @brief      Set the direction for port.pin.
 * @param[in]  port  Parent port of pin.
 * @param[in]  pin   Pin of port to set the direction of.
 * @param[in]  dir   Direction to set port.pin to.
 * @return     void
 **/
void Set_direction_pin_gpio(gpioMod, Direction, unsigned char);

/**
 * @fn pin_direction_t GPIO_getDirection(GPIO_t port, pin_t pin)
 * @brief      Get the port.pin direction.
 * @param[in]  port  Parent port of pin.
 * @param[in]  pin   Pin of port to get the direction of.
 * @return           The direction of port.pin.
 **/
unsigned int Get_direction_pin_gpio(gpioMod , unsigned char);

/**
 * @fn void GPIO_setPin(GPIO_t port, pin_t pin)
 * @brief            Set port.pin to HIGH (port.pin must be set as OUTPUT previously).
 * @param[in]  port  Parent port of pin.
 * @param[in]  pin   Pin of port to set.
 * @return           void
 **/
void GpioSetPinValue(gpioMod , ucPinNumber , Level);

void GpioClearPinValue(gpioMod, ucPinNumber);

/**
 * @fn level_t GPIO_getPin(GPIO_t port, pin_t pin)
 * @brief            Get the level of port.pin (port.pin must be set as INPUT previously).
 * @param[in]  port  Parent port of pin.
 * @param[in]  pin   Pin of port to get.
 * @return           The level of port.pin.
 **/
unsigned int GpioGetPinValue(gpioMod , ucPinNumber);


int gpioIsrHandler(gpioMod , GroupInterrup , ucPinNumber);
int Pin_Interrup_Config(gpioMod , ucPinNumber , GroupInterrup);

/**
 * @fn void GPIO_setPort(GPIO_t port, uint32_t value)
 * @brief            Set the data on the port.
 * @param[in]  port  Port to write.
 * @param[in]  value Value to write on the port
 * @return           void
 **/
void GPIO_setPort(gpioMod port, unsigned int value);

/**
 * @fn uint32_t GPIO_getPort(GPIO_t port)
 * @brief            Get the data on the port.
 * @param[in] port   Port to read.
 * @return           data read on the port.
 **/
unsigned int GPIO_getPort(gpioMod port);

#endif