#include "hw_types.h"
#include "soc_AM335x.h"
#include "CONTROL_MODULE_Registers.h"
#include "CLOCK_MODULE_Registers.h"
#include "GPIO_Registers.h"
#include "INTERRUPT_SERVICE_ROUTINE.h"
#include "PAD_CONTROL_Registers.h"

unsigned int numberGPIO[4] = { SOC_GPIO_0_REGS, SOC_GPIO_1_REGS, SOC_GPIO_2_REGS, SOC_GPIO_3_REGS };

CONTROL_MODULE GPIO_CTRL_MODULE_ARRAY[32][4] = {
    //GPIO0                       //GPIO1                       //GPIO2                       //GPIO3
    {CM_conf_mdio                 ,CM_conf_gpmc_ad0             ,CM_conf_gpmc_csn3            ,CM_conf_mii1_col       },//.0
    {CM_conf_mdc                  ,CM_conf_gpmc_ad1             ,CM_conf_gpmc_clk             ,CM_conf_mii1_crs       },//.1
    {CM_conf_spi0_sclk            ,CM_conf_gpmc_ad2             ,CM_conf_gpmc_advn_ale        ,CM_conf_mii1_rx_er     },//.2
    {CM_conf_spi0_d0              ,CM_conf_gpmc_ad3             ,CM_conf_gpmc_oen_ren         ,CM_conf_mii1_tx_en     },//.3
    {CM_conf_spi0_d1              ,CM_conf_gpmc_ad4             ,CM_conf_gpmc_wen             ,CM_conf_mii1_rx_dv     },//.4
    {CM_conf_spi0_cs0             ,CM_conf_gpmc_ad5             ,CM_conf_gpmc_ben0_cle        ,CM_conf_i2c0_sda       },//.5
    {CM_conf_spi0_cs1             ,CM_conf_gpmc_ad6             ,CM_conf_lcd_data0            ,CM_conf_i2c0_scl       },//.6
    {CM_conf_ecap0_in_pwm0_out    ,CM_conf_gpmc_ad7             ,CM_conf_lcd_data1            ,CM_conf_emu0           },//.7
    {CM_conf_lcd_data12           ,CM_conf_uart0_ctsn           ,CM_conf_lcd_data2            ,CM_conf_emu1           },//.8
    {CM_conf_lcd_data13           ,CM_conf_uart0_rtsn           ,CM_conf_lcd_data3            ,CM_conf_mii1_tx_clk    },//.9
    {CM_conf_lcd_data14           ,CM_conf_uart0_rxd            ,CM_conf_lcd_data4            ,CM_conf_mii1_rx_clk    },//.10
    {CM_conf_lcd_data15           ,CM_conf_uart0_txd            ,CM_conf_lcd_data5            ,-1                     },//.11
    {CM_conf_uart1_ctsn           ,CM_conf_gpmc_ad12            ,CM_conf_lcd_data6            ,-1                     },//.12
    {CM_conf_uart1_rtsn           ,CM_conf_gpmc_ad13            ,CM_conf_lcd_data7            ,CM_conf_usb1_drvvbus   },//.13
    {CM_conf_uart1_rxd            ,CM_conf_gpmc_ad14            ,CM_conf_lcd_data8            ,CM_conf_mcasp0_aclkx   },//.14
    {CM_conf_uart1_txd            ,CM_conf_gpmc_ad15            ,CM_conf_lcd_data9            ,CM_conf_mcasp0_fsx     },//.15
    {CM_conf_mii1_txd3            ,CM_conf_gpmc_a0              ,CM_conf_lcd_data10           ,CM_conf_mcasp0_axr0    },//.16
    {CM_conf_mii1_txd2            ,CM_conf_gpmc_a1              ,CM_conf_lcd_data11           ,CM_conf_mcasp0_ahclkr  },//.17
    {CM_conf_usb0_drvvbus         ,CM_conf_gpmc_a2              ,CM_conf_mii1_rxd3            ,CM_conf_mcasp0_aclkr   },//.18
    {CM_conf_xdma_event_intr0     ,CM_conf_gpmc_a3              ,CM_conf_mii1_rxd2            ,CM_conf_mcasp0_fsr     },//.19
    {CM_conf_xdma_event_intr1     ,CM_conf_gpmc_a4              ,CM_conf_mii1_rxd1            ,CM_conf_mcasp0_axr1    },//.20
    {CM_conf_mii1_txd1            ,CM_conf_gpmc_a5              ,CM_conf_mii1_rxd0            ,CM_conf_mcasp0_ahclkx  },//.21
    {CM_conf_gpmc_ad8             ,CM_conf_gpmc_a6              ,CM_conf_lcd_vsync            ,-1                     },//.22
    {CM_conf_gpmc_ad9             ,CM_conf_gpmc_a7              ,CM_conf_lcd_hsync            ,-1                     },//.23
    {-1                           ,CM_conf_gpmc_a8              ,CM_conf_lcd_pclk             ,-1                     },//.24
    {-1                           ,CM_conf_gpmc_a9              ,CM_conf_lcd_ac_bias_en       ,-1                     },//.25
    {CM_conf_gpmc_ad10            ,CM_conf_gpmc_a10             ,CM_conf_mmc0_dat3            ,-1                     },//.26
    {CM_conf_gpmc_ad11            ,CM_conf_gpmc_a11             ,CM_conf_mmc0_dat2            ,-1                     },//.27
    {CM_conf_mii1_txd0            ,CM_conf_gpmc_ben1            ,CM_conf_mmc0_dat1            ,-1                     },//.28
    {CM_conf_rmii1_ref_clk        ,CM_conf_gpmc_csn0            ,CM_conf_mmc0_dat0            ,-1                     },//.29
    {CM_conf_gpmc_wait0           ,CM_conf_gpmc_csn1            ,CM_conf_mmc0_clk             ,-1                     },//.30
    {CM_conf_gpmc_wpn             ,CM_conf_gpmc_csn2            ,CM_conf_mmc0_cmd             ,-1                     },//.31
};

static bool GPIO_checkValidPortPin(gpioMod port, unsigned char pin)
{
    // TODO: raise error (port is either too big or negative: /port)
    // TODO: raise error (pin is either too big or negative: /pin)
    // TODO: raise error (pin/port combinaison isnt present on this chip: /port./pin)
    // return ((port < GPIO0) || (port > GPIO3) || (pin < 0) || (pin > 31) || (GPIO_CTRL_MODULE_ARRAY[pin][port] == -1)) ? false: true;
    return 1;
}

static bool GPIO_CheckValidDirection(Direction dir)
{
    // TODO: raise error (direction not valid: /dir)
    // return ((dir != OUTPUT) && (dir != INPUT)) ? false: true;
    return 1;
}

void Init_module_gpio(gpioMod GPIO){
   if(GPIO_checkValidPortPin(GPIO,0))  // pin 0 always exist
   {
      unsigned int setting = (1<<18) | (0x2<<0);   //enable functional clock & enable module, TRM 8.1.12.1.32
      switch(GPIO)
      {
         case GPIO0:
            return;        // GPIO0 doesnt have a clock module register, TRM 8.1.12.1
            break;
         case GPIO1:
            ckmSetCLKModuleRegister(CLKM_PER, CLKM_PER_GPIO1_CLKCTRL, setting);
            while((ckmGetCLKModuleRegister(CLKM_PER, CLKM_PER_GPIO1_CLKCTRL) & (0x3<<16)) != 0)
            break;
         case GPIO2:
            ckmSetCLKModuleRegister(CLKM_PER, CLKM_PER_GPIO2_CLKCTRL, setting);
            while((ckmGetCLKModuleRegister(CLKM_PER, CLKM_PER_GPIO2_CLKCTRL) & (0x3<<16)) != 0)
            break;
         case 3:
            ckmSetCLKModuleRegister(CLKM_PER, CLKM_PER_GPIO3_CLKCTRL, setting);
            while((ckmGetCLKModuleRegister(CLKM_PER, CLKM_PER_GPIO3_CLKCTRL) & (0x3<<16)) != 0)
            break;
         default:
            // TODO: raise error (not possible, checked port before: /port)
            break;
      }
   }
}

void Init_pin_gpio(gpioMod GPIO, Direction dir, unsigned char pin){
    if(GPIO_checkValidPortPin(GPIO, pin)){
        CONTROL_MODULE module = GPIO_CTRL_MODULE_ARRAY[pin][GPIO];  // get conf <module> <pin> for port/pin combination
        padControlSetMode(module, MODE_7);  //set mode to GPIO, Datasheet 4.3.2
    }

    Set_direction_pin_gpio(GPIO, dir, pin);
}

void Set_direction_pin_gpio(gpioMod GPIO, Direction dir, unsigned char pin){
    if(GPIO_checkValidPortPin(GPIO, pin)){
        if(GPIO_CheckValidDirection(dir)){
            unsigned int temp = numberGPIO[GPIO];
            HWREG(temp+GPIO_OE) &= ~(1<<pin);
            HWREG(temp+GPIO_OE) |= dir<<pin;   
        }
    }
}

unsigned int Get_direction_pin_gpio(gpioMod GPIO, unsigned char pin){
    if(GPIO_checkValidPortPin(GPIO, 0)){
        return ((HWREG(numberGPIO[GPIO]+GPIO_OE) & (1<<pin))>>pin);
    }
    else{
        return -1;
    }
}

void GpioSetPinValue(gpioMod GPIO, ucPinNumber pin, Level set){
    if(GPIO_checkValidPortPin(GPIO, pin)){
        HWREG(numberGPIO[GPIO]+GPIO_SETDATAOUT) = (set<<pin);
    }
}

void GpioClearPinValue(gpioMod GPIO, ucPinNumber pin){
    if(GPIO_checkValidPortPin(GPIO, pin)){
        HWREG(numberGPIO[GPIO]+GPIO_CLEARDATAOUT) = (1<<pin);
    }
}

unsigned int GpioGetPinValue(gpioMod GPIO, ucPinNumber pin){
    if(GPIO_checkValidPortPin(GPIO, pin)){
        return ((HWREG(numberGPIO[GPIO]+GPIO_DATAIN) & (1<<pin))>>pin);
    }
    else{
        return -1;
    }
}

int gpioIsrHandler(gpioMod , GroupInterrup , ucPinNumber){

}

int Pin_Interrup_Config(gpioMod GPIO, ucPinNumber gpio_pin, GroupInterrup POINTRPEND){
    unsigned int interruptIntNumber = 0;
    if(GPIO0){
        interruptIntNumber = 96+POINTRPEND;
    }

    if(GPIO1){
        interruptIntNumber = 98+POINTRPEND;
    }

    if(GPIO2){
        interruptIntNumber = 32+POINTRPEND;
    }

    if(GPIO3){
        interruptIntNumber = 62+POINTRPEND;
    }
    interrupts(interruptIntNumber, 0x3F, 1);
    interruptConfigGPIO(numberGPIO[GPIO], POINTRPEND, gpio_pin);
}


// uint32_t GPIO_getPort(GPIO_t port)
// {
//    if(GPIO_checkValidPortPin(port,0)) // pin 0 always exist
//    {
//       uint32_t addr_temp = GPIO_BASE_ARRAY[port] + GPIO_DATAIN;  // GPIOx base + data in offset, TRM 2.1 & 25.4.1.17
//       return GET32(addr_temp);
//    }
//    return -1;
// }


// void GPIO_setPort(GPIO_t port, uint32_t value)
// {
//    if(GPIO_checkValidPortPin(port,0)) // pin 0 always exist
//    {
//       uint32_t addr_temp = GPIO_BASE_ARRAY[port] + GPIO_DATAOUT;  // GPIOx base + data in offset, TRM 2.1 & 25.4.1.18
//       PUT32(addr_temp, value);
//    }
// }