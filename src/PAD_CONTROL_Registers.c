#include "PAD_CONTROL_Registers.h"

void padControlSetMode(CONTROL_MODULE conf_module_pin, muxmode mode){
	if((CM_conf_gpmc_ad0 <= conf_module_pin) && (conf_module_pin <= CM_conf_usb1_drvvbus)){
		unsigned int temp = controlModuleGetPad(conf_module_pin);
		temp &= ~(0b111);
		temp |= mode;
		controlModuleSetPad(conf_module_pin, temp);
	}else{
		// TODO: raise error (control module isnt a "conf <module> <pin>" register)
	}
}

muxmode padControlGetMode(CONTROL_MODULE conf_module_pin){
	if((CM_conf_gpmc_ad0 <= conf_module_pin) && (conf_module_pin <= CM_conf_usb1_drvvbus)){
		unsigned int temp = controlModuleGetPad(conf_module_pin);
		temp &= ~(0b111);
		return ((muxmode) temp);
	}else{
		// TODO: raise error (control module isnt a "conf <module> <pin>" register)
	return(-1);
	}
}