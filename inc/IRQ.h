#ifndef __IRG_H__
#define __IRG_H__

unsigned int mode;

void irqHandler(){
    irqGroup();
}

void irqGroup(){
    mode=20;
}

unsigned retornoIRQ(){
    return mode;    
}

#endif