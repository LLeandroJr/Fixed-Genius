#include "hw_types.h"
#include "soc_AM335x.h"
#include "CONTROL_MODULE_Registers.h"
#include "CLOCK_MODULE_Registers.h"
#include "GPIO_Registers.h"
#include "INTERRUPT_SERVICE_ROUTINE.h"
#include "TIMERS_Registers.h"
#include "uart.h"

#include "types.h"


int interruptServiceRoutine_Setup(unsigned int inter){

}

static _uint32 m[21];

void buzzer(_uint8 buzzerID){
    
    switch (buzzerID)
    {
    case 1:

        break;
    
    case 2:

        break;
    
    case 3:

        break;
    
    case 4:

        break;
    
    default:
        break;
    }
}


static unsigned int seed = 1;

// Função para inicializar a semente do gerador de números aleatórios
void srand_custom(unsigned int new_seed) {
    seed = new_seed;
}

// Função para gerar um número pseudo-aleatório
int rand_custom(void) {
    seed = seed * 1103515245 + 12345;   // Parâmetros do gerador congruencial linear
    return (seed / 65536) % 32768;
}

// Função para inverter uma string
void reverse(char* str, int length) {
    int start = 0;
    int end = length - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
}

// Função para converter um inteiro para string
int intToStr(int num, char* str) {
    int i = 0;
    int isNegative = 0;

    // Se o número for negativo, converte para positivo
    if (num < 0) {
        isNegative = 1;
        num = -num;
    }

    // Extrai os dígitos do número
    do {
        str[i++] = (num % 10) + '0';
        num /= 10;
    } while (num != 0);

    // Adiciona o sinal negativo se necessário
    if (isNegative) {
        str[i++] = '-';
    }

    // Inverte a string para corrigir a ordem dos dígitos
    reverse(str, i);

    // Adiciona o caractere de fim de string
    str[i] = '\0';

    return i;
}