#include "types.h"

#ifndef __RANDOM__
#define __RANDOM__

// Função Xorshift para gerar números aleatórios
_uint32 xorshift32(_uintptr32 state)
{
    _uint32 x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return ((x % 4) + 1);
}

// static _uint32 random_seed = (unsigned int)time(NULL);
static _uint32 random_seed;

// static _uint32 random_value = xorshift32(&random_seed);
static _uint32 random_value;

#endif