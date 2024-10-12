#include <stdio.h>
#include <stdint.h>

#define struct_name  novo

typedef struct struct_name block;

struct struct_name{
    unsigned char flag0:1;
    unsigned char flag1:1;
    unsigned char flag2:1;
    unsigned char flag3:1;
}var={flag0:1, flag1:1, flag2:1, flag3:1};

int main()
{
    // flag3, flag2, flag1,flag 0
    block no__={0,0,0,0};
    uint32_t n=*(uint32_t*)(&no__);
    
    while(n){
        printf("passou\n");  
    }
    
    printf("%d\n",no__);     
    printf("%d\n",n);

    

    return 0;
}
