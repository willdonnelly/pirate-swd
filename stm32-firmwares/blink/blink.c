#define STACK_TOP 0x20002000
#include "stm32f10x.h"

#define DELAY_COUNT 1000000

int main(void){
    RCC->APB2ENR |= 0x14;
    GPIOC->CRH = 0x11;

    while(1) {
        GPIOC->BRR |= 1<<8;
        delay(DELAY_COUNT);
        GPIOC->BSRR |= 1<<8;
        delay(DELAY_COUNT);
    }
}

void delay (unsigned int ii) {
    unsigned int jj;
    for(jj = ii; jj > 0; jj--)
        asm("nop");
}

/* do nothing functions */
int  assert_param     (void *v) { return 0; }
void handle_nmi       (void)    { return;   }
void handle_hardfault (void)    { return;   }

/* vector table */
unsigned int *myvectors[4] __attribute__ ((section("vectors")))= {
    (unsigned int *) STACK_TOP,       // stack pointer
    (unsigned int *) main,            // code entry point
    (unsigned int *) handle_nmi,      // handle non-maskable interrupts
    (unsigned int *) handle_hardfault // handle hard faults
};
