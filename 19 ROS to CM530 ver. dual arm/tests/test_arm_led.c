/* Exercise the real GPIO adapter; only register/library access is simulated. */
#include <stdio.h>
#include "stm32f10x_lib.h"
#include "arm_led.h"
GPIO_TypeDef testGPIOB, testGPIOC;
static unsigned int clocks;
static int failures, calls, lastWasOff;
#define CHECK(c) do { if(!(c)){fprintf(stderr,"LED FAIL line %d: %s\n",__LINE__,#c);failures++;} } while(0)
void RCC_APB2PeriphClockCmd(unsigned int mask,int enabled)
{CHECK(enabled==1);clocks|=mask;}
void GPIO_StructInit(GPIO_InitTypeDef *c)
{c->GPIO_Pin=c->GPIO_Speed=c->GPIO_Mode=0;}
void GPIO_Init(GPIO_TypeDef *port,GPIO_InitTypeDef *c)
{
    CHECK(c->GPIO_Pin==(port==GPIOB?0xF000U:0xC000U));
    CHECK(c->GPIO_Speed==2 && c->GPIO_Mode==0x10);
    CHECK((port->ODR&c->GPIO_Pin)==c->GPIO_Pin); /* off before output */
    port->configured|=c->GPIO_Pin;
}
void GPIO_SetBits(GPIO_TypeDef *port,u16 pins)
{
    CHECK((clocks&(port==GPIOB?0x08U:0x10U))!=0);
    CHECK((pins&~(port==GPIOB?0xF000U:0xC000U))==0);
    port->ODR|=pins;calls++;lastWasOff=1;
}
void GPIO_ResetBits(GPIO_TypeDef *port,u16 pins)
{
    CHECK(lastWasOff);lastWasOff=0;calls++;
    CHECK((port->configured&pins)==pins);
    port->ODR&=~pins;
    CHECK((port->ODR&(port==GPIOB?0x6000U:0xC000U))!=0); /* never both on */
}
int main(void)
{
    unsigned int b,c;
    testGPIOB.ODR=0x0355;testGPIOC.ODR=0x2355; /* POWER PC13 initially high */
    ArmLedInit();CHECK(clocks==0x18);CHECK(calls==2);
    CHECK(testGPIOB.ODR==0xF355 && testGPIOC.ODR==0xE355);
    ArmLedSet(0,0);ArmLedSet(1,0);
    CHECK(testGPIOB.ODR==0xB355 && testGPIOC.ODR==0x6355); /* both red */
    c=testGPIOC.ODR;ArmLedSet(0,1);
    CHECK(testGPIOB.ODR==0xD355 && testGPIOC.ODR==c); /* left green only */
    b=testGPIOB.ODR;ArmLedSet(1,1);
    CHECK(testGPIOB.ODR==b && testGPIOC.ODR==0xA355); /* both green */
    ArmLedSet(0,0);CHECK(testGPIOB.ODR==0xB355 && testGPIOC.ODR==0xA355);
    ArmLedSet(1,0);CHECK(testGPIOC.ODR==0x6355);
    ArmLedSet(1,0);CHECK(testGPIOC.ODR==0x6355);
    CHECK((testGPIOB.ODR&0x9000)==0x9000); /* AUX/PLAY off */
    CHECK((testGPIOC.ODR&0x2000)==0x2000); /* POWER unchanged */
    if(failures)return 1;
    puts("PASS: LED GPIO mapping, active-low, mutual exclusion, port clocks and untouched POWER/UART pins");
    return 0;
}
