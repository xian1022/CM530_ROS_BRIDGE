#include "stm32f10x_lib.h"
#include "arm_led.h"

/* User-confirmed board colors. All LED outputs are active low. */
#define ARM1_GREEN GPIO_Pin_13 /* MANAGE */
#define ARM1_RED   GPIO_Pin_14 /* PROGRAM */
#define ARM2_GREEN GPIO_Pin_14 /* TX */
#define ARM2_RED   GPIO_Pin_15 /* RX */
#define UNUSED_LEDS (GPIO_Pin_12 | GPIO_Pin_15) /* AUX, PLAY on GPIOB */

void ArmLedInit(void)
{
    GPIO_InitTypeDef config;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
    /* Preload off before enabling outputs, avoiding a low-going setup flash. */
    GPIO_SetBits(GPIOB, ARM1_GREEN | ARM1_RED | UNUSED_LEDS);
    GPIO_SetBits(GPIOC, ARM2_GREEN | ARM2_RED);
    GPIO_StructInit(&config);
    config.GPIO_Speed = GPIO_Speed_2MHz;
    config.GPIO_Mode = GPIO_Mode_Out_PP;
    config.GPIO_Pin = ARM1_GREEN | ARM1_RED | UNUSED_LEDS;
    GPIO_Init(GPIOB, &config);
    config.GPIO_Pin = ARM2_GREEN | ARM2_RED;
    GPIO_Init(GPIOC, &config);
    /* BridgeInit sets both arms to red standby before motor initialization. */
}

void ArmLedSet(int arm, int moving)
{
    GPIO_TypeDef *port = arm == 0 ? GPIOB : GPIOC;
    u16 green = arm == 0 ? ARM1_GREEN : ARM2_GREEN;
    u16 red = arm == 0 ? ARM1_RED : ARM2_RED;
    /* Break before make: never illuminate both colors at once. */
    GPIO_SetBits(port, green | red);
    GPIO_ResetBits(port, moving ? green : red);
}
