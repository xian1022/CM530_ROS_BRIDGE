/* Host-only seam for the GPIO/RCC calls used by the real arm_led.c. */
#ifndef TEST_STM32_GPIO_H
#define TEST_STM32_GPIO_H
typedef unsigned short u16;
typedef struct { unsigned int ODR, configured; } GPIO_TypeDef;
typedef struct { unsigned int GPIO_Pin, GPIO_Speed, GPIO_Mode; } GPIO_InitTypeDef;
extern GPIO_TypeDef testGPIOB, testGPIOC;
#define GPIOB (&testGPIOB)
#define GPIOC (&testGPIOC)
#define GPIO_Pin_12 0x1000
#define GPIO_Pin_13 0x2000
#define GPIO_Pin_14 0x4000
#define GPIO_Pin_15 0x8000
#define RCC_APB2Periph_GPIOB 0x08
#define RCC_APB2Periph_GPIOC 0x10
#define ENABLE 1
#define GPIO_Speed_2MHz 2
#define GPIO_Mode_Out_PP 0x10
void RCC_APB2PeriphClockCmd(unsigned int mask, int enabled);
void GPIO_StructInit(GPIO_InitTypeDef *config);
void GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *config);
void GPIO_SetBits(GPIO_TypeDef *port, u16 pins);
void GPIO_ResetBits(GPIO_TypeDef *port, u16 pins);
#endif
