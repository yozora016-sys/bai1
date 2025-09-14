#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
#include "misc.h"

volatile uint32_t msTicks = 0;

void SysTick_Handler(void) {
    msTicks++;
}

void DelayMs(uint32_t ms) {
    uint32_t curTicks = msTicks;
    while ((msTicks - curTicks) < ms);
}

void GPIO_Config(void) {
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);

    // LED PC13 output
    gpio.GPIO_Pin = GPIO_Pin_13;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &gpio);

    // LED PA1 output
    gpio.GPIO_Pin = GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &gpio);

    // Button PA0 input (pull-down)
    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &gpio);
}

void EXTI_Config(void) {
    EXTI_InitTypeDef exti;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);

    exti.EXTI_Line = EXTI_Line0;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    nvic.NVIC_IRQChannel = EXTI0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0x0F;
    nvic.NVIC_IRQChannelSubPriority = 0x0F;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

void EXTI0_IRQHandler(void) {
    if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
        GPIOA->ODR ^= GPIO_Pin_1;   // Ð?o tr?ng thái LED PC13
        EXTI_ClearITPendingBit(EXTI_Line0);
    }
}

int main(void) {
    GPIO_Config();
    EXTI_Config();
    SysTick_Config(SystemCoreClock / 1000); // 1ms tick

    while (1) {
        GPIOC->ODR ^= GPIO_Pin_13; // LED2 nh?p nháy 1Hz
        DelayMs(500);
    }
}