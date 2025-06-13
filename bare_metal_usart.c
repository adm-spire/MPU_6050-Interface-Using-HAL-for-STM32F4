
#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>

void SysTick_Handler(void);
void delay_ms(uint32_t ms);
void systick_init(void);
void uart2_init(void);
char uart2_read(void);
void uart2_print(char *str);

int main(void){
	systick_init();
	    uart2_init();

	    uart2_print("Hello from STM32F401CCU6!\r\n");


   while(1){
	   uart2_print("Delay 1s...\r\n");
	           delay_ms(1000);





   }
}

volatile uint32_t msTicks = 0;

void SysTick_Handler(void) {
    msTicks++;
}
void delay_ms(uint32_t ms) {
    uint32_t start = msTicks;
    while ((msTicks - start) < ms);
}
void systick_init(void) {
    // Set reload for 1ms interrupt (16MHz / 1000 = 16000)
    SysTick->LOAD = 16000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
}

void uart2_init(void) {
    // Enable GPIOA and USART2 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // PA2 -> TX, PA3 -> RX
    GPIOA->MODER &= ~((3U << (2 * 2)) | (3U << (3 * 2))); // Clear
    GPIOA->MODER |= (2U << (2 * 2)) | (2U << (3 * 2));    // Alternate function

    GPIOA->AFR[0] |= (7U << (4 * 2)) | (7U << (4 * 3));   // AF7 for USART2

    USART2->BRR = 0x0683; // 9600 baud @16 MHz (simplified value)
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void uart2_write(char ch) {
    while (!(USART2->SR & USART_SR_TXE));
    USART2->DR = (ch & 0xFF);
}

char uart2_read(void) {
    while (!(USART2->SR & USART_SR_RXNE));
    return (char)(USART2->DR & 0xFF);
}

void uart2_print(char *str) {
    while (*str) {
        uart2_write(*str++);
    }
}
