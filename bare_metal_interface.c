
#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>

#define SLAVE_ADDR  0x68

typedef struct {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
} mpu6050_data_t;

void SysTick_Handler(void);
void delay_ms(uint32_t ms);
void systick_init(void);
void uart2_init(void);
char uart2_read(void);
void uart2_print(char *str);
void i2c1_init(void);
void i2c1_start(void);
void i2c1_write_addr(uint8_t addr);
void i2c1_write(uint8_t data) ;
uint8_t i2c1_read_nack(void);
uint8_t i2c1_read_ack(void);
void i2c1_stop(void);
void mpu6050_init(void);
uint8_t mpu6050_who_am_i(void) ;
int16_t mpu6050_read_word(uint8_t reg_addr) ;

void mpu6050_read_all(mpu6050_data_t* data);


int main(void){
	systick_init();
	    uart2_init();
	    delay_ms(7000);

	    i2c1_init();


	        mpu6050_init();




	        uint8_t whoami = mpu6050_who_am_i(); // Should be 0x68

	        mpu6050_data_t mpu_data;
	        char buffer[128];
	        uart2_print("started\n");



   while(1){

	   mpu6050_read_all(&mpu_data);
	   uart2_print("started2\n");
	   int ax_g = mpu_data.ax / 8192;
	   	    int ay_g = mpu_data.ay / 8192;
	   	    int az_g = mpu_data.az / 8192;

	   	    int gx_dps = mpu_data.gx / 65;
	   	    int gy_dps = mpu_data.gy / 65;
	   	    int gz_dps = mpu_data.gz / 65;

	   	    sprintf(buffer, "AX: %d g\n", ax_g);
	   	    uart2_print(buffer);
	   	    sprintf(buffer, "AY: %d g\n", ay_g);
	   	    uart2_print(buffer);
	   	    sprintf(buffer, "AZ: %d g\n", az_g);
	   	    uart2_print(buffer);

	   	    sprintf(buffer, "GX: %d dps\n", gx_dps);
	   	    uart2_print(buffer);
	   	    sprintf(buffer, "GY: %d dps\n", gy_dps);
	   	    uart2_print(buffer);
	   	    sprintf(buffer, "GZ: %d dps\n\n", gz_dps);
	   	    uart2_print(buffer);






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

void i2c1_init(void) {
    // Enable clocks for GPIOB and I2C1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Configure PB6 (SCL) and PB7 (SDA) to AF4, open-drain, high-speed
    GPIOB->MODER &= ~((3U << (6 * 2)) | (3U << (7 * 2)));
    GPIOB->MODER |= (2U << (6 * 2)) | (2U << (7 * 2));  // Alternate function
    GPIOB->AFR[0] |= (4U << (6 * 4)) | (4U << (7 * 4)); // AF4 for I2C
    GPIOB->OTYPER |= (1U << 6) | (1U << 7);            // Open-drain
    GPIOB->OSPEEDR |= (3U << (6 * 2)) | (3U << (7 * 2)); // High speed
    GPIOB->PUPDR |= (1U << (6 * 2)) | (1U << (7 * 2)); // Pull-up

    // Reset I2C1
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    // Configure I2C for 100kHz assuming 16MHz PCLK1
    I2C1->CR2 = 16;            // Peripheral clock = 16 MHz
    I2C1->CCR = 80;            // 100kHz = 16MHz / (2 * 80)
    I2C1->TRISE = 17;          // Max rise time = 1000ns / (1/16MHz) + 1

    // Enable I2C
    I2C1->CR1 |= I2C_CR1_PE;
}


void i2c1_start(void) {
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)); // Wait for start bit to be sent
}

void i2c1_write_addr(uint8_t addr) {
    I2C1->DR = addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)); // Wait for address to be sent
    (void)I2C1->SR2; // Clear ADDR by reading SR2
}

void i2c1_write(uint8_t data) {
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF)); // Wait for byte to finish transmitting
}
uint8_t i2c1_read_ack(void) {
    I2C1->CR1 |= I2C_CR1_ACK;
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    return I2C1->DR;
}
uint8_t i2c1_read_nack(void) {
    I2C1->CR1 &= ~I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_STOP;
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    return I2C1->DR;
}

void i2c1_stop(void) {
    I2C1->CR1 |= I2C_CR1_STOP;
}

void mpu6050_init(void) {
    i2c1_start();
    i2c1_write_addr(0x68 << 1);  // Write mode
    i2c1_write(0x6B);            // Power management register
    i2c1_write(0x00);            // Set to 0 to wake up MPU6050
    i2c1_stop();

    // Set accelerometer to ±4g
    i2c1_start();
    i2c1_write_addr(0x68 << 1);
    i2c1_write(0x1C);               // ACCEL_CONFIG
    i2c1_write(0x08);               // 0b00001000 => ±4g (bits 4:3 = 01)
    i2c1_stop();

    // Set gyroscope to ±500°/s
    i2c1_start();
    i2c1_write_addr(0x68 << 1);
    i2c1_write(0x1B);               // GYRO_CONFIG
    i2c1_write(0x08);               // 0b00001000 => ±500°/s (bits 4:3 = 01)
    i2c1_stop();

}

uint8_t mpu6050_who_am_i(void) {
    uint8_t id;
    i2c1_start();
    i2c1_write_addr(0x68 << 1);  // Write
    i2c1_write(0x75);            // WHO_AM_I register
    i2c1_start();                // Repeated start
    i2c1_write_addr((0x68 << 1) | 1); // Read
    id = i2c1_read_nack();
    return id;
}

int16_t mpu6050_read_word(uint8_t reg_addr) {
    int16_t data;
    uint8_t high, low;

    i2c1_start();
    i2c1_write_addr(0x68 << 1);  // Write
    i2c1_write(reg_addr);        // Register to read
    i2c1_start();
    i2c1_write_addr((0x68 << 1) | 1); // Read
    high = i2c1_read_ack();
    low = i2c1_read_nack();
    data = (high << 8) | low;
    return data;
}


void mpu6050_read_all(mpu6050_data_t* data) {
    data->ax = mpu6050_read_word(0x3B);
    data->ay = mpu6050_read_word(0x3D);
    data->az = mpu6050_read_word(0x3F);
    data->gx = mpu6050_read_word(0x43);
    data->gy = mpu6050_read_word(0x45);
    data->gz = mpu6050_read_word(0x47);
}
