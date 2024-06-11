#include "stm32f10x.h"
#include <stdio.h>
#include <math.h>
#include <stdint.h>

// dia chi I2C cua MPU6050
#define MPU6050_ADDR 0xD0 //0x68<<1 

// dia chi cua i2c ket noi lcd
#define LCD_ADDR 0x27 // Ðia chi I2C cua LCD 

// cac tham so dung cho tao tre
#define SYSTICK_LOAD (SystemCoreClock/1000000U)
#define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)

//
// cac thanh ghi cua MPU6050
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_INT_STATUS 0x3A
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40

//gia tri gia toc theo 3 phuong
volatile float a_x, a_y, a_z;

//*******//

volatile float Threshold = (1.58*9.81); // NGUONG PHAT HIEN NGA

//*******//

//bien trang thai cua he thong
volatile uint8_t system_on = 1;

//khai bao ham
void SysClkConf_72MHz(void) ;
void enter_sleep_mode(void);
float calculateAccelMagnitude(float ax, float ay, float az);
int checkFall(float ax, float ay, float az, float threshold);
void I2C_Init(void);
void MPU_WriteReg(uint8_t reg, uint8_t data);
uint8_t MPU_ReadReg(uint8_t reg);
void MPU6050_Init(void);
void MPU6050_ReadAccelRaw(int16_t* ax, int16_t* ay, int16_t* az);
void LED_Init(void);
void MPU6050_ReadAccel(float* ax, float* ay, float* az);
void I2C_Write(uint8_t data);
void LCD_Write(uint8_t address, uint8_t *data, int size);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(char *str);
void lcd_init(void);
void lcd_set_cursor(uint8_t row, uint8_t col);
void lcd_clear(void);
void EXTI_Config(void);
void delayUs(uint32_t us);
void delayMs(uint32_t ms);
//ham cau hinh xung he thong
void SysClkConf_72MHz(void) {
    //1. Turn on HSE to use.
    RCC->CR |= RCC_CR_HSEON; // HSE on
    while((RCC->CR & RCC_CR_HSERDY) == 0); // wait HSERDY.

    //2. config PLL (HSE, MUL9).
    RCC->CFGR |= RCC_CFGR_PLLMULL9; // PLLMUL9 -> systemclock = 72MHz
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6; // ADC prescale 6.
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; //APB1 prescale 2.

    //3. choose new clock source.
    RCC->CFGR |= RCC_CFGR_PLLSRC; // PLLSRC HSE

    //4. enable PLL
    RCC->CR |= RCC_CR_PLLON;
    while((RCC->CR & RCC_CR_PLLRDY) == 0); // wait PLLRDY.

    //5. switch sysclk to PLL
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) == 0); //wait SWS.

    //6. turn off original source
    RCC->CR &= ~(RCC_CR_HSION); // off HSION
    while((RCC->CR & RCC_CR_HSIRDY) == RCC_CR_HSIRDY);
}

//ham delay
void delayUs(uint32_t us){
		uint32_t i;
		for(i=0;i<us;i++){
			SysTick->LOAD = 9-1;
			SysTick->VAL = 0;
			SysTick->CTRL |= SysTick_CTRL_ENABLE;
			while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG));
			SysTick->CTRL &= ~SysTick_CTRL_ENABLE;
	}
}
void delayMs(uint32_t ms){
    for(uint32_t i = 0; i < ms; i++) {
        delayUs(1000); // Delay 1ms b?ng cách g?i hàm delayUs
    }
}
//ham vao che do ngu
void enter_sleep_mode(void) {
    // cap xung cho pwr
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    // xoa co pdds de vao Stop mode 
    PWR->CR &= ~PWR_CR_PDDS;

    // dat lpds de vao che do low-power deepsleep
    PWR->CR |= PWR_CR_LPDS;

    // dat che do sleepdeep
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // xoa WFI/WFE de chuan bi ngu dung cac ham de hoan thanh cac lenh truoc
    __DSB();
    __ISB();

    // vao che do ngu
    __WFI();
}

//ham tinh toan do lon gia toc theo 3 phuong
float calculateAccelMagnitude(float ax, float ay, float az) {
    return sqrt(ax * ax + ay * ay + az * az);
}

//ham kiem tra nga
int checkFall(float ax, float ay, float az, float threshold) {
    float accelMag = calculateAccelMagnitude(ax, ay, az);
    return (accelMag > threshold) ? 1 : 0;
}

//cau hinh i2c
void I2C_Init(void) {
    // cap xung cho I2C va GPIOB
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

		// cau hinh PB6 va PB7 cho I2C1 (scl va sda)
    // output mode speed 2Mhz, push - pull
    GPIOB->CRL &= ~(GPIO_CRL_MODE6 | GPIO_CRL_MODE7);
    GPIOB->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_MODE7_1;
    GPIOB->CRL |= GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_1;
		GPIOB->ODR |= ((1<<6 | 1<<7));  // Pull up scl va sda 
		
    // reset I2C1
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    // cau hinh I2C1
    I2C1->CR2 |= 36; // tan so PCLK1
    I2C1->CCR = 180; // che do tieu chuan, 100kHz
    I2C1->TRISE = 37; //rise time toi da

    // bat I2C1
    I2C1->CR1 |= I2C_CR1_PE;
}

//ham ghi vao thanh ghi qua i2c
void MPU_WriteReg(uint8_t reg, uint8_t data) {
    // khoi dong i2c
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB)); // doi den khi i2c duoc khoi dong

    // gui dia chi muc tieu(mpu6050) 
    I2C1->DR = MPU6050_ADDR;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;

    // gui dia chi cua thanh ghi muon ghi 
    I2C1->DR = reg;
    while (!(I2C1->SR1 & I2C_SR1_TXE));

    // ghi du lieu vao thanh ghi reg
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data; // Send data
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    // ket thuc i2c
    I2C1->CR1 |= I2C_CR1_STOP;
}

//ham doc tu thanh ghi qua i2c
uint8_t MPU_ReadReg(uint8_t reg) {
    uint8_t data;
    // bat dau i2c
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));

    // gui dia chi cua mpu6050
    I2C1->DR = MPU6050_ADDR;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;

    // gui dia chi thanh ghi muon doc
    I2C1->DR = reg;
    while (!(I2C1->SR1 & I2C_SR1_TXE));

    // khoi dong lai i2c de chuan bi cho viec doc
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));

    // doc du lieu dia chi MPU6050_ADDR va set bit lsb = 1 de cho biet la muon nhan du lieu
    I2C1->DR = MPU6050_ADDR | 0x01;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2; // xoa thanh ghi sr1, sr2 sau khi ghi hoac doc

    // xoa ack va gui tin hieu stop roi doc du lieu
    I2C1->CR1 &= ~I2C_CR1_ACK;
    I2C1->CR1 |= I2C_CR1_STOP;
    while (!(I2C1->SR1 & I2C_SR1_RXNE)); // cho tin hieu interrupt
    data = I2C1->DR;

    return data;
}

//ham cau hinh mpu
void MPU6050_Init(void) {
    MPU_WriteReg(MPU6050_PWR_MGMT_1, 0x00); // wake up MPU6050
    MPU_WriteReg(MPU6050_CONFIG, 0x00); // tat FSYNC, dai 260Hz
    MPU_WriteReg(MPU6050_ACCEL_CONFIG, 0x00); // dai cua cam bien gia toc ±2g
    MPU_WriteReg(MPU6050_INT_ENABLE, 0x01); // bat ngat thong bao khi du lieu san sang
    MPU_WriteReg(MPU6050_INT_PIN_CFG, 0x10); // xoa cac bit int status khi doc
}

//ham doc du lieu tu mpu (du lieu tho)
void MPU6050_ReadAccelRaw(int16_t* ax, int16_t* ay, int16_t* az) {
    // du lieu gia toc duoc luu tai 2 thanh ghi high và low dai 8 bit
    *ax = ((int16_t)MPU_ReadReg(MPU6050_ACCEL_XOUT_H) << 8) | MPU_ReadReg(MPU6050_ACCEL_XOUT_L);
    *ay = ((int16_t)MPU_ReadReg(MPU6050_ACCEL_YOUT_H) << 8) | MPU_ReadReg(MPU6050_ACCEL_YOUT_L);
    *az = ((int16_t)MPU_ReadReg(MPU6050_ACCEL_ZOUT_H) << 8) | MPU_ReadReg(MPU6050_ACCEL_ZOUT_L);
}

//ham doi du lieu doc tu mpu sang so float
void MPU6050_ReadAccel(float* ax, float* ay, float* az) {
    int16_t raw_ax, raw_ay, raw_az;
    MPU6050_ReadAccelRaw(&raw_ax, &raw_ay, &raw_az);
    // doi sang so float (don vi m/s^2)
    *ax = (raw_ax / 16384.0f)*9.81;
    *ay = (raw_ay / 16384.0f)*9.81;
    *az = (raw_az / 16384.0f)*9.81;
}
void I2C_Write(uint8_t data){
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
}

void LCD_Write(uint8_t address, uint8_t *data, int size){
    I2C1->CR1 |= I2C_CR1_ACK | I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    uint8_t addr = (address << 1) ;
    I2C1->DR = addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void) I2C1->SR1;
    (void) I2C1->SR2;
    for (uint8_t i = 0; i < size; i++) {
        I2C_Write(data[i]);  
    }
    I2C1->CR1 |= I2C_CR1_STOP;
}

void lcd_send_cmd(char cmd){
    char data_u = cmd & 0xF0;
    char data_l = (cmd << 4) & 0xF0;
    uint8_t data_t[4] = {
        data_u | 0x0C, data_u | 0x08,
        data_l | 0x0C, data_l | 0x08
    };
    LCD_Write(LCD_ADDR, data_t, 4);
}
void lcd_send_data(char data){
    char data_u = data & 0xF0;
    char data_l = (data << 4) & 0xF0;
    uint8_t data_t[4] = {
        data_u | 0x0D, data_u | 0x09,
        data_l | 0x0D, data_l | 0x09
    };
    LCD_Write(LCD_ADDR, data_t, 4);
}
void lcd_send_string(char *str){
    while (*str){
			lcd_send_data(*str++);
		}
}
// Hàm kh?i t?o LCD
void lcd_init(void) {
		delayMs(50);
    lcd_send_cmd(0x30);
    delayMs(5);
    lcd_send_cmd(0x30);
    delayUs(150);
    lcd_send_cmd(0x30);
    delayMs(10);
    lcd_send_cmd(0x20);

    lcd_send_cmd(0x28);
    delayMs(1);
    lcd_send_cmd(0x08);
    delayMs(1);
    lcd_send_cmd(0x01);
    delayMs(1);
    lcd_send_cmd(0x06);
    delayMs(1);
    lcd_send_cmd(0x0C);
}
// Hàm d? thi?t l?p v? trí con tr? trên LCD
void lcd_set_cursor(uint8_t row, uint8_t col) {
		uint8_t addr = (row == 0) ? 0x80 + col : 0xC0 + col;
    lcd_send_cmd(addr);
}

void lcd_clear(void) {
    lcd_send_cmd(0x01);  // Send clear display command
    delayMs(2);  // Wait for command to execute
}

void EXTI_Config(void) {
    // cap xung cho afio và gpioa de su dung lam ngat cong tac
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    // input floating cho interrupt tu mpu6050 
    GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);
    GPIOA->CRL |= GPIO_CRL_CNF0_1;
    GPIOA->ODR |= 1<<0;

    // pullup / pulldown input cho cong tac tat mo he thong
    GPIOA->CRL &= ~(GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
    GPIOA->CRL |= GPIO_CRL_CNF1_1;
		GPIOA->ODR |= 1<<1;
		
    // cau hình afio và exti
    AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PA | AFIO_EXTICR1_EXTI1_PA; // bat 2 chan pa0 va pa1 voi nhiem vu ngat ngoai
    EXTI->PR |= EXTI_PR_PR0 | EXTI_PR_PR1; // xoa pending 
    EXTI->FTSR |= EXTI_FTSR_TR0 | EXTI_FTSR_TR1; // chon suon xuong

    // xoa suon len    
    EXTI->RTSR &= ~(EXTI_RTSR_TR0);
    EXTI->RTSR &= ~(EXTI_RTSR_TR1);

    // chon interrupt xoa event
    EXTI->IMR |= EXTI_IMR_MR0 | EXTI_IMR_MR1;
    EXTI->EMR &= ~(EXTI_EMR_MR0);
    EXTI->EMR &= ~(EXTI_EMR_MR1);

    // set muc do uu tien
    NVIC_SetPriority(EXTI0_IRQn, 1);
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_SetPriority(EXTI1_IRQn, 0);
    NVIC_EnableIRQ(EXTI1_IRQn);
}

void EXTI1_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR1) {
        EXTI->PR |= EXTI_PR_PR1;
        system_on = !system_on; // dao nguoc che do
    }
}
void lcd_Writedata(void){
		char buffer[8];
		sprintf(buffer, "%.2f", a_x);
		lcd_set_cursor(0,0);
		lcd_send_string(buffer);	
		delayMs(50);
		sprintf(buffer, "%.2f", a_y);
		lcd_set_cursor(0, 6);
		lcd_send_string(buffer);
		delayMs(50);
		sprintf(buffer, "%.2f", a_z);		
		lcd_set_cursor(1, 0);
		lcd_send_string(buffer);
		delayMs(50);
}
void EXTI0_IRQHandler(void) {
		
    if (EXTI->PR & EXTI_PR_PR0) {
        EXTI->PR |= EXTI_PR_PR0;
        if (system_on) {
            MPU6050_ReadAccel(&a_x, &a_y, &a_z);
            if (checkFall(a_x, a_y, a_z, Threshold)) {
                GPIOC->ODR ^= (1 << 13);
								delayMs(100);
            } else {
                GPIOC->BSRR |= (1 << 13); // tat den
            }
						lcd_Writedata();
        }
    }
}

void LED_Init(void) {
		//led pc13 general output push pull
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
    GPIOC->CRH |= GPIO_CRH_MODE13;
    GPIOC->BSRR |= (1 << 13); // tat den
}

int main(void) {
		SysClkConf_72MHz();
    LED_Init();
    I2C_Init();
    MPU6050_Init();
    EXTI_Config();
    lcd_init(); // Khoi tao LCD
    while (1) {
        if (!system_on) {
            EXTI->IMR &= ~EXTI_IMR_MR0; // tat ngat EXTI0 
            enter_sleep_mode();
						SysClkConf_72MHz();
            EXTI->IMR |= EXTI_IMR_MR0; // bat lai ngat EXTI0 
        } else{
        }
    }
		
}
