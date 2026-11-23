#include "imu.h"
#include "stm32f4xx.h"

/* Chip Select PB12 macros */
#define CS_LOW()   (GPIOB->BSRR = (1U << (12 + 16)))
#define CS_HIGH()  (GPIOB->BSRR = (1U << 12))

static void delay(volatile uint32_t count) {
    while (count--) { __NOP(); }
}

/* Transmit and receive one byte via SPI2 */
static uint8_t SPI_Transfer(uint8_t data) {
    while (!(SPI2->SR & SPI_SR_TXE));             // Wait until transmit buffer is empty
    *((__IO uint8_t *)&SPI2->DR) = data;          // Send byte
    while (!(SPI2->SR & SPI_SR_RXNE));            // Wait until receive buffer has data
    return *((__IO uint8_t *)&SPI2->DR);          // Return received byte
}

/* Write one byte to an MPU register */
static void MPU_Write(uint8_t reg, uint8_t val) {
    CS_LOW();
    SPI_Transfer(reg & 0x7F);                    // Bit 7 = 0 for Write
    SPI_Transfer(val);
    CS_HIGH();
}

/* Burst read multiple bytes from an MPU register */
static void MPU_Read(uint8_t reg, uint8_t *buf, uint8_t len) {
    CS_LOW();
    SPI_Transfer(reg | 0x80);                    // Bit 7 = 1 for Read
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = SPI_Transfer(0xFF);             // Send dummy byte to clock in data
    }
    CS_HIGH();
}

/* Configure GPIO and SPI2 peripheral registers */
static void SPI_GPIO_Init(void) {
    // 1. Enable GPIOB and SPI2 clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    // 2. PB12: CS (Output Push-Pull, High Speed)
    GPIOB->MODER   &= ~(3U << (12 * 2));
    GPIOB->MODER   |=  (1U << (12 * 2));
    GPIOB->OSPEEDR |=  (3U << (12 * 2));
    CS_HIGH();

    // 3. PB13 (SCK), PB14 (MISO), PB15 (MOSI): Alternate Function AF5, High Speed
    GPIOB->MODER   &= ~((3U << 26) | (3U << 28) | (3U << 30));
    GPIOB->MODER   |=  ((2U << 26) | (2U << 28) | (2U << 30));
    GPIOB->OSPEEDR |=  ((3U << 26) | (3U << 28) | (3U << 30));
    GPIOB->PUPDR   |=  (1U << 28); // Enable pull-up on MISO (PB14)

    GPIOB->AFR[1]  &= ~((0xFU << 20) | (0xFU << 24) | (0xFU << 28));
    GPIOB->AFR[1]  |=  ((5U << 20) | (5U << 24) | (5U << 28));

    // 4. SPI2: Master mode, Mode 3 (CPOL=1, CPHA=1), Prescaler /16, Software NSS
    SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_CPOL | SPI_CR1_CPHA |
                SPI_CR1_SSM  | SPI_CR1_SSI  | (3U << SPI_CR1_BR_Pos);
    SPI2->CR1 |= SPI_CR1_SPE;
}

/* ================= MPU9250 PUBLIC FUNCTIONS ================= */

uint8_t MPU_Init(void) {
    SPI_GPIO_Init();
    delay(50000);

    // 1. Device reset
    MPU_Write(0x6B, 0x80); // PWR_MGMT_1: Reset
    delay(100000);

    // 2. Wake up and select auto clock (PLL)
    MPU_Write(0x6B, 0x01); // PWR_MGMT_1: Auto clock
    delay(10000);

    // 3. Disable I2C slave to lock into SPI mode
    MPU_Write(0x6A, 0x10); // USER_CTRL: I2C_IF_DIS (0x10)

    // 4. Verify WHO_AM_I register (MPU9250=0x71, MPU9255=0x73, MPU6500=0x70)
    uint8_t who = 0;
    MPU_Read(0x75, &who, 1);
    if (who != 0x71 && who != 0x73 && who != 0x70) {
        return 0; // Communication failed
    }

    // 5. Configure DLPF and ranges
    MPU_Write(0x1A, 0x03); // CONFIG: DLPF = 41Hz
    MPU_Write(0x19, 0x04); // SMPLRT_DIV: Sample Rate = 200Hz
    MPU_Write(0x1B, 0x18); // GYRO_CONFIG: +/- 2000 dps
    MPU_Write(0x1C, 0x00); // ACCEL_CONFIG: +/- 2g
    MPU_Write(0x6C, 0x00); // PWR_MGMT_2: Enable all axes

    return 1;
}

void MPU_ReadRaw(MPU_t *raw) {
    uint8_t b[14];
    // Burst read 14 bytes starting from 0x3B (ACCEL_XOUT_H) to 0x48 (GYRO_ZOUT_L)
    MPU_Read(0x3B, b, 14);

    // Accelerometer (Big-Endian)
    raw->ax   = (int16_t)((b[0] << 8) | b[1]);
    raw->ay   = (int16_t)((b[2] << 8) | b[3]);
    raw->az   = (int16_t)((b[4] << 8) | b[5]);

    // Temperature (Big-Endian)
    raw->temp = (int16_t)((b[6] << 8) | b[7]);

    // Gyroscope (Big-Endian)
    raw->gx   = (int16_t)((b[8]  << 8) | b[9]);
    raw->gy   = (int16_t)((b[10] << 8) | b[11]);
    raw->gz   = (int16_t)((b[12] << 8) | b[13]);
}

void MPU_ReadData(MPU_t *raw) {
    // 1. Read raw 16-bit values from sensor
    MPU_ReadRaw(raw);

    // 2. Convert Accelerometer to g (+/-2g -> 16384 LSB/g)
    raw->accel_x = (float)raw->ax / 16384.0f;
    raw->accel_y = (float)raw->ay / 16384.0f;
    raw->accel_z = (float)raw->az / 16384.0f;

    // 3. Convert Temperature to degrees Celsius (333.87 LSB/°C + 21°C offset)
    raw->temperature = ((float)raw->temp) / 333.87f + 21.0f;

    // 4. Convert Gyroscope to deg/s (+/-2000 dps -> 16.4 LSB/(°/s))
    raw->gyro_x = (float)raw->gx / 16.4f;
    raw->gyro_y = (float)raw->gy / 16.4f;
    raw->gyro_z = (float)raw->gz / 16.4f;
}
