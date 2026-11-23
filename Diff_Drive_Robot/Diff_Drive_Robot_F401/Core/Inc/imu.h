#ifndef IMU_H
#define IMU_H

#include <stdint.h>

/*
 * NCS/SC		PB12
 * SCL/SCK		PB13
 * AD0/SO0/MISO	PB14
 * SDA/SDI/MOSI	PB15
 */

/* Structure for raw and converted sensor data from MPU9250 */
typedef struct {
    // Raw 16-bit ADC values
    int16_t ax, ay, az;     // Accelerometer (X, Y, Z)
    int16_t temp;           // Temperature
    int16_t gx, gy, gz;     // Gyroscope (X, Y, Z)

    // Converted physical values
    float accel_x, accel_y, accel_z; // Acceleration (g)
    float temperature;               // Temperature (°C)
    float gyro_x, gyro_y, gyro_z;    // Angular velocity (°/s)
} MPU_t;

/* Initialize SPI2 and MPU9250 (Returns 1 on success, 0 on error) */
uint8_t MPU_Init(void);

/* Read raw 16-bit ADC sensor data */
void MPU_ReadRaw(MPU_t *raw);

/* Read raw data and convert to readable engineering units (g, °C, °/s) */
void MPU_ReadData(MPU_t *raw);

#endif /* IMU_H */
