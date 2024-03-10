/*
 * MPUXX50.c
 *
 *  Created on: Feb 27, 2022
 *      Author: MarkSherstan
 *
 *  Editied on: MAr 10, 2024
 *      Filename: MPUXX50 --> MPU6050
 *      by: Mounith H
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_ 1

// Libs
#include "stm32f1xx_hal_i2c.h"
#include <stdint.h>
#include <stdbool.h>
#include "math.h"

// Constants
#define RAD2DEG 57.2957795131

// Defines
#define WHO_AM_I_6050_ANS 0x68
#define WHO_AM_I_9250_ANS 0x71
#define WHO_AM_I          0x75
#define AD0_LOW           0x68
#define AD0_HIGH          0x69
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define PWR_MGMT_1        0x6B
#define ACCEL_XOUT_H      0x3B
#define I2C_TIMOUT_MS     1000

#define CALIB_OFFSET_NB_MES   500



// Full scale ranges
enum gyroscopeFullScaleRange
{
    GFSR_250DPS,
    GFSR_500DPS,
    GFSR_1000DPS,
    GFSR_2000DPS
};

enum accelerometerFullScaleRange
{
    AFSR_2G,
    AFSR_4G,
    AFSR_8G,
    AFSR_16G
};

// Structures
struct RawData
{
    int16_t ax, ay, az, gx, gy, gz;
} rawData;

struct SensorData
{
    float ax, ay, az, gx, gy, gz;
} sensorData;

struct GyroCal
{
    float x, y, z;
} gyroCal;

struct Attitude
{
    float r, p, y;
} attitude;

// Functions
uint8_t MPU_begin(I2C_HandleTypeDef *I2Cx, uint8_t addr, uint8_t aScale, uint8_t gScale, float tau, float dt);

void MPU_calibrateGyro(I2C_HandleTypeDef *I2Cx, uint16_t numCalPoints);
void MPU_calcAttitude(I2C_HandleTypeDef *I2Cx);
void MPU_readRawData(I2C_HandleTypeDef *I2Cx);
void MPU_readProcessedData(I2C_HandleTypeDef *I2Cx);
void MPU_writeGyroFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t gScale);
void MPU_writeAccFullScaleRange(I2C_HandleTypeDef *I2Cx, uint8_t aScale);
void setGyroOffsets(float x, float y, float z);
void calcOffsets(bool is_calc_gyro, bool is_calc_acc);
void MPU_readProcessedDataLPF(I2C_HandleTypeDef *I2Cx);



#endif /* INC_MPU6050_H_ */
