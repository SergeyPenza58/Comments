
#include <math.h>
#include "mpu6050.h"

#define RAD2DEG 57.295779513082320876798154814105       // coefficient to convert radiant to degree
#define WHOAMI 0x75                                     // mpu6050 address of register "WHO AM I"
#define PWR_MGMT 0x6B                                   // mpu6050 address of register "Power management"
#define SMPLRT_DIV 0x19                                 // mpu6050 address of register "Sample Rate Divider"
#define ACCEL_CONFIG 0x1C                               // mpu6050 address of register "Accelerometer Configuration"
#define ACCEL_XOUT_H 0x3B                               // mpu6050 address of register "Accelerometer Measurements High byte"
#define TEMP_OUT_H 0x41                                 // mpu6050 address of register "Temperature Measurement High byte"
#define GYRO_CONFIG 0x1B                                // mpu6050 address of register "– Gyroscope Configuration"
#define GYRO_XOUT_H 0x43                                // mpu6050 address of register "Gyroscope Measurements High byte"
#define MPU6050_ADDR 0xD0                               // I2c address of sensor mpu6050
const uint16_t I2C_TIMEOUT = 100; 						// MAX timeout for operation read/write on I2C interface
const double Acc_Z_corrector = 14418.0; 				// Correct coefficient for Accelerometer to Convert received value
const double Acc_sensitivity = 16384.0; 				// Coefficient sensitivity for Accelerometer to Convert received value
const double Gyro_sensitivity = 131.0;					// Coefficient sensitivity for Gyroscope to Convert received value

uint32_t timer;

/*
 * Initialization of filter Y axle with default value
 */
Filter_t FilterX =
{ .Q_ANGLE = 0.001f, .Q_BIAS = 0.003f, .R_MEASURE = 0.03f };

/*
 * Initialization of filter Y axle with default value
 */
Filter_t FilterY =
{ .Q_ANGLE = 0.001f, .Q_BIAS = 0.003f, .R_MEASURE = 0.03f, };

/* @brief Performs initialization of the MPU6050 connected via i2c
 *
 * @param I2C_HandleTypeDef* I2Cx - pointer to initialized structure @I2C_HandleTypeDef
 *                                  who is responsible for the exchange from I2C with MPU6050
 *
 *
 * @retval If еру MPU6050 initialized successfully was returned "0". Else "1"
 */
uint8_t MPU_Init(I2C_HandleTypeDef *I2Cx)
{
	uint8_t check;
	uint8_t Data;

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHOAMI, 1, &check, 1, I2C_TIMEOUT); // check the device for identity

	if (check == 104)             // 104 (0x68) is default value in this register
	{
		Data = 0; // Internal oscillator, enable device,not sleep mode, no cycle mode, no device reset
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT, 1, &Data, 1,
				I2C_TIMEOUT);      // Transmit data to register "Power managment"

		Data = 0x07; // Set new frequency 1 kHz = 8 kHz / (1 + 0x07) (8kHz is Gyroscope Output Rate)
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV, 1, &Data, 1,
				I2C_TIMEOUT);  // Transmit data to register "Sample Rate Divider"

		Data = 0x00;          // Set Scale Range +/- 2g. Self test not activated
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1,
				I2C_TIMEOUT); //Transmit data to register "Accelerometer Configuration"

		Data = 0x00;         // Set Scale Range 250 °/s. Self test not activated
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1,
				I2C_TIMEOUT); //Transmit data to register "Gyroscope Configuration"
		return 0;
	}
	return 1;
}

/* @brief Read the most recent accelerometer measurements
 *
 * @param I2C_HandleTypeDef* I2Cx - pointer to initialized structure @I2C_HandleTypeDef
 *                                  who is responsible for the exchange from I2C with MPU6050
 *
 * @param MPU6050_t* DataStruct - pointer to struct @MPU6050_t.
 *
 * @retval none. Value will be update in param @DataStruct:
 *						- @Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW - received data bytes
 *						- @Ax, @Ay, @Az - most recent accelerometer measurements
 */
void Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
	uint8_t Rec_Data[6];                                  // array for read data

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 6,
			I2C_TIMEOUT); // Read 6 byte the most recent accelerometer measurements.

	// Let's analyze the received data
	// Save code value in user var

	DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]); // Union received bytes in a code. Most recent X axis accelerometer measurement.
	DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]); // Union received bytes in a code. Most recent Y axis accelerometer measurement.
	DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]); // Union received bytes in a code. Most recent Z axis accelerometer measurement.

	// Convert received code value with coefficient sensitivity
	// Save value in user var
	DataStruct->Ax = DataStruct->Accel_X_RAW / Acc_sensitivity;
	DataStruct->Ay = DataStruct->Accel_Y_RAW / Acc_sensitivity;
	DataStruct->Az = DataStruct->Accel_Z_RAW / Acc_Z_corrector;
}

/* @brief Read the most recent Gyroscope measurements
 *
 * @param I2C_HandleTypeDef* I2Cx - pointer to initialized structure @I2C_HandleTypeDef
 *                                  who is responsible for the exchange from I2C with MPU6050
 *
 * @param MPU6050_t* DataStruct - pointer to struct @MPU6050_t.
 *
 * @retval none. Value will be update in param @DataStruct:
 *						- @Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW - received data bytes
 *						- @Gx, @Gy, @Gz - most recent Gyroscope measurements
 */
void Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
	uint8_t Rec_Data[6];                                  // array for read data

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 6,
			I2C_TIMEOUT); // Read 6 byte the most recent Gyroscope Measurements.

	// Let's analyze the received data
	// Save code value in user var

	DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]); // Union received bytes in a code. Most recent X axis Gyroscope measurement.
	DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]); // Union received bytes in a code. Most recent Y axis Gyroscope measurement.
	DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]); // Union received bytes in a code. Most recent Z axis Gyroscope measurement.

	// Convert received code value with coefficient sensitivity
	// Save value in user var
	DataStruct->Gx = DataStruct->Gyro_X_RAW / Gyro_sensitivity;
	DataStruct->Gy = DataStruct->Gyro_Y_RAW / Gyro_sensitivity;
	DataStruct->Gz = DataStruct->Gyro_Z_RAW / Gyro_sensitivity;
}

/* @brief Read the most recent Temperature Measurement
 *
 *
 * @param I2C_HandleTypeDef* I2Cx - pointer to initialized structure @I2C_HandleTypeDef
 *                                  who is responsible for the exchange from I2C with MPU6050
 *
 * @param MPU6050_t* DataStruct - pointer to struct @MPU6050_t.
 *
 * @retval none. Value will be update in param @DataStruct: @Temperature - most recent Temperature measurements
 */
void Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
	uint8_t Rec_Data[2];
	int16_t temp;

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H, 1, Rec_Data, 2,
			I2C_TIMEOUT); // Read 2 byte the most recent Temperature Measurements.

	temp = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);// Union received bytes in a code.
	//See section 4.18 where define coefficient in formula
	DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0
			+ (float) 36.53);// Convert received code to the temperature in degrees C.

}

/* @brief Read the most recent accelerometer,Gyroscope, Temperature Measurement
 *
 *
 * @param I2C_HandleTypeDef* I2Cx - pointer to initialized structure @I2C_HandleTypeDef
 *                                  who is responsible for the exchange from I2C with MPU6050
 *
 * @param MPU6050_t* DataStruct - pointer to struct @MPU6050_t.
 *
 * @retval none. Value will be update in param @DataStruct.
 */
void Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{

	Read_Accel(I2Cx, DataStruct); 	// Read accelerometer

	Read_Gyro(I2Cx, DataStruct); 	// Read Gyroscope

	Read_Temp(I2Cx, DataStruct);	// Read Temperature

	double dt = (double) (HAL_GetTick() - timer) / 1000; 	// calculate how much time has passed since the last poll (in tick)
	timer = HAL_GetTick();									// Save the current tick value

	//  Calculate the roll
	double roll;
	// calculate the value of the denominator
	double roll_sqrt = sqrt(
			DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW
					+ DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);

	// check that the value is not null. Division by zero is not allowed
	if (roll_sqrt != 0.0)
	{
		// Calculate the user is
		roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD2DEG;
	}
	else
	{
		roll = 0.0;
	}

	//  Calculate the pitch
	double pitch = atan2(-DataStruct->Accel_X_RAW,
			DataStruct->Accel_Z_RAW) * RAD2DEG;

	if ((pitch < -90 && DataStruct->FilterAngleY > 90)
			|| (pitch > 90 && DataStruct->FilterAngleY < -90))
	{
		FilterY.angle = pitch;
		DataStruct->FilterAngleY = pitch;
	}
	else
	{
		DataStruct->FilterAngleY = Filter_getAngle(&FilterY, pitch,
				DataStruct->Gy, dt);
	}

	if (fabs(DataStruct->FilterAngleY) > 90)
		DataStruct->Gx = -DataStruct->Gx;

	DataStruct->FilterAngleX = Filter_getAngle(&FilterX, roll, DataStruct->Gx,
			dt);
}

/* @brief Performs filtering of the measured angle value
 *
 *
 * @param Filter_t* Filter - pointer to struct @Filter_t
 *
 * @param double newAngle - new value of angle roll
 *
 * @param double newRate - new value of angle roll
 *
 * @param double dt - time tick from last poll
 *
 * @retval double filter value of Angle
 */
double Filter_getAngle(Filter_t *Filter, double newAngle, double newRate,
		double dt)
{
	double rate = newRate - Filter->bias;
	Filter->angle += dt * rate;

	Filter->P[0][0] += dt
			* (dt * Filter->P[1][1] - Filter->P[0][1] - Filter->P[1][0]
					+ Filter->Q_ANGLE);
	Filter->P[0][1] -= dt * Filter->P[1][1];
	Filter->P[1][0] -= dt * Filter->P[1][1];
	Filter->P[1][1] += Filter->Q_BIAS * dt;

	double S = Filter->P[0][0] + Filter->R_MEASURE;
	double K[2];
	K[0] = Filter->P[0][0] / S;
	K[1] = Filter->P[1][0] / S;

	double y = newAngle - Filter->angle;
	Filter->angle += K[0] * y;
	Filter->bias += K[1] * y;

	double P00_temp = Filter->P[0][0];
	double P01_temp = Filter->P[0][1];

	Filter->P[0][0] -= K[0] * P00_temp;
	Filter->P[0][1] -= K[0] * P01_temp;
	Filter->P[1][0] -= K[1] * P00_temp;
	Filter->P[1][1] -= K[1] * P01_temp;

	return Filter->angle;
}
;
