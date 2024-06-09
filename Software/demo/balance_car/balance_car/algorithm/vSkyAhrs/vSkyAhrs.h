
#ifndef VSKY_AHRS
#define VSKY_AHRS


#include "stdint.h"
#include "stdio.h"


#define VSKY_AHRS_CALI_READY_US     50*000   	//丢弃刚启动时的不稳定数据 60ms
#define VSKY_AHRS_CALI_END_US       400*1000  	//校准时长 0.4s
#define VSKY_AHRS_CALI_READY_OK_US	50*0000		//丢弃刚启动校准结果时的不稳定数据

#define MadgwickAHRS_USER  0

#ifndef MadgwickAHRS_USER
#define MahonyAHRS_USER    1
#endif

typedef enum
{
	SKY_IMU_CALI_NULL = 0,
    SKY_IMU_CALI_READY,
	SKY_IMU_CALI_GYRO_START,
	SKY_IMU_CALI_GYRO_WHILE,
	SKY_IMU_CALI_GYRO_END,
	SKY_IMU_CALI_READY_OK,
	SKY_IMU_CALI_OK,
} SKY_IMU_CALI_ENUM;

typedef struct
{
	uint16_t imu_cali;
	float    vAaccel_data[3];//m/s^2
	float    vGyro_data[3];//弧度/s
	float    vMag_data[3];//
	float    vAngle_data[3];//度

    float    vGyro_cali[3];

    float    vAccel_offset[3];
    float    vMag_offset[3];
    float    vGyro_offset[3];
	float    vRtemp_data;
    float    vCail_gyro;
	float    vCycle_s;//周期，单位s
	uint64_t vCali_run_count;
	uint64_t vCali_res_count;

	uint64_t vImu_time_last_us;
	uint64_t vImu_time_us;
    uint64_t vMag_time_last_us;
	uint64_t vMag_time_us; 

    uint64_t vFirst_time_us;
} vsky_ahrs_info;


//旋转矩阵
#define BOARD_INSTALL_SPIN_MATRIX \
	{0.0f, 1.0f, 0.0f},           \
		{-1.0f, 0.0f, 0.0f},      \
	{                             \
		0.0f, 0.0f, 1.0f          \
	}


void vsky_imu_offset_config(vsky_ahrs_info *ahrs_info, float *acc_offset, float *gyro_offset, float *mag_offset);
void vsky_imu_updata(vsky_ahrs_info *ahrs_info, float *acc, float *gyro, float *mag, uint64_t timer_us);


#endif
