#ifndef BMI088DRIVER_H
#define BMI088DRIVER_H

#include "struct_typedef.h"

#pragma pack(push, 1)

typedef struct BMI088_RAW_DATA
{
    uint8_t status;
    int16_t accel[3];
    int16_t temp;
    int16_t gyro[3];
} bmi088_raw_data_t;

#pragma pack(pop)

typedef struct BMI088_REAL_DATA
{
    uint8_t status;
    float 	accel[3];
    float 	gyro[3];
    float 	time;
    float 	temp;
} bmi088_real_data_t;

#define BMI088_GYRO_DATA_READY_BIT 0
#define BMI088_ACCEL_DATA_READY_BIT 1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2


#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)

#define BMI088_ACCEL_RANGE_3G
//#define BMI088_ACCEL_RANGE_6G
//#define BMI088_ACCEL_RANGE_12G
//#define BMI088_ACCEL_RANGE_24G

#define BMI088_GYRO_RANGE_2000
//#define BMI088_GYRO_RANGE_1000
//#define BMI088_GYRO_RANGE_500
//#define BMI088_GYRO_RANGE_250
//#define BMI088_GYRO_RANGE_125


#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f


#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f


enum
{
    BMI088_NO_Error = 0x00,
    BMI088_ACC_PWR_CTRL_Error = 0x01,
    BMI088_ACC_PWR_CONF_Error = 0x02,
    BMI088_ACC_CONF_Error = 0x03,
    BMI088_ACC_SELF_TEST_Error = 0x04,
    BMI088_ACC_RANGE_Error = 0x05,
    BMI088_INT1_IO_CTRL_Error = 0x06,
    BMI088_INT_MAP_DATA_Error = 0x07,
    BMI088_GYRO_RANGE_Error = 0x08,
    BMI088_GYRO_BANDWIDTH_Error = 0x09,
    BMI088_GYRO_LPM1_Error = 0x0A,
    BMI088_GYRO_CTRL_Error = 0x0B,
    BMI088_GYRO_INT3_INT4_IO_CONF_Error = 0x0C,
    BMI088_GYRO_INT3_INT4_IO_MAP_Error = 0x0D,

    BMI088_SELF_TEST_ACCEL_Error = 0x80,
    BMI088_SELF_TEST_GYRO_Error = 0x40,
    BMI088_NO_Sensor = 0xFF,
};

#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME 150

extern uint8_t bsp_bmi088_init(void);
extern bool_t bmi088_accel_self_test(void);
extern bool_t bmi088_gyro_self_test(void);
extern bool_t bmi088_accel_init(void);
extern bool_t bmi088_gyro_init(void);

extern void bsp_bmi088_accel_read(uint8_t *rx_buf, float accel[3], float *time);
extern void bsp_bmi088_gyro_read(uint8_t *rx_buf, float gyro[3]);
extern void bsp_bmi088_temperature_read(uint8_t *rx_buf, float *temperate);
extern void bsp_bmi088_read(float gyro[3], float accel[3], float *temperate);
extern uint32_t bsp_bmi088_sensor_time(void);
extern float bsp_bmi088_get_temperate(void);
extern void bsp_bmi088_get_gyro(int16_t gyro[3]);
extern void bsp_bmi088_get_accel(float accel[3]);

extern void bsp_bmi088_read_gyro_who_am_i(void);
extern void bsp_bmi088_read_accel_who_am_i(void);

#endif
