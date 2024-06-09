/*****************************(C) COPYRIGHT 2016 DJI****************************/
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"


float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_Write_ACCEL_Reg_Num 6
#define BMI088_Write_GYRO_Reg_Num 6

static void bsp_bmi088_write_single_reg(uint8_t reg, uint8_t data);
static void bsp_bmi088_read_single_reg(uint8_t reg, uint8_t *return_data);
static void bsp_bmi088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#define bsp_bmi088_accel_write_single_reg(reg, data) \
    {                                            \
        BSP_BMI088_ACCEL_NS_L();                     \
        bsp_bmi088_write_single_reg((reg), (data));  \
        BSP_BMI088_ACCEL_NS_H();                     \
    }
#define bsp_bmi088_accel_read_single_reg(reg, data) \
    {                                           \
        BSP_BMI088_ACCEL_NS_L();                    \
        bsp_bmi088_read_write_byte((reg) | 0x80);   \
        bsp_bmi088_read_write_byte(0x55);           \
        (data) = bsp_bmi088_read_write_byte(0x55);  \
        BSP_BMI088_ACCEL_NS_H();                    \
    }

#define bsp_bmi088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BSP_BMI088_ACCEL_NS_L();                       \
        bsp_bmi088_read_write_byte((reg) | 0x80);      \
        bsp_bmi088_read_muli_reg(reg, data, len);      \
        BSP_BMI088_ACCEL_NS_H();                       \
    }

#define bsp_bmi088_gyro_write_single_reg(reg, data) \
    {                                           \
        BSP_BMI088_GYRO_NS_L();                     \
        bsp_bmi088_write_single_reg((reg), (data)); \
        BSP_BMI088_GYRO_NS_H();                     \
    }
#define bsp_bmi088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BSP_BMI088_GYRO_NS_L();                     \
        bsp_bmi088_read_single_reg((reg), &(data)); \
        BSP_BMI088_GYRO_NS_H();                     \
    }

#define bsp_bmi088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BSP_BMI088_GYRO_NS_L();                         \
        bsp_bmi088_read_muli_reg((reg), (data), (len)); \
        BSP_BMI088_GYRO_NS_H();                         \
    }

static uint8_t Write_BMI088_ACCEL_Reg_Data_Error[BMI088_Write_ACCEL_Reg_Num][3] =
{
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_Error},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_Error},
    {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_Error},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_Error},
    {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_Error},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_Error}

};

static uint8_t Write_BMI088_GYRO_Reg_Data_Error[BMI088_Write_GYRO_Reg_Num][3] =
{
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_Error},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_Error},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_Error},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_Error},
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_Error},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_Error}

};

uint8_t bsp_bmi088_init(void)
{
    uint8_t error = BMI088_NO_Error;
    // GPIO and SPI  Init .
    bsp_bmi088_gpio_init();
    bsp_bmi088_com_init();

    // 自检通过并初始化
    if (bmi088_accel_self_test() != BMI088_NO_Error)
        {
            error |= BMI088_SELF_TEST_ACCEL_Error;
        }
    else
        {
            error |= bmi088_accel_init();
        }

    if (bmi088_gyro_self_test() != BMI088_NO_Error)
        {
            error |= BMI088_SELF_TEST_GYRO_Error;
        }
    else
        {
            error |= bmi088_gyro_init();
        }

    return error;
}

bool_t bmi088_accel_init(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    //检查通信
    bsp_bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    bsp_bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //accel软件复位
    bsp_bmi088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    bsp_bmi088_delay_ms(BMI088_LONG_DELAY_TIME);

    //重置后检查通信是否正常
    bsp_bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    bsp_bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // 检查 "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
        {
            return BMI088_NO_Sensor;
        }

    //设置accel sonsor配置并检查
    for (write_reg_num = 0; write_reg_num < BMI088_Write_ACCEL_Reg_Num; write_reg_num++)
        {

            bsp_bmi088_accel_write_single_reg(Write_BMI088_ACCEL_Reg_Data_Error[write_reg_num][0], Write_BMI088_ACCEL_Reg_Data_Error[write_reg_num][1]);
            bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

            bsp_bmi088_accel_read_single_reg(Write_BMI088_ACCEL_Reg_Data_Error[write_reg_num][0], res);
            bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

            if (res != Write_BMI088_ACCEL_Reg_Data_Error[write_reg_num][1])
                {
                    return Write_BMI088_ACCEL_Reg_Data_Error[write_reg_num][2];
                }
        }
    return BMI088_NO_Error;
}

bool_t bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    //检查通信
    bsp_bmi088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    bsp_bmi088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //重置陀螺仪传感器
    bsp_bmi088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    bsp_bmi088_delay_ms(BMI088_LONG_DELAY_TIME);
    //重置后检查通信是否正常
    bsp_bmi088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    bsp_bmi088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // 检查 "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
        {
            return BMI088_NO_Sensor;
        }

    //设置陀螺仪传感器配置并检查
    for (write_reg_num = 0; write_reg_num < BMI088_Write_GYRO_Reg_Num; write_reg_num++)
        {

            bsp_bmi088_gyro_write_single_reg(Write_BMI088_GYRO_Reg_Data_Error[write_reg_num][0], Write_BMI088_GYRO_Reg_Data_Error[write_reg_num][1]);
            bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

            bsp_bmi088_gyro_read_single_reg(Write_BMI088_GYRO_Reg_Data_Error[write_reg_num][0], res);
            bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

            if (res != Write_BMI088_GYRO_Reg_Data_Error[write_reg_num][1])
                {
                    return Write_BMI088_GYRO_Reg_Data_Error[write_reg_num][2];
                }
        }

    return BMI088_NO_Error;
}

bool_t bmi088_accel_self_test(void)
{

    int16_t self_test_accel[2][3];

    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    uint8_t res = 0;

    uint8_t write_reg_num = 0;

    static const uint8_t write_BMI088_ACCEL_self_test_Reg_Data_Error[6][3] =
    {
        {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_1600_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_Error},
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_Error},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_24G, BMI088_ACC_RANGE_Error},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_Error},
        {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL, BMI088_ACC_PWR_CONF_Error},
        {BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL, BMI088_ACC_PWR_CONF_Error}

    };

    //检查通信是否正常
    bsp_bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    bsp_bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // 重置bmi088加速度传感器并等待> 50毫秒
    bsp_bmi088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    bsp_bmi088_delay_ms(BMI088_LONG_DELAY_TIME);

    //检查通信是否正常
    bsp_bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    bsp_bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if(res != BMI088_ACC_CHIP_ID_VALUE)
        {
            return BMI088_NO_Sensor;
        }

    // 设置accel寄存器
    for (write_reg_num = 0; write_reg_num < 4; write_reg_num++)
        {

            bsp_bmi088_accel_write_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1]);
            bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

            bsp_bmi088_accel_read_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][0], res);
            bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

            if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][1])
                {
                    return write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num][2];
                }
            // 加速度和加速度范围。两个寄存器组需要等待> 50毫秒
            bsp_bmi088_delay_ms(BMI088_LONG_DELAY_TIME);
        }

    // 自我测试包括阳性和阴性
    for (write_reg_num = 0; write_reg_num < 2; write_reg_num++)
        {

            bsp_bmi088_accel_write_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1]);
            bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

            bsp_bmi088_accel_read_single_reg(write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][0], res);
            bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

            if (res != write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][1])
                {
                    return write_BMI088_ACCEL_self_test_Reg_Data_Error[write_reg_num + 4][2];
                }
            //加速度和加速度范围。两个寄存器组需要等待> 50毫秒
            bsp_bmi088_delay_ms(BMI088_LONG_DELAY_TIME);

            //读取响应加速度
            bsp_bmi088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

            self_test_accel[write_reg_num][0] = (int16_t)((buf[1]) << 8) | buf[0];
            self_test_accel[write_reg_num][1] = (int16_t)((buf[3]) << 8) | buf[2];
            self_test_accel[write_reg_num][2] = (int16_t)((buf[5]) << 8) | buf[4];
        }

    //关闭自检
    bsp_bmi088_accel_write_single_reg(BMI088_ACC_SELF_TEST, BMI088_ACC_SELF_TEST_OFF);
    bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    bsp_bmi088_accel_read_single_reg(BMI088_ACC_SELF_TEST, res);
    bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    if (res != (BMI088_ACC_SELF_TEST_OFF))
        {
            return BMI088_ACC_SELF_TEST_Error;
        }

    //重置加速度传感器
    bsp_bmi088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    bsp_bmi088_delay_ms(BMI088_LONG_DELAY_TIME);

    if ((self_test_accel[0][0] - self_test_accel[1][0] < 1365) || (self_test_accel[0][1] - self_test_accel[1][1] < 1365) || (self_test_accel[0][2] - self_test_accel[1][2] < 680))
        {
            return BMI088_SELF_TEST_ACCEL_Error;
        }

    bsp_bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    bsp_bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    return BMI088_NO_Error;
}
bool_t bmi088_gyro_self_test(void)
{
    uint8_t res = 0;
    uint8_t retry = 0;
    //检查通信是否正常
    bsp_bmi088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    bsp_bmi088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    //重置陀螺仪传感器
    bsp_bmi088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    bsp_bmi088_delay_ms(BMI088_LONG_DELAY_TIME);
    //重置后检查通信是否正常
    bsp_bmi088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    bsp_bmi088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    bsp_bmi088_gyro_write_single_reg(BMI088_GYRO_SELF_TEST, BMI088_GYRO_TRIG_BIST);
    bsp_bmi088_delay_ms(BMI088_LONG_DELAY_TIME);

    do
        {

            bsp_bmi088_gyro_read_single_reg(BMI088_GYRO_SELF_TEST, res);
            bsp_bmi088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
            retry++;
        }
    while (!(res & BMI088_GYRO_BIST_RDY) && retry < 10);

    if (retry == 10)
        {
            return BMI088_SELF_TEST_GYRO_Error;
        }

    if (res & BMI088_GYRO_BIST_FAIL)
        {
            return BMI088_SELF_TEST_GYRO_Error;
        }

    return BMI088_NO_Error;
}

void bsp_bmi088_read_gyro_who_am_i(void)
{
    uint8_t buf;
    bsp_bmi088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, buf);
}


void bsp_bmi088_read_accel_who_am_i(void)
{
    uint8_t buf;
    bsp_bmi088_accel_read_single_reg(BMI088_ACC_CHIP_ID,buf);
    buf = 0;
}

void bsp_bmi088_temperature_read(uint8_t *rx_buf, float *temperate)
{
    int16_t bmi088_raw_temp;
    bmi088_raw_temp = (int16_t)((rx_buf[0] << 3) | (rx_buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
        {
            bmi088_raw_temp -= 2048;
        }
    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

}

void bsp_bmi088_accel_read(uint8_t *rx_buf, float accel[3], float *time)
{
    int16_t bmi088_raw_temp;
    uint32_t sensor_time;
    bmi088_raw_temp = (int16_t)((rx_buf[1]) << 8) | rx_buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    sensor_time = (uint32_t)((rx_buf[8] << 16) | (rx_buf[7] << 8) | rx_buf[6]);
    *time = sensor_time * 39.0625f;
}

void bsp_bmi088_gyro_read(uint8_t *rx_buf, float gyro[3])
{
    int16_t bmi088_raw_temp;
    bmi088_raw_temp = (int16_t)((rx_buf[1]) << 8) | rx_buf[0];
    gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
    bmi088_raw_temp = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
    gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
    bmi088_raw_temp = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
    gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
}
int16_t gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;

void bsp_bmi088_read(float gyro[3], float accel[3], float *temperate)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;

    bsp_bmi088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    accel_x = bmi088_raw_temp;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    accel_y = bmi088_raw_temp;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    accel_z = bmi088_raw_temp;

    bsp_bmi088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
        {
            bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
            gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
            gyro_x = bmi088_raw_temp;
            bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
            gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
            gyro_y = bmi088_raw_temp;
            bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
            gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
            gyro_z = bmi088_raw_temp;
        }
    bsp_bmi088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
        {
            bmi088_raw_temp -= 2048;
        }

    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

uint32_t bsp_bmi088_sensor_time(void)
{
    uint32_t sensor_time = 0;
    uint8_t buf[3];
    bsp_bmi088_accel_read_muli_reg(BMI088_SENSORTIME_DATA_L, buf, 3);

    sensor_time = (uint32_t)((buf[2] << 16) | (buf[1] << 8) | (buf[0]));

    return sensor_time;
}

float bsp_bmi088_get_temperate(void)
{
    uint8_t buf[2];
    float temperate;
    int16_t temperate_raw_temp;

    bsp_bmi088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    temperate_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (temperate_raw_temp > 1023)
        {
            temperate_raw_temp -= 2048;
        }

    temperate = temperate_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;

    return temperate;
}

void bsp_bmi088_get_gyro(int16_t gyro[3])
{
    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    int16_t gyro_raw_temp;

    bsp_bmi088_gyro_read_muli_reg(BMI088_GYRO_X_L, buf, 6);

    gyro_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    gyro[0] = gyro_raw_temp ;
    gyro_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    gyro[1] = gyro_raw_temp ;
    gyro_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    gyro[2] = gyro_raw_temp ;
}

void bsp_bmi088_get_accel(float accel[3])
{
    uint8_t buf[6] = {0, 0, 0, 0, 0, 0};
    int16_t accel_raw_temp;

    bsp_bmi088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    accel_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = accel_raw_temp * BMI088_ACCEL_SEN;
    accel_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = accel_raw_temp * BMI088_ACCEL_SEN;
    accel_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = accel_raw_temp * BMI088_ACCEL_SEN;
}

#if defined(BMI088_USE_SPI)

static void bsp_bmi088_write_single_reg(uint8_t reg, uint8_t data)
{
    bsp_bmi088_read_write_byte(reg);
    bsp_bmi088_read_write_byte(data);
}
static void bsp_bmi088_read_single_reg(uint8_t reg, uint8_t *return_data)
{
    bsp_bmi088_read_write_byte(reg | 0x80);
    *return_data = bsp_bmi088_read_write_byte(0x55);
}
static void bsp_bmi088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    bsp_bmi088_read_write_byte(reg | 0x80);

    while (len != 0)
        {
            *buf = bsp_bmi088_read_write_byte(0x55);
            buf++;
            len--;
        }
}
#elif defined(BMI088_USE_IIC)


#endif
