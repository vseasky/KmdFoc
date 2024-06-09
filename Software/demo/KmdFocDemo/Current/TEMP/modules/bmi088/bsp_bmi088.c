#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include "ist8310driver.h"

#include "bsp_bmi088.h"
#include "bsp_spi.h"
#include "spi.h"
#include "vSkyAhrs.h"

#include "cmsis_os.h"
#include "pid.h"


#define  BMI088_TEMP_PWM_MAX 5000
#define  BMI088_TEMP_MAX     45.0f
extern TIM_HandleTypeDef htim10;


void bsp_bmi088_drive_init(void);
void bsp_bmi088_read_updata(void);
void bim088_gyro_offset_update(void);
void bsp_imu_temp_control(float temp);
void bsp_imu_pwm_set(uint16_t pwm);

static void imu_cmd_spi_dma(void);


/*����DMA������*/
uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2,0xFF,0xFF,0xFF};
/*����DMA������*/

/*DMA״̬��־*/
volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;




bmi088_real_data_t bmi088_real_data;
vsky_ahrs_info mvsky_ahrs_info = 
{
	.imu_cali = SKY_IMU_CALI_NULL,
};

pid_type_def imu_temp_pid;
float imu_mag[3] = {0,0,0};


uint64_t systick_us;
uint8_t  first_temperate;

void bsp_bmi088_drive_init(void)
{
//    float pid_temp[3] = { 1600.0f,0.2f,0.0f};
//    PID_init(&imu_temp_pid, PID_POSITION, pid_temp, 4500.0f,4500.0f);

    while(bsp_bmi088_init())
        {
            osDelay(100);
        }


    bsp_bmi088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    //����SPIƵ��
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
        {
            Error_Handler();
        }
    SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
    imu_start_dma_flag = 1;
}

void bsp_bmi088_read_updata(void)
{
    //��ȡ�Ĵ�����ֵ
    if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))//�ж�bit2�Ƿ�Ϊ1
        {
            //bit2λ��0
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            bsp_bmi088_gyro_read(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET,bmi088_real_data.gyro);
        }
    if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            bsp_bmi088_accel_read(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET,bmi088_real_data.accel,&bmi088_real_data.time);
        }
    if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            bsp_bmi088_temperature_read(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET,&bmi088_real_data.temp);
        }

    systick_us = bsp_bmi088_get_us()+osKernelSysTick()*1000;
//	systick_us+=2000;
    vsky_imu_updata(&mvsky_ahrs_info,bmi088_real_data.accel,bmi088_real_data.gyro,imu_mag,systick_us);
//    bsp_imu_temp_control(bmi088_real_data.temp);
}


/**
  * @brief          ����bmi088���¶�
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
void bsp_imu_temp_control(float temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
        {
            PID_calc(&imu_temp_pid, temp, BMI088_TEMP_MAX);
            if (imu_temp_pid.out < 0.0f)
                {
                    imu_temp_pid.out = 0.0f;
                }
            tempPWM = (uint16_t)imu_temp_pid.out;
            bsp_imu_pwm_set(tempPWM);
        }
    else
        {
            //��û�дﵽ���õ��¶ȣ�һֱ����ʼ���
            //in beginning, max power 141
            if (temp > BMI088_TEMP_MAX)
                {
                    temp_constant_time++;
                    if (temp_constant_time > 200)
                        {
                            //�ﵽ�����¶ȣ�������������Ϊһ������ʣ���������
                            first_temperate = 1;
                            imu_temp_pid.Iout = BMI088_TEMP_PWM_MAX / 2.0f;
                        }
                }
            bsp_imu_pwm_set(BMI088_TEMP_PWM_MAX - 1);
        }
}

void bsp_imu_pwm_set(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)
        {
            accel_update_flag |= 1 << IMU_DR_SHFITS;
            accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
            if(imu_start_dma_flag)
                {
                    imu_cmd_spi_dma();
                }
        }
    else if(GPIO_Pin == INT1_GYRO_Pin)
        {
            gyro_update_flag |= 1 << IMU_DR_SHFITS;
            if(imu_start_dma_flag)
                {
                    imu_cmd_spi_dma();
                }
        }
    else if(GPIO_Pin == DRDY_IST8310_Pin)
        {
            mag_update_flag |= 1 << IMU_DR_SHFITS;
            if(mag_update_flag &= 1 << IMU_DR_SHFITS)
                {
                    mag_update_flag &= ~(1<< IMU_DR_SHFITS);
                    mag_update_flag |= (1 << IMU_SPI_SHFITS);
//            ist8310_read_mag(ist8310_real_data.mag);
                }
        }
//��ʹ���жϻ��ѷ�ʽ
//    else if(GPIO_Pin == GPIO_PIN_0)
//    {
//        //��������
//        if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
//        {
//            static BaseType_t xHigherPriorityTaskWoken;
//            vTaskNotifyGiveFromISR(imu_Task_handle, &xHigherPriorityTaskWoken);
//            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//        }
//    }
}

/**
  * @brief          ����imu_update_flag��ֵ����SPI DMA
  * @param[in]      temp:bmi088���¶�
  * @retval         none
  */
static void imu_cmd_spi_dma(void)
{
    //���������ǵ�DMA����
    //�ж�gyro_update_flag bit0�Ƿ�Ϊ1,accel_update_flag bit1�Ƿ�Ϊ1,accel_temp_update_flag bit1�Ƿ�Ϊ1
    if( (gyro_update_flag & (1 << IMU_DR_SHFITS) ) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
            && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            gyro_update_flag &= ~(1 << IMU_DR_SHFITS);//bit0��0
            gyro_update_flag |= (1 << IMU_SPI_SHFITS);//bit1��1
            //GYRO��ʼ����
            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);//Ƭѡ��GYRO
            SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf,SPI_DMA_GYRO_LENGHT);
            return;
        }
    //�������ٶȼƵ�DMA����
    //accel_update_flag bit0�Ƿ�Ϊ1,gyro_update_flag bit1�Ƿ�Ϊ1,accel_temp_update_flag bit1�Ƿ�Ϊ1
    if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
            && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_update_flag &= ~(1 << IMU_DR_SHFITS);//bit0��0
            accel_update_flag |= (1 << IMU_SPI_SHFITS);//bit1��1
            //ACCEL��ʼ����
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);//Ƭѡ��ACCEL
            SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
            return;
        }
    //accel_temp_update_flag bit0�Ƿ�Ϊ1,gyro_update_flag bit1�Ƿ�Ϊ1,accel_update_flag bit1�Ƿ�Ϊ1
    if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
            && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
        {
            accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);//bit0��0
            accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);//bit1��1

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);//Ƭѡ��ACCEL
            SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
            return;
        }
}
void DMA2_Stream2_IRQHandler(void)
{
    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
        {
            __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));
            //�����Ƕ�ȡ���
            //�ж�bit1�Ƿ�Ϊ1
            if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
                {
                    gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);//bit1��0
                    gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);//bit2��1

                    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET); //ȡ��GYROƬѡ
                }
            //���ٶȼƶ�ȡ���
            if(accel_update_flag & (1 << IMU_SPI_SHFITS))
                {
                    accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
                    accel_update_flag |= (1 << IMU_UPDATE_SHFITS);
                    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
                }
            //�¶ȶ�ȡ���
            if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
                {
                    accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
                    accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);
                    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
                }
            imu_cmd_spi_dma();
        }
}



