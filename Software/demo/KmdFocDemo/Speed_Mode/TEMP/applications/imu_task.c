#include "imu_task.h"

#include "cmsis_os.h"

#include "bsp_bmi088.h"
#include "bsp_protocol_app.h"

extern void bsp_bmi088_drive_init(void);
extern void bsp_bmi088_read_updata(void);
osThreadId imuTaskHandle;
void imu_task_creat(void)
{
    osThreadDef(imuTask, imu_task, osPriorityRealtime, 0, 512);
    imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);
}
//static TaskHandle_t imu_Task_handle;//此任务大概2ms
void imu_task(void const *pvParameters)
{
    TickType_t peroid = osKernelSysTick();
    /*开启任务延时时间(单位ms)*/
    osDelay(IMU_TASK_INIT_TIME1);
    bsp_bmi088_drive_init();
    while(1)
        {
            bsp_bmi088_read_updata();
            osDelayUntil(&peroid,IMU_TASK_PERIOD);
        }
}


