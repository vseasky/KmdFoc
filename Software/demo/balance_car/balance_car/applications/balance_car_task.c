#include "balance_car_task.h"
#include "bsp_usb.h"
#include "cmsis_os.h"
#include "FreeRtos.h"


#include "hal_kmd_interface.h"
#include "hal_foc_enum.h"
#include "vSkyAhrs.h"
#include "arm_math.h"

#define AngleToRadian (0.01745329251994329576923690768489f)


#define MaxLimit(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

//内部闭环已经有了，此处闭环周期没必要太高
#define BALANCE_CAR_TASK_TIME 5

//extern tHalFocInfo mHalFocInfo1;
//extern tHalFocInfo mHalFocInfo2;
//extern hal_kmd_user_struct tHalKmdUser;
extern vsky_ahrs_info mvsky_ahrs_info;

uint8_t check_cali[2] = {0,0};
uint8_t check_mode[2] = {0,0};
uint8_t check_encoder[2] = {0,0};
int     hal_cpr[2] = {0,0};


float  kmd_foc_vel_kp = 1.5;
float  kmd_foc_vel_ki = 0.00;
//设置每5ms自动上报一次数据，如果需要上报所有数据，则第一个5ms上报HAL_KMD_FSM_HEARTBEAT0,第二个上报HAL_KMD_POS_HEARTBEAT1...
int    kmd_foc_report_ms = 5;
//设置需要上报的数据，在此只需要转速
int	   kmd_foc_report_ch = ((0<<(HAL_KMD_FSM_HEARTBEAT0-HAL_KMD_FSM_HEARTBEAT0))|
						    (0<<(HAL_KMD_POS_HEARTBEAT1-HAL_KMD_FSM_HEARTBEAT0))|
						    (1<<(HAL_KMD_VEL_HEARTBEAT2-HAL_KMD_FSM_HEARTBEAT0))|
						    (0<<(HAL_KMD_CUR_HEARTBEAT3-HAL_KMD_FSM_HEARTBEAT0))|
						    (0<<(HAL_KMD_BUS_HEARTBEAT4-HAL_KMD_FSM_HEARTBEAT0)));

//float   balance_pid_kp = 0.5;
//float   balance_pid_kd = 0.035;

float   balance_pid_kp = 60.0;
float   balance_pid_kd = 5.6;

//float   balance_pid_kp = 0;
//float   balance_pid_kd = 0;
//float   balance_last_angle = 0;
float   balance_out = 0;
float   balance_pout = 0;
float   balance_dout = 0;
float   balance_sin = 0;

float   velicoty_pid_kp = 10;
float   velicoty_pid_ki = 10;
float   velicoty_last_speed[2] = {0,0};
float   velicoty_out[2] = {0,0};

float   turn_pid_kp = 10;
float   turn_last_yaw;


osThreadId BalanceCarTaskHandle;
static void balance_task(void const *pvParameters);

void balance_task_creat(void)
{
    osThreadDef(BalanceCarTask,balance_task,osPriorityRealtime,0,128);
    BalanceCarTaskHandle = osThreadCreate(osThread(BalanceCarTask), NULL);
}

static void balance_task(void const *pvParameters)
{
    TickType_t peroid = osKernelSysTick();
	osDelay(3000);
    /*KmdFoc将上电后第一个获取版本号的通信方式设定本次上电的控制方式，其它接口将无法用于控制*/
    tHalKmdUser.hal_kmd_get_version_hook(&mHalFocInfo1.__tx_frame,mHalFocInfo1.node_id);
    osDelay(1);
    tHalKmdUser.hal_kmd_get_version_hook(&mHalFocInfo2.__tx_frame,mHalFocInfo2.node_id);
    osDelay(1);
    /*发送获取配置信息请求,配置信息在CAN中断接收后会更新在 mHalFocInfo1.__kmd_user.__config 数组中,而具体的参数类型是int或float可以查阅 kmd_config_map[][2]数组*/
    for(uint8_t i=0; i<KMD_CONFIG_MAP_MAX_LENGTH; i++)
        {
			uint8_t mData[4];
			if(kmd_config_map[i][1] == HAL_USER_CONFIG_VEL_GAIN)
			{
				float_to_data(kmd_foc_vel_kp,mData);
			    tHalKmdUser.kmd_config.hal_kmd_user_config_set_hook(&mHalFocInfo1.__tx_frame,mHalFocInfo1.node_id,kmd_config_map[i][1],&mData[0]);
				osDelay(1);
				float_to_data(kmd_foc_vel_kp,mData);
				tHalKmdUser.kmd_config.hal_kmd_user_config_set_hook(&mHalFocInfo2.__tx_frame,mHalFocInfo2.node_id,kmd_config_map[i][1],&mData[0]);
				osDelay(1);
			}
			if(kmd_config_map[i][1] == HAL_USER_CONFIG_VEL_INTEGRATOR_GAIN)
			{

				float_to_data(kmd_foc_vel_ki,mData);
			    tHalKmdUser.kmd_config.hal_kmd_user_config_set_hook(&mHalFocInfo1.__tx_frame,mHalFocInfo1.node_id,kmd_config_map[i][1],&mData[0]);
				osDelay(1);
				float_to_data(kmd_foc_vel_ki,mData);
				tHalKmdUser.kmd_config.hal_kmd_user_config_set_hook(&mHalFocInfo2.__tx_frame,mHalFocInfo2.node_id,kmd_config_map[i][1],&mData[0]);
				osDelay(1);
			}
			if(kmd_config_map[i][1] == HAL_USER_CONFIG_CAN_HEARTBEAT_MS)
			{

				int_to_data(kmd_foc_report_ms,mData);
			    tHalKmdUser.kmd_config.hal_kmd_user_config_set_hook(&mHalFocInfo1.__tx_frame,mHalFocInfo1.node_id,kmd_config_map[i][1],&mData[0]);
				osDelay(1);
				tHalKmdUser.kmd_config.hal_kmd_user_config_set_hook(&mHalFocInfo2.__tx_frame,mHalFocInfo2.node_id,kmd_config_map[i][1],&mData[0]);
				osDelay(1);
			}
			if(kmd_config_map[i][1] == HAL_USER_CONFIG_CAN_HEARTBEAT_CH)
			{

				int_to_data(kmd_foc_report_ms,mData);
			    tHalKmdUser.kmd_config.hal_kmd_user_config_set_hook(&mHalFocInfo1.__tx_frame,mHalFocInfo1.node_id,kmd_config_map[i][1],&mData[0]);
				osDelay(1);
				tHalKmdUser.kmd_config.hal_kmd_user_config_set_hook(&mHalFocInfo2.__tx_frame,mHalFocInfo2.node_id,kmd_config_map[i][1],&mData[0]);
				osDelay(1);
			}
        }

    /*发送获取配置信息请求,配置信息在CAN中断接收后会更新在 mHalFocInfo1.__kmd_user.__config 数组中,而具体的参数类型是int或float可以查阅 kmd_config_map[][2]数组*/
    for(uint8_t i=0; i<KMD_CONFIG_MAP_MAX_LENGTH; i++)
        {
            tHalKmdUser.kmd_config.hal_kmd_user_config_get_hook(&mHalFocInfo1.__tx_frame,mHalFocInfo1.node_id,kmd_config_map[i][1]);
            osDelay(1);
            tHalKmdUser.kmd_config.hal_kmd_user_config_get_hook(&mHalFocInfo2.__tx_frame,mHalFocInfo2.node_id,kmd_config_map[i][1]);
            osDelay(1);
        }
//    /*如果是单独获取某个参数，则可以*/
//    {
//        /*是否已经校准*/
//        tHalKmdUser.kmd_config.hal_kmd_user_config_get_hook(&mHalFocInfo1.__tx_frame,mHalFocInfo1.node_id,HAL_USER_CONFIG_CALIB_VALID);
//        osDelay(1);
//        tHalKmdUser.kmd_config.hal_kmd_user_config_get_hook(&mHalFocInfo2.__tx_frame,mHalFocInfo2.node_id,HAL_USER_CONFIG_CALIB_VALID);
//        osDelay(1);
//        /*控制模式*/
//        tHalKmdUser.kmd_config.hal_kmd_user_config_get_hook(&mHalFocInfo1.__tx_frame,mHalFocInfo1.node_id,HAL_USER_CONFIG_CONTROL_MODE);
//        osDelay(1);
//        tHalKmdUser.kmd_config.hal_kmd_user_config_get_hook(&mHalFocInfo2.__tx_frame,mHalFocInfo2.node_id,HAL_USER_CONFIG_CONTROL_MODE);
//        osDelay(1);
//    }
    /*等待数据返回*/
    osDelay(10);

    /*查询数据*/
    for(uint8_t i=0; i<KMD_CONFIG_MAP_MAX_LENGTH; i++)
        {
            if(kmd_config_map[i][1] == HAL_USER_CONFIG_CONTROL_MODE)
                {
                    //是否工作于转矩模式
                    if(mHalFocInfo1.__kmd_user.__config[i].value_int == HAL_CONTROL_MODE_CURRENT)
                        {
                            check_mode[0] = 1;
                        }
                    if(mHalFocInfo2.__kmd_user.__config[i].value_int == HAL_CONTROL_MODE_CURRENT)
                        {
                            check_mode[1] = 1;
                        }
                }
            if(kmd_config_map[i][1] == HAL_USER_CONFIG_CALIB_VALID)
                {
                    //是否已经校准
                    if(mHalFocInfo1.__kmd_user.__config[i].value_int == 1)
                        {
                            check_cali[0] = 1;
                        }
                    if(mHalFocInfo2.__kmd_user.__config[i].value_int == 1)
                        {
                            check_cali[1] = 1;
                        }
                }
            if(kmd_config_map[i][1] == HAL_USER_CONFIG_ENCODER_TYPE)
                {
                    //检查编码器是否为HALL
                    if(mHalFocInfo1.__kmd_user.__config[i].value_int == HAL_ENCODER_HALL)
                        {
                            check_encoder[0] = 1;
                        }
                    if(mHalFocInfo2.__kmd_user.__config[i].value_int == HAL_ENCODER_HALL)
                        {
                            check_encoder[1] = 1;
                        }
                }
            if(kmd_config_map[i][1] == HAL_USER_CONFIG_ENCODER_CPR)
                {
                    /*检查轮毂电机编码器CPR = 6*极对数*/
                    /*轮毂电机的编码器必须设置为 6*极对数并进行校准，绝对值编码器则会自动设置*/
                    hal_cpr[0] = mHalFocInfo1.__kmd_user.__config[i].value_int;
                    hal_cpr[1] = mHalFocInfo2.__kmd_user.__config[i].value_int;
                }
            /*必要的，你还可以检查闭环等诸多参数，以检测KmdFoc的值是否被有效的保存，或者你在使用上位机后未保存到Flash*/
            /************************************************************************************************/
        }
    if((check_cali[0]==0)|(check_cali[1]==0))
        {
            //debug_printf()
            while(1);
        }
    if((check_mode[0]==0)|(check_mode[1]==0))
        {
            //debug_printf()
            while(1);

        }
    if((check_encoder[0]==0)|(check_encoder[1]==0))
        {
            //debug_printf()
            while(1);

        }
    if((check_cali[0]==0)|(check_cali[1]==0))
        {
            //debug_printf()
            while(1);
        }
    if((hal_cpr[0]==90)&(hal_cpr[1]==90))
        {
            tHalKmdUser.kmd_fsm.hal_kmd_fsm_motor_enable_hook(&mHalFocInfo1.__tx_frame,mHalFocInfo1.node_id);
            osDelay(1);
            tHalKmdUser.kmd_fsm.hal_kmd_fsm_motor_enable_hook(&mHalFocInfo2.__tx_frame,mHalFocInfo2.node_id);
            osDelay(1);
        }
    else
        {
            //debug_printf()
            while(1);
        }
    //设定一个较小的转速进行测试
    {
        //左右电机的转向应该反向
        tHalKmdUser.hal_kmd_set_current_hook(&mHalFocInfo1.__tx_frame,mHalFocInfo1.node_id,0);
        tHalKmdUser.hal_kmd_set_current_hook(&mHalFocInfo2.__tx_frame,mHalFocInfo2.node_id,0);
    }
    {
	
    }
	balance_out = 0;
	balance_pout = 0;
	balance_dout = 0;
    while(1)
        {
            //等待陀螺仪校准Ok，再开始控制
            if(SKY_IMU_CALI_OK == mvsky_ahrs_info.imu_cali)
                {
                    /*计算平衡环-双电机共用*/
                    {
						if(fabs(mvsky_ahrs_info.vAngle_data[1]-0.4)<45)
						{
							balance_sin = sinf((mvsky_ahrs_info.vAngle_data[1]-0.40)*AngleToRadian);
							//差分->>一般都是用的 kp*Angle,相比之下使用 kp*sin(Angle)的鲁棒性会更高
							balance_pout =    balance_pid_kp*balance_sin;
							//角度微分->>转速，因此直接用Gyro替代微分项，以获得更准确的角速度
							balance_dout =    balance_pid_kd*(mvsky_ahrs_info.vGyro_data[1]);
							MaxLimit(balance_pout,40);//35A
							MaxLimit(balance_dout,40);
							balance_out = balance_pout+balance_dout;
							MaxLimit(balance_out,40);
						}
						else
						{
							balance_sin = 0;
							balance_dout = 0;
							balance_out = 0;
						}
                    }
                    /*计算速度环*/
                    {
                        //暂时不写，等自己的开发板

                    }
                    /*计算转向环*/
                    {
                         //暂时不写，等自己的开发板
                    }

                    /*给定速度*/
                    {
                        //左右电机的转向应该反向
                        tHalKmdUser.hal_kmd_set_current_hook(&mHalFocInfo1.__tx_frame,mHalFocInfo1.node_id,-balance_out);
                        tHalKmdUser.hal_kmd_set_current_hook(&mHalFocInfo2.__tx_frame,mHalFocInfo2.node_id,balance_out);
                    }
                }
            else
                {
                    balance_out = 0;
                    /*给定速度*/
                    {
                        //左右电机的转向应该反向
                        tHalKmdUser.hal_kmd_set_current_hook(&mHalFocInfo1.__tx_frame,mHalFocInfo1.node_id,-balance_out);
                        tHalKmdUser.hal_kmd_set_current_hook(&mHalFocInfo2.__tx_frame,mHalFocInfo2.node_id,balance_out);
                    }
                }
            osDelayUntil(&peroid,BALANCE_CAR_TASK_TIME);
        }
}