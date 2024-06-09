#include "hal_foc_struct.h"
#include "hal_kmd_foc.h"
#include "hal_kmd_interface.h"
// extern const uint8_t KMD_CONFIG_MAP_MAX_LENGTH;
// extern const char kmd_config_map[][2];
//  [数据类型,CONFIG_CMD_ID,name]
const uint8_t KMD_CONFIG_MAP_MAX_LENGTH = 33;
const uint8_t kmd_config_map[][2] =
{
    // Name,单位,控件类型,CMD，提示
    {KMD_TYPE_INT, HAL_USER_CONFIG_MOTOR_POLE_PAIRS},           //"motor_pole_pairs", "Auto","电磁极对数，在校准过程中自动测量，目前支持2~30极对数，无需设置"},
    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_MOTOR_PHASE_RESISTANCE},   //"motor_phase_resistance", "Auto", "电机定子绕组线圈电阻值，在校准过程中自动测量,无需设置。"},
    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_MOTOR_PHASE_INDUCTANCE},   //"motor_phase_inductance", "Auto","电机定子线圈相电感值，在校准过程中自动测量，无需设置。"},
    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_INERTIA},                  //"inertia", "A/(turn/s^2)","inertia", "A/(turn/s^2)","设置转动惯量，电机轴维持以1转每秒的加速度运行时所需要提供的电流值(能量)，需要根据电机轴重量和所带负载来进行调试,HAL_CONTROL_MODE_VELOCITY_RAMP和HAL_CONTROL_MODE_POSITION_TRAP控制模式下有效。"},
    // Encoder
    {KMD_TYPE_INT, HAL_USER_CONFIG_ENCODER_TYPE},               //"encoder_type", "","设置编码器类型，更改编码器之后必须复位和重启，有效更改复位和重启前还应该先保存设置。"},
    {KMD_TYPE_INT, HAL_USER_CONFIG_ENCODER_PLL_BW},             //"encoder_pll_bandwidth", "Auto","设置编码器 PLL 带宽，一般对于高分辨率编码器 (> 4000个计数/转) 此值应该越高，这样有助于减少电机振动。对于HALL这种分辨率很低的编码器，不宜设置过高的参数。"},
    {KMD_TYPE_INT, HAL_USER_CONFIG_ENCODER_CPR},                //"encoder_cpr", "Auto","设置编码器分辨率，绝对值编码器无需设置，对于HALL编码器必须在校准前设置为(6*极对数)。},
    // Calib
    {KMD_TYPE_INT, HAL_USER_CONFIG_CALIB_VALID},                //"calib_valid", "Auto", "当前校准数据是否有效，校准完成后设置为True。"},
    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_CALIB_CURRENT},            //"calib_current", "A","设置校准时的电流值，设置太小校准时无法顺畅转动，设置过大则可能导致电机发热严重。"},
    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_CALIB_MAX_VOLTAGE},        //"calib_max_voltage", "V","设置校准时的最大电压。"},
    // Control
    {KMD_TYPE_INT, HAL_USER_CONFIG_CONTROL_MODE},               //"control_mode", "","设置电机控制模式,一共有六种模式。"},
    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_CURRENT_RAMP_RATE},        //"current_ramp_rate", "A/sec", "设置电流爬升控制模式下电流爬升速度，电机控制模式为CURRENT_RAMP时有效。"},
    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_VEL_RAMP_RATE},            //"ramp_rate", "(turn/s)/s", "设置转速爬升模式下转速爬升速度，电机控制模式为VELOCITY_RAMP时有效。"},
    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_TRAJ_VEL},                 //"traj_vel", "turn/s", "设置梯形轨迹控制模式下最大转速，电机控制模式为POSITION_TRAP时有效。"},
    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_TRAJ_ACCEL},               //"traj_accel", "(turn/s)/s", "设置梯形轨迹控制模式下加速度，电机控制模式为POSITION_TRAP时有效。"},
    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_TRAJ_DECEL},               //"traj_decel", "(turn/s)/s", "设置梯形轨迹控制模式下减速度，电机控制模式为POSITION_TRAP时有效。"},
    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_POS_GAIN},                 //"pos_gain", "(turn/s)/turn","设置位置环增益。"},
    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_VEL_GAIN},                 //"vel_gain", "A/(turn/s)", "设置转速环增益。"},
    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_VEL_INTEGRATOR_GAIN},      //"vel_integrator_gain", "A/((turn/s)*s)", "设置转速环积分增益。"},
    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_VEL_LIMIT},                //"vel_limit", "turn/s","设置转速环限制。"},
    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_CURRENT_LIMIT},            //"current_limit", "A","设置电流环限制。"},                                                       
    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_CURRENT_CTRL_P_GAIN},      //"current_ctrl_p_gain", "Auto","设置电流环增益，在校准过程中自动计算得到，也可自行设置。"}, 
    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_CURRENT_CTRL_I_GAIN},      //"current_ctrl_i_gain", "Auto", "设置电流环积分增益，在校准过程中自动计算得到，也可自行设置。"}, 
    {KMD_TYPE_INT, HAL_USER_CONFIG_CURRENT_CTRL_BW},            //"current_ctrl_bandwidth", "rad/s","设置电流环滤波带宽，更新带宽时也会同时更新电流环增益和电流环积分增益。[100~2000]"},
    // Protect
    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_PROTECT_UNDER_VOLTAGE},    //"protect_under_voltage", "V", "设置低压保护阈值。"}, 
    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_PROTECT_OVER_VOLTAGE},     //"protect_over_voltage", "V", "设置过压保护阈值。""},   
    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_PROTECT_OVER_CURRENT},     //"protect_over_current", "A", "设置过流保护阈值。"},   

    {KMD_TYPE_FLOAT, HAL_USER_CONFIG_PROTECT_OVER_SPEED},       //"protect_over_speed", "turn/s","设置电机超速保护阈值。"}, 
    // CAN
    {KMD_TYPE_INT, HAL_USER_CONFIG_CAN_ID},                     //"can_id", "","设置设备ID，范围1~16。"}, 
    {KMD_TYPE_INT, HAL_USER_CONFIG_CAN_TIMEOUT_MS},             //"can_timeout_ms", "","设置通信超时时间，超过此时间未进行通信，电机将停止运行，设置为0则不启用。如果启用，设置值需满足大于500ms，默认0。"}, 
    {KMD_TYPE_INT, HAL_USER_CONFIG_CAN_HEARTBEAT_MS},           //"can_report_heart_ms", "","设置设备心跳，设备将周期性上报状态和位置，设置为0则不启用。如果启用，设置值需满足大于2ms，特别的串口方式需要大于10ms,默认0。"},     
    {KMD_TYPE_INT, HAL_USER_CONFIG_CAN_HEARTBEAT_CH},           //"cen_report_heart_choose", "","选择can心跳周期性更新的数据，默认[(1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)]。"},      
    
    // Debug
    {KMD_TYPE_INT, HAL_USER_CONFIG_UART_DEBUG_MS},              //"uart_debug_ms", "",  "设置串口开启调试，按设定周期打印数据波形。设置为0则不启用。如果启用，设置值需满足大于15ms,默认0。"} 
};

//(((uint16_t)pFocFsm->state) << 16) | (pFocFsm->motor_err & 0XFFFF);
static void parse_motor_state(uint16_t *pFsm, uint16_t *pErr, uint32_t tData)
{
    *pFsm = tData >> 16;
    *pErr = tData & 0XFFFF;
}

/**
 * @Description  : 解析配置数据
 * @param         (tHalFocUserConfig) *pKmdUserConfigInfo
 * @return        (*)
 */
bool hal_kmd_user_config_receive_callback(hal_frame_struct *pFrame, tHalFocUserConfig *pKmdUserConfigInfo)
{
    uint32_t kmdUserCmdId = 0;
    uint32_t kmdUserResult = 0;
    uint32_t kmdUserTxOeder;
    kmdUserTxOeder = data_to_uint32(&pFrame->data[0]);
    hal_parse_order_id(&kmdUserCmdId, &kmdUserResult, kmdUserTxOeder);
    uint16_t config_index;
    for (config_index = 0; config_index < KMD_CONFIG_MAP_MAX_LENGTH; config_index++)
        {
            if (kmd_config_map[config_index][1] == kmdUserCmdId)
                {
                    switch (kmd_config_map[config_index][0])
                        {
                        case KMD_TYPE_FLOAT:
                        {
                            pKmdUserConfigInfo->__config[config_index].value_float = data_to_float(&pFrame->data[4]);
                        }
                        break;
                        case KMD_TYPE_INT:
                        {
                            pKmdUserConfigInfo->__config[config_index].value_int = data_to_int(&pFrame->data[4]);
                        }
                        break;
                        }
                }
        }
}

/**
 * @Description  : HAL_KMD_MOTOR_CTR 对应的FOC状态控制返回
 * @param         (tHalFocControl) *pKmdCtr
 * @return        (*)
 */
bool hal_kmd_motor_receive_callback(hal_frame_struct *pFrame, tHalFocControl *pKmdCtr)
{
    uint32_t kmdUserCmdId = 0;
    uint32_t kmdUserResult = 0;
    uint32_t kmdUserTxOeder;
    kmdUserTxOeder = data_to_uint32(&pFrame->data[0]);
    hal_parse_order_id(&kmdUserCmdId, &kmdUserResult, kmdUserTxOeder);

    switch (kmdUserCmdId)
        {

        case HAL_MOTOR_DISABLE: //失能电机              pData[1] = 当前位置
        {
            if (kmdUserResult == HAL_RESULT_OK)
                {
                    pKmdCtr->__motor_states.motor_fsm_mode = HAL_FSM_MENU_MODE;
                }
        }
        break;
HAL_MOTOR_ENABLE: //使能电机              pData[1] = 当前位置
        {
            if (kmdUserResult == HAL_RESULT_OK)
                {
                    pKmdCtr->__closed_loop.target.position = data_to_float(&pFrame->data[4]);
                    pKmdCtr->__closed_loop.target.velocity = 0;
                    pKmdCtr->__closed_loop.target.current_ = 0;
                    pKmdCtr->__motor_states.motor_fsm_mode = HAL_FSM_MOTOR_MODE;
                }
        }
        break;
HAL_MOTOR_ERROR_REPORT: //报告错误              pData[1] = 错误信息，自动上报，获取无效
        {
            if (kmdUserResult == HAL_RESULT_OK)
                {
                    parse_motor_state(
                        &pKmdCtr->__motor_states.motor_fsm_mode,
                        &pKmdCtr->__motor_states.motor_error,
                        data_to_uint32(&pFrame->data[4]));
                }
        }
        break;
HAL_MOTOR_ERROR_RESET: //复位错误              pData[1] = 0
        {
            if (kmdUserResult == HAL_RESULT_OK)
                {
                    pKmdCtr->__motor_states.motor_error = 0;
                }
        }
        break;
HAL_MOTOR_GET_STAT: //获取电机状态          pData[1] = 错误状态
        {
            if (kmdUserResult == HAL_RESULT_OK)
                {
                    parse_motor_state(
                        &pKmdCtr->__motor_states.motor_fsm_mode,
                        &pKmdCtr->__motor_states.motor_error,
                        data_to_uint32(&pFrame->data[4]));
                }
        }
        break;
HAL_MOTOR_CALIBRATION_START: //开始校准              pData[1] = 0
        {
            if (kmdUserResult == HAL_RESULT_OK)
                {
                    pKmdCtr->__motor_states.motor_fsm_mode = HAL_FSM_CALIBRATION_MODE;
                }
        }
        break;
HAL_MOTOR_CALIBRATION_ABORT: //复位校准              pData[1] = 0
        {
            if (kmdUserResult == HAL_RESULT_OK)
                {
                    pKmdCtr->__motor_states.motor_fsm_mode = HAL_FSM_MENU_MODE;
                }
        }
        break;
HAL_MOTOR_CONFIGS_UPDATA: //保存参数             pData[1] = 0
        {
            if (kmdUserResult == HAL_RESULT_OK)
                {
                    //保存成功
                }
        }
        break;
HAL_MOTOR_CONFIGS_RESET_ALL: //复位所有参数         pData[1] = 0
        {
            if (kmdUserResult == HAL_RESULT_OK)
                {
                    //复位成功
                    pKmdCtr->__motor_states.motor_fsm_mode = HAL_FSM_MENU_MODE;
                }
        }
        break;
        default:
            ;
            break;
        }
    return true;
}

/**
 * @Description  : FOC校准结果返回值
 * @return        (*)
 */
bool hal_kmd_cali_receive_callback(hal_frame_struct *pFrame, tHalFocControl *pKmdCtr)
{
    uint32_t OrderCmdId = 0;
    uint32_t OrderResult = 0;
    hal_parse_order_id(&OrderCmdId, &OrderResult, data_to_uint32(&pFrame->data[0]));

    switch (OrderCmdId)
        {
        case HAL_CALIBRATION_ERROR:
        {
            pKmdCtr->__cali_info.cali_error = data_to_uint32(&pFrame->data[4]);
        }
        break;
        case HAL_CALIBRATION_RES:
        {
            pKmdCtr->__cali_info.cali_res = data_to_float(&pFrame->data[4]);
        }
        break;
        case HAL_CALIBRATION_IND:
        {
            pKmdCtr->__cali_info.cali_ind = data_to_float(&pFrame->data[4]);
        }
        break;
        case HAL_CALIBRATION_PAIRS:
        {
            pKmdCtr->__cali_info.cali_pairs = data_to_int(&pFrame->data[4]);
        }
        break;
        case HAL_CALIBRATION_DIR:
        {
            pKmdCtr->__cali_info.cali_dir = data_to_int(&pFrame->data[4]);
        }
        break;
        case HAL_CALIBRATION_OFFSET:
        {
            pKmdCtr->__cali_info.cali_offset = data_to_int(&pFrame->data[4]);
        }
        break;
        case HAL_CALIBRATION_POSITION:
        {
            pKmdCtr->__cali_info.cali_position = data_to_float(&pFrame->data[4]);
        }
        break;
        case HAL_CALIBRATION_END:
        {
            pKmdCtr->__cali_info.cali_error = data_to_uint32(&pFrame->data[4]);
        }
        break;
        default:
            break;
        }
    if (OrderCmdId > HAL_CALIBRATION_OFFSET_LUT)
        {
            pKmdCtr->__cali_info.__offset_lut[OrderCmdId - HAL_CALIBRATION_OFFSET_LUT].value_int = data_to_int(&pFrame->data[4]);
        }
    return true;
}

bool hal_kmd_frame_receive_callback(hal_frame_struct *pFrame, tHalFocInfo *pKmdInfo)
{
    uint32_t node_id;
    uint32_t cmd_id;

    hal_parse_frame_id(&node_id, &cmd_id, pFrame->can_id);

    if (node_id != pKmdInfo->node_id)
        {
            return false;
        }
		
    switch (cmd_id)
        {
        case HAL_KMD_MOTOR_DISABLE: //失能电机
        {
        }
        break;
        case HAL_KMD_MOTOR_CTR: //电机有限状态机控制	    ->返回值 [(result<<16)|(tKmdMotorCmd),float]
        {
            hal_kmd_motor_receive_callback(pFrame, &pKmdInfo->__kmd_ctr);
        }
        break;
        case HAL_KMD_CALIBRATION_REPORT: //报告校准
        {
            hal_kmd_cali_receive_callback(pFrame, &pKmdInfo->__kmd_ctr);
        }
        break;
        case HAL_KMD_SET_TARGET_POSITION: //设置目标位置			->返回值 [float,0]
        {
            pKmdInfo->__kmd_ctr.__closed_loop.target.position = data_to_float(&pFrame->data[0]);
        }
        break;
        case HAL_KMD_SET_TARGET_VELOCITY: //设置目标转速			->返回值 [float,0]
        {
            pKmdInfo->__kmd_ctr.__closed_loop.target.velocity = data_to_float(&pFrame->data[0]);
        }
        break;
        case HAL_KMD_SET_TARGET_CURRENT: //设置目标电流			->返回值 [float,0]
        {
            pKmdInfo->__kmd_ctr.__closed_loop.target.current_ = data_to_float(&pFrame->data[0]);
        }
        break;
        case HAL_KMD_GET_POSITION: //目标位置，实际位置	->返回值 [float,float]
        {
            pKmdInfo->__kmd_ctr.__closed_loop.target.position = data_to_float(&pFrame->data[0]);
            pKmdInfo->__kmd_ctr.__closed_loop.realit.position = data_to_float(&pFrame->data[4]);
        }
        break;
        case HAL_KMD_GET_VELOCITY: //目标转速，实际转速	->返回值 [float,float]
        {
            pKmdInfo->__kmd_ctr.__closed_loop.target.velocity = data_to_float(&pFrame->data[0]);
            pKmdInfo->__kmd_ctr.__closed_loop.realit.velocity = data_to_float(&pFrame->data[4]);
        }
        break;
        case HAL_KMD_GET_CURRENT: //目标电流，实际电流	->返回值 [float,float]
        {
            pKmdInfo->__kmd_ctr.__closed_loop.target.current_ = data_to_float(&pFrame->data[0]);
            pKmdInfo->__kmd_ctr.__closed_loop.realit.current_ = data_to_float(&pFrame->data[4]);
        }
        break;
        case HAL_KMD_GET_FOCBUS: //总线电压，实际电压	->返回值 [float,float]
        {
            pKmdInfo->__kmd_ctr.__bus_info.v_bus_filt = data_to_float(&pFrame->data[0]);
            pKmdInfo->__kmd_ctr.__bus_info.i_bus_filt = data_to_float(&pFrame->data[4]);
        }
        break;
        case HAL_KMD_SET_CONFIG: //设置参数				->返回值 [(result<<16)|(tHalUsrConfigsCmd),实际值(int/float)]
        {
            hal_kmd_user_config_receive_callback(pFrame, &pKmdInfo->__kmd_user);
        }
        break;
        case HAL_KMD_GET_CONFIG: //获取参数				->返回值 [(result<<16)|(tHalUsrConfigsCmd),实际值(int/float)]
        {
            hal_kmd_user_config_receive_callback(pFrame, &pKmdInfo->__kmd_user);
        }
        break;
        case HAL_KMD_GET_FW_VERSION: //获取电机版本号
        {
            pKmdInfo->__kmd_user.__version[0] = pFrame->data[0];
            pKmdInfo->__kmd_user.__version[1] = pFrame->data[1];
            pKmdInfo->__kmd_user.__version[2] = pFrame->data[2];
            pKmdInfo->__kmd_user.__version[3] = pFrame->data[3];
        }
        break;
        //以下内容在设置好心跳周期和心跳内容后，会根据心跳频率自动返回数据，以减少控制时数据请求带来的通信负载
        case HAL_KMD_FSM_HEARTBEAT0: //心跳0
        {
            //更新电机状态和位置
            parse_motor_state(
                &pKmdInfo->__kmd_ctr.__motor_states.motor_fsm_mode,
                &pKmdInfo->__kmd_ctr.__motor_states.motor_error,
                data_to_uint32(&pFrame->data[4]));
            pKmdInfo->__kmd_ctr.__closed_loop.realit.position = data_to_float(&pFrame->data[4]);
        }
        break;
        case HAL_KMD_POS_HEARTBEAT1: //心跳1
        {
            pKmdInfo->__kmd_ctr.__closed_loop.target.position = data_to_float(&pFrame->data[0]);
            pKmdInfo->__kmd_ctr.__closed_loop.realit.position = data_to_float(&pFrame->data[4]);
        }
        break;
        case HAL_KMD_VEL_HEARTBEAT2: //心跳2
        {
            pKmdInfo->__kmd_ctr.__closed_loop.target.velocity = data_to_float(&pFrame->data[0]);
            pKmdInfo->__kmd_ctr.__closed_loop.realit.velocity = data_to_float(&pFrame->data[4]);
        }
        break;
        case HAL_KMD_CUR_HEARTBEAT3: //心跳3
        {
            //更新电机状态和位置
            pKmdInfo->__kmd_ctr.__closed_loop.target.current_ = data_to_float(&pFrame->data[0]);
            pKmdInfo->__kmd_ctr.__closed_loop.realit.current_ = data_to_float(&pFrame->data[4]);
        }
        break;
        case HAL_KMD_BUS_HEARTBEAT4: //心跳4
        {
            //更新电机状态和位置
            pKmdInfo->__kmd_ctr.__bus_info.v_bus_filt = data_to_float(&pFrame->data[0]);
            pKmdInfo->__kmd_ctr.__bus_info.i_bus_filt = data_to_float(&pFrame->data[4]);
        }
        break;
        default:
            break;
        }
    return true;
}


/** 
 * @Description  : 失能电机
 * @param         (uint16_t) node_id
 * @return        (*)
 */
bool hal_kmd_disable_motor(hal_frame_struct *pFrame, uint16_t node_id)
{
    hal_make_frame_id((uint32_t)node_id, (uint32_t)HAL_KMD_MOTOR_DISABLE, &pFrame->can_id);
    //只需要命令
    memset(pFrame->data, 0, 8);
    return kmd_interface_transmit(pFrame);
}

/** 
 * @Description  : HAL_KMD_SET_TARGET_POSITION,//设置目标位置			->消息返回值 [float,0]  当且仅当位置控制模式有效
 * @param         (uint16_t) node_id
 * @param         (float) tData
 * @return        (*)
 */
bool hal_kmd_set_pisotion(hal_frame_struct *pFrame, uint16_t node_id, float tData)
{
    hal_make_frame_id((uint32_t)node_id, (uint32_t)HAL_KMD_SET_TARGET_POSITION, &pFrame->can_id);
    float_to_data(tData, pFrame->data);
    memset(&pFrame->data[4], 0, 4);
    return kmd_interface_transmit(pFrame);
}

/** 
 * @Description  : HAL_KMD_SET_TARGET_VELOCITY,//设置目标转速			->消息返回值 [float,0] 当且仅当转速控制模式有效
 * @param         (uint16_t) node_id
 * @param         (float) tData
 * @return        (*)
 */
bool hal_kmd_set_velocity(hal_frame_struct *pFrame, uint16_t node_id, float tData)
{
    hal_make_frame_id((uint32_t)node_id, (uint32_t)HAL_KMD_SET_TARGET_VELOCITY, &pFrame->can_id);
    float_to_data(tData, pFrame->data);
    memset(&pFrame->data[4], 0, 4);
    return kmd_interface_transmit(pFrame);
}

/** 
 * @Description  : HAL_KMD_SET_TARGET_CURRENT, //设置目标电流			->消息返回值 [float,0] 当且仅当电流控制模式有效
 * @param         (uint16_t) node_id
 * @param         (float) tData
 * @return        (*)
 */
bool hal_kmd_set_current(hal_frame_struct *pFrame, uint16_t node_id, float tData)
{
    hal_make_frame_id((uint32_t)node_id, (uint32_t)HAL_KMD_SET_TARGET_CURRENT, &pFrame->can_id);
    float_to_data(tData, pFrame->data);
    memset(&pFrame->data[4], 0, 4);
    return kmd_interface_transmit(pFrame);
}

/** 
 * @Description  : HAL_KMD_GET_POSITION,		//目标位置，实际位置消息请求	->消息返回值 [float,float]
 * @param         (uint16_t) node_id
 * @return        (*)
 */
bool hal_kmd_get_position(hal_frame_struct *pFrame, uint16_t node_id)
{
    hal_make_frame_id((uint32_t)node_id, (uint32_t)HAL_KMD_GET_POSITION, &pFrame->can_id);
    //只需要命令
    memset(pFrame->data, 0, 8);

    return kmd_interface_transmit(pFrame);
}

/** 
 * @Description  : HAL_KMD_GET_VELOCITY,		//目标转速，实际转速消息请求	->消息返回值 [float,float]
 * @param         (uint16_t) node_id
 * @return        (*)
 */
bool hal_kmd_get_velocity(hal_frame_struct *pFrame, uint16_t node_id)
{
    hal_make_frame_id((uint32_t)node_id, (uint32_t)HAL_KMD_GET_VELOCITY, &pFrame->can_id);
    //只需要命令
    memset(pFrame->data, 0, 8);

    return kmd_interface_transmit(pFrame);
}

/** 
 * @Description  :  HAL_KMD_GET_CURRENT,		//目标电流，实际电流消息请求	->消息返回值 [float,float]
 * @param         (uint16_t) node_id
 * @return        (*)
 */
bool hal_kmd_get_current(hal_frame_struct *pFrame, uint16_t node_id)
{
    hal_make_frame_id((uint32_t)node_id, (uint32_t)HAL_KMD_GET_CURRENT, &pFrame->can_id);
    //只需要命令
    memset(pFrame->data, 0, 8);
    return kmd_interface_transmit(pFrame);
}

/** 
 * @Description  : HAL_KMD_GET_FOCBUS,			//总线电压，实际电压消息请求	->消息返回值 [float,float]
 * @param         (uint16_t) node_id
 * @return        (*)
 */
bool hal_kmd_get_bus(hal_frame_struct *pFrame, uint16_t node_id)
{
    hal_make_frame_id((uint32_t)node_id, (uint32_t)HAL_KMD_GET_FOCBUS, &pFrame->can_id);
    //只需要命令
    memset(pFrame->data, 0, 8);

    return kmd_interface_transmit(pFrame);
}

/** 
 * @Description  : HAL_KMD_GET_FW_VERSION,		//获取电机版本号消息请求
 * @param         (uint16_t) node_id
 * @return        (*)
 */
bool hal_kmd_get_version(hal_frame_struct *pFrame, uint16_t node_id)
{
    hal_make_frame_id((uint32_t)node_id, (uint32_t)HAL_KMD_GET_FW_VERSION, &pFrame->can_id);
    //只需要命令
    memset(pFrame->data, 0, 8);

    return kmd_interface_transmit(pFrame);
}

/** 
 * @Description  : HAL_MOTOR_DISABLE = 0,          //失能电机		pData[1] = 当前位置
 * @param         (uint16_t) node_id
 * @return        (*)
 */
static bool hal_kmd_fsm_motor_disable(hal_frame_struct *pFrame, uint16_t node_id)
{

    uint32_t tOrderId;
    hal_make_frame_id((uint32_t)node_id, (uint32_t)HAL_KMD_MOTOR_CTR, &pFrame->can_id);
    hal_make_order_id((uint32_t)HAL_MOTOR_DISABLE, 0, &tOrderId);
    uint32_to_data(tOrderId, &pFrame->data[0]);
    memset(&pFrame->data[4], 0, 4);
    return kmd_interface_transmit(pFrame);
}

/** 
 * @Description  : HAL_MOTOR_ENABLE,               //使能电机		pData[1] = 当前位置
 * @param         (uint16_t) node_id
 * @return        (*)
 */
static bool hal_kmd_fsm_motor_enable(hal_frame_struct *pFrame, uint16_t node_id)
{
    uint32_t tOrderId;
    hal_make_frame_id((uint32_t)node_id, (uint32_t)HAL_KMD_MOTOR_CTR, &pFrame->can_id);
    hal_make_order_id((uint32_t)HAL_MOTOR_ENABLE, 0, &tOrderId);
    uint32_to_data(tOrderId, &pFrame->data[0]);
    memset(&pFrame->data[4], 0, 4);
    return kmd_interface_transmit(pFrame);
}

/** 
 * @Description  : HAL_MOTOR_ERROR_REPORT,         //报告错误
 * @param         (uint16_t) node_id
 * @return        (*)
 */
bool hal_kmd_fsm_motor_report_error(hal_frame_struct *pFrame,uint16_t node_id)
{
	uint32_t tOrderId;
	hal_make_frame_id((uint32_t)node_id, (uint32_t)HAL_KMD_MOTOR_CTR, &pFrame->can_id);
	hal_make_order_id((uint32_t)HAL_MOTOR_ERROR_REPORT,0,&tOrderId);
	uint32_to_data(tOrderId,&pFrame->data[0]);
	memset(&pFrame->data[4], 0, 4);
	return kmd_interface_transmit(pFrame);
}

/** 
 * @Description  : HAL_MOTOR_ERROR_RESET,          //复位错误
 * @param         (uint16_t) node_id
 * @return        (*)
 */
static bool hal_kmd_fsm_motor_reset_error(hal_frame_struct *pFrame, uint16_t node_id)
{
    uint32_t tOrderId;
    hal_make_frame_id((uint32_t)node_id, (uint32_t)HAL_KMD_MOTOR_CTR, &pFrame->can_id);
    hal_make_order_id((uint32_t)HAL_MOTOR_ERROR_RESET, 0, &tOrderId);
    uint32_to_data(tOrderId, &pFrame->data[0]);
    memset(&pFrame->data[4], 0, 4);
    return kmd_interface_transmit(pFrame);
}

/** 
 * @Description  : HAL_MOTOR_GET_STAT,             //获取电机状态消息请求
 * @param         (uint16_t) node_id
 * @return        (*)
 */
static bool hal_kmd_fsm_motor_get_state(hal_frame_struct *pFrame, uint16_t node_id)
{
    uint32_t tOrderId;
    hal_make_frame_id((uint32_t)node_id, (uint32_t)HAL_KMD_MOTOR_CTR, &pFrame->can_id);
    hal_make_order_id((uint32_t)HAL_MOTOR_GET_STAT, 0, &tOrderId);
    uint32_to_data(tOrderId, &pFrame->data[0]);
    memset(&pFrame->data[4], 0, 4);
    return kmd_interface_transmit(pFrame);
}

/** 
 * @Description  : HAL_MOTOR_CALIBRATION_START,    //开始校准
 * @param         (uint16_t) node_id
 * @return        (*)
 */
static bool hal_kmd_fsm_motor_cali_start(hal_frame_struct *pFrame, uint16_t node_id)
{
    uint32_t tOrderId;
    hal_make_frame_id((uint32_t)node_id, (uint32_t)HAL_KMD_MOTOR_CTR, &pFrame->can_id);
    hal_make_order_id((uint32_t)HAL_MOTOR_CALIBRATION_START, 0, &tOrderId);
    uint32_to_data(tOrderId, &pFrame->data[0]);
    memset(&pFrame->data[4], 0, 4);
    return kmd_interface_transmit(pFrame);
}

/** 
 * @Description  : HAL_MOTOR_CALIBRATION_ABORT,    //停止校准
 * @param         (uint16_t) node_id
 * @return        (*)
 */
static bool hal_kmd_fsm_motor_cali_stop(hal_frame_struct *pFrame, uint16_t node_id)
{
    uint32_t tOrderId;
    hal_make_frame_id((uint32_t)node_id, (uint32_t)HAL_KMD_MOTOR_CTR, &pFrame->can_id);
    hal_make_order_id((uint32_t)HAL_MOTOR_CALIBRATION_ABORT, 0, &tOrderId);
    uint32_to_data(tOrderId, &pFrame->data[0]);
    memset(&pFrame->data[4], 0, 4);
    return kmd_interface_transmit(pFrame);
}


/** 
 * @Description  : HAL_MOTOR_CONFIGS_UPDATA,        //保存参数
 * @param         (uint16_t) node_id
 * @return        (*)
 */
static bool hal_kmd_fsm_motor_configs_updata(hal_frame_struct *pFrame, uint16_t node_id)
{
    uint32_t tOrderId;
    hal_make_frame_id((uint32_t)node_id, (uint32_t)HAL_KMD_MOTOR_CTR, &pFrame->can_id);
    hal_make_order_id((uint32_t)HAL_MOTOR_CONFIGS_UPDATA, 0, &tOrderId);
    uint32_to_data(tOrderId, &pFrame->data[0]);
    memset(&pFrame->data[4], 0, 4);
    return kmd_interface_transmit(pFrame);
}

/** 
 * @Description  : HAL_MOTOR_CONFIGS_RESET_ALL,	 //复位所有参数
 * @param         (uint16_t) node_id
 * @return        (*)
 */
static bool hal_kmd_fsm_motor_configs_reset(hal_frame_struct *pFrame, uint16_t node_id)
{
    uint32_t tOrderId;
    hal_make_frame_id((uint32_t)node_id, (uint32_t)HAL_KMD_MOTOR_CTR, &pFrame->can_id);
    hal_make_order_id((uint32_t)HAL_MOTOR_CONFIGS_RESET_ALL, 0, &tOrderId);
    uint32_to_data(tOrderId, &pFrame->data[0]);
    memset(&pFrame->data[4], 0, 4);
    return kmd_interface_transmit(pFrame);
}

/** 
 * @Description  : 设置参数,使用前务必安装正确的数据格式进行参数传递
 * @param         (uint16_t) node_id
 * @param         (uint16_t) tOrderCmd
 * @param         (uint8_t) *pData
 * @return        (*)
 */
static bool hal_kmd_user_config_set(hal_frame_struct *pFrame, uint16_t node_id, uint16_t tOrderCmd, uint8_t *pData)
{
    uint32_t tOrderId;
    hal_make_frame_id((uint32_t)node_id, (uint32_t)HAL_KMD_SET_CONFIG, &pFrame->can_id);
    hal_make_order_id((uint32_t)tOrderCmd, 0, &tOrderId);
    uint32_to_data(tOrderId, &pFrame->data[0]);
    memcpy(&pFrame->data[4], pData, 4);
    return kmd_interface_transmit(pFrame);
}

/** 
 * @Description  : 获取参数消息请求
 * @param         (uint16_t) node_id
 * @param         (uint16_t) tOrderCmd
 * @return        (*)
 */
static bool hal_kmd_user_config_get(hal_frame_struct *pFrame, uint16_t node_id, uint16_t tOrderCmd)
{
    uint32_t tOrderId;
    hal_make_frame_id((uint32_t)node_id, (uint32_t)HAL_KMD_GET_CONFIG, &pFrame->can_id);
    hal_make_order_id((uint32_t)tOrderCmd, 0, &tOrderId);
    uint32_to_data(tOrderId, &pFrame->data[0]);
    memset(&pFrame->data[4], 0, 4);
    return kmd_interface_transmit(pFrame);
}

hal_kmd_user_struct tHalKmdUser =
    {
        .hal_kmd_disable_motor_hook = &hal_kmd_disable_motor,//失能电机
        .hal_kmd_set_pisotion_hook = &hal_kmd_set_pisotion,//设置目标位置
        .hal_kmd_set_velocity_hook = &hal_kmd_set_velocity,//设置目标转速
        .hal_kmd_set_current_hook = &hal_kmd_set_current,//设置目标电流
        .hal_kmd_get_position_hook = &hal_kmd_get_position,//目标位置，实际位置消息请求
        .hal_kmd_get_velocity_hook = &hal_kmd_get_velocity,//目标转速，实际转速消息请求
        .hal_kmd_get_current_hook = &hal_kmd_get_current,//目标电流，实际电流消息请求
        .hal_kmd_get_bus_hook = &hal_kmd_get_bus,//总线电压，实际电压消息请求
        .hal_kmd_get_version_hook = &hal_kmd_get_version,//获取电机版本号消息请求
        .kmd_fsm.hal_kmd_fsm_motor_disable_hook = &hal_kmd_fsm_motor_disable,//失能电机
        .kmd_fsm.hal_kmd_fsm_motor_enable_hook = &hal_kmd_fsm_motor_enable,//使能电机
        .kmd_fsm.hal_kmd_fsm_motor_reset_error_hook = &hal_kmd_fsm_motor_reset_error,//复位错误
        .kmd_fsm.hal_kmd_fsm_motor_get_state_hook = &hal_kmd_fsm_motor_get_state,//获取电机状态消息请求
        .kmd_fsm.hal_kmd_fsm_motor_cali_start_hook = &hal_kmd_fsm_motor_cali_start,//开始校准
        .kmd_fsm.hal_kmd_fsm_motor_cali_stop_hook = &hal_kmd_fsm_motor_cali_stop,//停止校准
        .kmd_fsm.hal_kmd_fsm_motor_configs_updata_hook = &hal_kmd_fsm_motor_configs_updata,//保存参数
        .kmd_fsm.hal_kmd_fsm_motor_configs_reset_hook = &hal_kmd_fsm_motor_configs_reset,//复位所有参数
        .kmd_config.hal_kmd_user_config_set_hook = &hal_kmd_user_config_set,//设置参数,使用前务必安装正确的数据格式进行参数传递
        .kmd_config.hal_kmd_user_config_get_hook = &hal_kmd_user_config_get};//获取参数消息请求
