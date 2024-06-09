#ifndef __HAL_FOC_STRUCT_H_
#define __HAL_FOC_STRUCT_H_

#include "stdio.h"
#include "stdint.h"
#include "stdbool.h"
#include "hal_foc_enum.h"

#define HAL_OFFSET_LUT_NUM (128)

typedef struct hal_frame
{
    uint32_t can_id;  //数据ID
    uint8_t msg_type; //数据通信方式
    uint8_t can_dlc;
    uint8_t data[8];   //数据内容
    uint16_t data_len; //数据长度
} hal_frame_struct;

typedef enum
{
    KMD_TYPE_INT = 0,
    KMD_TYPE_FLOAT = 1,
} kmd_type_of_data;

typedef union
{
    int value_int;
    float value_float;
} config_data;

typedef struct
{
    //闭环输入输出信息
    struct
    {
        struct
        {
            float position; //闭环目标位置
            float velocity; //闭环目标转速
            float current_; //闭环目标电流
        } target;
        struct
        {
            float position; //输出位置
            float velocity; //输出转速
            float current_; //输出电流
        } realit;
    } __closed_loop;

    //总线电压&电流
    struct
    {
        float v_bus_filt;
        float i_bus_filt;
    } __bus_info;

    // FOC运行状态
    struct
    {
        uint16_t motor_error;
        uint16_t motor_fsm_mode;
        /* data */
    } __motor_states;

    // 校准结果
    struct
    {
        float cali_res;
        float cali_ind;
        int cali_pairs;
        int cali_dir;
        int cali_offset;
        float cali_position;
        uint32_t cali_error;
        config_data __offset_lut[HAL_OFFSET_LUT_NUM];
    } __cali_info;
} tHalFocControl;

typedef struct
{
    // // Motor
    // struct
    // {
    //     int motor_pole_pairs;         // (Auto)
    //     float motor_phase_resistance; // (Auto)
    //     float motor_phase_inductance; // (Auto)
    //     float inertia;                // [A/(turn/s^2)]
    // } __motor;
    // // Encoder
    // struct
    // {
    //     int encoder_type;                   // 编码器类型
    //     int encoder_dir;                    // (Auto)
    //     int encoder_offset;                 // (Auto)	编码器计数和转子电相位之间的偏移
    //     float encoder_offset_float;         // (Auto)	子计数相位对齐偏移
    //     int offset_lut[HAL_OFFSET_LUT_NUM]; // (Auto)
    //     int encoder_pll_bandwidth;          // PLL带宽
    //     int encoder_cpr;                    // 编码器量程 非绝对编码器需要设置
    // } __encoder;
    // // Calib
    // struct
    // {
    //     int calib_valid;         // (Auto)
    //     float calib_current;     // [A]
    //     float calib_max_voltage; // [V]
    // } __calib;
    // // Control
    // struct
    // {
    //     int control_mode;
    //     float current_ramp_rate;    // [A/sec]		  	电流爬升速率
    //     float vel_ramp_rate;        // [(turn/s)/s]   	转速爬升速率
    //     float traj_vel;             // [turn/s]
    //     float traj_accel;           // [(turn/s)/s]
    //     float traj_decel;           // [(turn/s)/s]
    //     float pos_gain;             // [(turn/s)/turn]  位置环增益
    //     float vel_gain;             // [A/(turn/s)]
    //     float vel_integrator_gain;  // [A/((turn/s)*s)]
    //     float vel_limit;            // [turn/s]
    //     float current_limit;        // [A]				电流限制
    //     float current_ctrl_p_gain;  // (Auto)
    //     float current_ctrl_i_gain;  // (Auto)
    //     int current_ctrl_bandwidth; // [rad/s] Current loop bandwidth 100~2000
    // } __control;
    // // Protect
    // struct
    // {
    //     float protect_under_voltage; // [V]
    //     float protect_over_voltage;  // [V]
    //     float protect_over_speed;    // [turn/s]
    //     float protect_over_current;  // [A] 				过流保护
    // } __protect;

    // // message
    // struct
    // {
    //     int can_id;                  // CAN bus ID
    //     int can_timeout_ms;          // CAN bus timeout in xx(min:250) ms 	: 0 Disable
    //     int can_report_heart_ms;     // CAN bus heart   in xx(min:250) ms  	: 0 Disable
    //     int cen_report_heart_choose; // CAN bus heart data
    //     // UART Debug
    //     int uart_debug_ms; // UART Debug in xx(min:20) ms : 0 Disable
    // } __message;
    config_data __config[64];
    uint8_t __version[4];
} tHalFocUserConfig;

typedef struct
{
    const uint16_t node_id;
    tHalFocControl __kmd_ctr;
    tHalFocUserConfig __kmd_user;
	hal_frame_struct  __tx_frame;
	hal_frame_struct  __rx_frame;
} tHalFocInfo;

static inline void hal_make_frame_id(uint32_t pNodeID, uint32_t pCmdId, uint32_t *pFrameID)
{
    // Frame
    // CMD    | nodeID
    // 7 bits | 4 bits
    *pFrameID = (pCmdId << 4) | (pNodeID & 0X0F);
};

static inline void hal_parse_frame_id(uint32_t *pNodeID, uint32_t *pCmdId, uint32_t pFrameID)
{
    // Frame
    // CMD    | nodeID
    // 7 bits | 4 bits
    *pNodeID = (pFrameID & 0x0F);
    *pCmdId = pFrameID >> 4;
};

static inline void hal_make_order_id(uint32_t pCmdId, uint32_t pResult, uint32_t *pOrderId)
{
    // pOrderId
    // pResult    | pCmdId
    // 16 bits    | 16bits
    *pOrderId = (pResult << 16) | (pCmdId & 0XFFFF);
}
static inline void hal_parse_order_id(uint32_t *pCmdId, uint32_t *pResult, uint32_t pOrderId)
{
    // pOrderId
    // pResult    | pCmdId
    // 16 bits    | 16bits
    *pCmdId = pOrderId & 0xFFFF;
    *pResult = pOrderId >> 16;
}

static inline void int_to_data(int val, uint8_t *data)
{
    data[0] = *(((uint8_t *)(&val)) + 0);
    data[1] = *(((uint8_t *)(&val)) + 1);
    data[2] = *(((uint8_t *)(&val)) + 2);
    data[3] = *(((uint8_t *)(&val)) + 3);
}

static inline int data_to_int(uint8_t *data)
{
    int tmp_int;
    *(((uint8_t *)(&tmp_int)) + 0) = data[0];
    *(((uint8_t *)(&tmp_int)) + 1) = data[1];
    *(((uint8_t *)(&tmp_int)) + 2) = data[2];
    *(((uint8_t *)(&tmp_int)) + 3) = data[3];
    return tmp_int;
}

static inline void float_to_data(float val, uint8_t *data)
{
    data[0] = *(((uint8_t *)(&val)) + 0);
    data[1] = *(((uint8_t *)(&val)) + 1);
    data[2] = *(((uint8_t *)(&val)) + 2);
    data[3] = *(((uint8_t *)(&val)) + 3);
}

static inline float data_to_float(uint8_t *data)
{
    float tmp_float;
    *(((uint8_t *)(&tmp_float)) + 0) = data[0];
    *(((uint8_t *)(&tmp_float)) + 1) = data[1];
    *(((uint8_t *)(&tmp_float)) + 2) = data[2];
    *(((uint8_t *)(&tmp_float)) + 3) = data[3];
    return tmp_float;
}

static inline void uint32_to_data(uint32_t val, uint8_t *data)
{
    data[0] = *(((uint8_t *)(&val)) + 0);
    data[1] = *(((uint8_t *)(&val)) + 1);
    data[2] = *(((uint8_t *)(&val)) + 2);
    data[3] = *(((uint8_t *)(&val)) + 3);
}

static inline uint32_t data_to_uint32(uint8_t *data)
{
    uint32_t tmp_uint32;
    *(((uint8_t *)(&tmp_uint32)) + 0) = data[0];
    *(((uint8_t *)(&tmp_uint32)) + 1) = data[1];
    *(((uint8_t *)(&tmp_uint32)) + 2) = data[2];
    *(((uint8_t *)(&tmp_uint32)) + 3) = data[3];
    return tmp_uint32;
}

#endif /* __HAL_FOC_CONFIG_H */
