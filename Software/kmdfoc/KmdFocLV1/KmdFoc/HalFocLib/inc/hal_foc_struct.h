#ifndef __HAL_FOC_STRUCT_H_
#define __HAL_FOC_STRUCT_H_

#include "hal_foc_config.h"
#include "hal_foc_enum.h"
#include "hal_foc_interface.h"

typedef struct
{
    int adc_vbus;
    int adc_phase_a, adc_phase_b, adc_phase_c;

    float v_bus, v_bus_filt; // DC link voltage
    float i_a, i_b, i_c;     // Phase currents
    float dtc_a, dtc_b, dtc_c;
    float i_d_filt, i_q_filt, i_bus_filt; // D/Q currents

    float current_ctrl_integral_d, current_ctrl_integral_q; // Current error integrals
    int adc_phase_a_offset;
    int adc_phase_b_offset;
    int adc_phase_c_offset;

    struct
    {
        const float current_filt_alpha;
        const float vbus_filt_alpha;
        const float hal_v_scale;
        const float hal_i_scale;
        const float hal_max_current;
        const uint32_t hal_pwm_arr;
        const float mFocDt;
    } const_define;
    //校准0电流
    struct
    {
        bool zero_current_start;
        bool zero_current_calib_valid;
        uint32_t zero_current_count;
        uint32_t zero_adc_phase_sum_a;
        uint32_t zero_adc_phase_sum_b;
        uint32_t zero_adc_phase_sum_c;
        const uint32_t zero_current_max_count; //零电流校准时间
    } foc_calib_zero_current;
    struct
    {
        float Id_des;
        float Iq_des;
        float I_phase;
        float pwm_phase;
    } foc_calc;

    //指向Usr中的 p_gain
    struct // read-only
    {
        const float *const current_ctrl_p_gain;
        const float *const current_ctrl_i_gain;
        const int *const encoder_dir;
    } usr_config;
    void (*const foc_apply_duty)(uint16_t duty_a, uint16_t duty_b, uint16_t duty_c);
} tHalFocCurrentStruct;

typedef struct
{
    // 电阻测量时间
    struct hal_foc_struct
    {
        const float num_R_ki;       //电阻测量Ki参数
        const float mCalibResTimeS; //电阻测量时间     			2s
        const float mCalibIndTimeS; //电感测量时间				1s
        const float mCalibDirTimeS; //极性校准电流提升时间		2s
        const float mCalibEncTimeS; //编码器校准电流提升时间	2s
        const float mFocDt;         //校准周期，单位s
        const float mCalibMaxPairs; //最大极对数
        const float mCalibSpeed;    //校准转速
        const float mReportTimeS;   //数据上报限制
    } const_define;

    tHalCalibrationStep mCalibStep;   //电机校准状态
    tHalCalibrationError mCalibError; //校准错误状态
    int *p_error_arr;
    // 单次运行时间 1/(HAL_CURRENT_MEASURE_HZ)
    float mCalibTime;
    uint64_t mCalibTimeCount;
    uint64_t reportCount;

    float voltages[2];
    float Ialphas[2];

    int raw_offset;
    int sample_count;
    float next_sample_time;
    float theta_ref;

    float enc_dir_start_count;
    float enc_dir_end_count;

    int AnticoggingValid;
    int AntiCoggingStep;
} tHalCalibStruct;

typedef struct
{
    int raw;
    int cnt;
    int pos_abs_;
    int count_in_cpr_; //绝对计数值
    int shadow_count_; //当前计数值
    float pos_estimate_counts_;
    float vel_estimate_counts_;
    float pos_cpr_counts_;

    float pos_estimate_;
    float vel_estimate_;
    float pos_cpr_;

    float phase_;
    float interpolation_;

    float pll_kp_;
    float pll_ki_;

    uint32_t encoder_cnt; //实际读数

    uint32_t enc_cpr;
    uint32_t enc_cpr_drv_2;
    uint32_t enc_cpr_bit;
    uint32_t enc_cpr_offset;

    // hall encoder
    int32_t hall_delta_enc;
    int32_t hall_count_in_cpr;
    int32_t hall_count;
    bool encoder_valid;
    bool encoder_err_check;
    struct
    {
        const float mFocDt;
    } const_define;

    struct
    {
        int *const encoder_cpr; // rw
        const float *const encoder_offset_float;
        const int *const encoder_type; // read-only
        const int *const encoder_dir;
        const int *const offset_lut[HAL_OFFSET_LUT_NUM];
        const int *const encoder_pll_bandwidth;
        const int *const motor_pole_pairs;
        const int *const calib_valid;
    } usr_config;
} tHalEncoderStruct;
typedef struct sTraj
{
    float Xi_;
    float Xf_;
    float Vi_;

    float Ar_;
    float Vr_;
    float Dr_;

    float Ta_;//加速运行时间
    float Tv_;//匀速运行时间
    float Td_;//减速运行时间
    float Tf_;//过程总时间
    float yAccel_;

    struct
    {
        float Y;
        float Yd;
        float Ydd;
    } trajStep;


    float t;
    bool trajectory_done;
} tHaltrapTraj;

typedef struct
{
    float current_setpoint;
    float phase_vel;
    float phase;
    float pwm_phase;
} tHalFocCtrOut;
typedef struct
{
    struct
    {
        float position; //预设位置
        float velocity; //预设转速
        float current_; //预设电流
    } ready_set;
    struct
    {
        float position; //输入位置
        float velocity; //输入转速
        float current_; //输入电流
    } input;
    struct
    {
        float position; //闭环目标位置
        float velocity; //闭环目标转速
        float current_; //闭环目标电流
    } target;

    tHaltrapTraj mTraj;
    float TrajVel;
    float TrajCurrent;
    float vel_integrator;
    bool input_pos_updated;

    uint32_t mLedTick;
    uint32_t mRecTick;
    uint32_t mHeartbeatTick;
    uint32_t mUartDebugTick;

    //串级PID输出
    tHalFocCtrOut mFocCtrOut;
} tHalFocControl;

typedef struct
{
    // Motor
    int motor_pole_pairs;         // (Auto)
    float motor_phase_resistance; // (Auto)
    float motor_phase_inductance; // (Auto)
    float inertia;                // [A/(turn/s^2)]

    // Encoder
    int encoder_type;                   // 编码器类型
    int encoder_dir;                    // (Auto)
    int encoder_offset;                 // (Auto)	编码器计数和转子电相位之间的偏移
    float encoder_offset_float;         // (Auto)	子计数相位对齐偏移
    int offset_lut[HAL_OFFSET_LUT_NUM]; // (Auto)
    int encoder_pll_bandwidth;          // PLL带宽
    int encoder_cpr;                    // 编码器量程 非绝对编码器需要设置

    // Calib
    int calib_valid;         // (Auto)
    float calib_current;     // [A]
    float calib_max_voltage; // [V]

    // Control
    int control_mode;
    float current_ramp_rate;    // [A/sec]		  	电流爬升速率
    float vel_ramp_rate;        // [(turn/s)/s]   	转速爬升速率
    float traj_vel;             // [turn/s]
    float traj_accel;           // [(turn/s)/s]
    float traj_decel;           // [(turn/s)/s]
    float pos_gain;             // [(turn/s)/turn]  位置环增益
    float vel_gain;             // [A/(turn/s)]
    float vel_integrator_gain;  // [A/((turn/s)*s)]
    float vel_limit;            // [turn/s]
    float current_limit;        // [A]				电流限制
    float current_ctrl_p_gain;  // (Auto)
    float current_ctrl_i_gain;  // (Auto)
    int current_ctrl_bandwidth; // [rad/s] Current loop bandwidth 100~2000

    // Protect
    float protect_under_voltage; // [V]
    float protect_over_voltage;  // [V]
    float protect_over_speed;    // [turn/s]
    float protect_over_current;  // [A] 				过流保护
    // CAN
    int can_id;                  // CAN bus ID
    int can_timeout_ms;          // CAN bus timeout in xx(min:250) ms 	: 0 Disable
    int can_report_heart_ms;     // CAN bus heart   in xx(min:250) ms  	: 0 Disable
    int cen_report_heart_choose; // CAN bus heart data
    // UART Debug
    int uart_debug_ms;           // UART Debug in xx(min:20) ms : 0 Disable

    uint32_t crc;
} tHalUsrConfig;

typedef struct
{
    tHalFsmStat state;
    tHalFsmStat next_state;
    uint8_t ready;
    uint16_t motor_err, motor_err_last;
} tHalFocFsm;

typedef struct
{
    tHalFocCurrentStruct mFocCurrent;
    tHalEncoderStruct mFocEncoder;
    tHalCalibStruct mFocCalib;
    tHalFocControl mFocControl;
    tHalFocFsm mFocFsm;
} tHalFocCoreStruct;

typedef struct
{
    tHalUsrConfig *const pUsrConfig;      //可保存参数
    const uint32_t mUsrConfigAddress;     // pUsrConfig参数保存地址
    tHalFocInterface *const pCallbackFun; //硬件函数接口
    uint32_t report_heart_cnt;            //心跳数据控制
} tHalFocUser;

typedef struct
{
    tHalFocCoreStruct   mFocCore;   //核心参数
    tHalFocUser         mFocUser;   //用户接口
    struct
    {
        const float mFocDt;
        const uint16_t version[4];
    } const_define;
} tHalFocStruct;

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


#endif /* __HAL_FOC_CONFIG_H */
