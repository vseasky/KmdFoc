#include "bsp_can.h"
#include "bsp_interface.h"
#include "hal_foc_interface.h"

#include "gd32c10x.h"

/**
 * @description: CAN初始化->>波特率：1M 采样率86.7%
 * @return {*}
 */
void bsp_can0_init(void)
{
    can_parameter_struct can_parameter;
    can_filter_parameter_struct can_filter;
    /* 启用 CAN 时钟 */
    rcu_periph_clock_enable(RCU_CAN0);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);

    /* 配置 CAN0 GPIO */
    // CAN0 重映射 CAN_RX->GPIO-PB8 CAN_TX->GPIO-PB9
    gpio_pin_remap_config(GPIO_CAN0_FULL_REMAP, DISABLE);
    gpio_pin_remap_config(GPIO_CAN0_PARTIAL_REMAP, DISABLE);
    gpio_pin_remap_config(GPIO_CAN0_PARTIAL_REMAP, ENABLE);
    gpio_bit_reset(GPIOB, GPIO_PIN_8);
    gpio_bit_reset(GPIOB, GPIO_PIN_9);
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_8);   // CAN_RX->GPIO-PB8
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9); // CAN_TX->GPIO-PB9

    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
    can_deinit(CAN0);

    // 重置 CAN 外设
    CAN_CTL(CAN0) |= CAN_CTL_SWRST;
    while ((CAN_CTL(CAN0) & CAN_CTL_SWRST) != 0)
        ; // 复位后复位位设置为零
    while ((CAN_STAT(CAN0) & CAN_STAT_SLPWS) == 0)
        ; // 复位后应处于睡眠模式
    // TTC  时间触发通信  0: 禁用时间触发通信                1: 使能时间触发通信
    can_parameter.time_triggered = DISABLE;
    // ABOR 自动离线恢复  0: 通过软件手动地从离线状态恢复     1: 通过硬件自动的从离线状态恢复
    can_parameter.auto_bus_off_recovery = ENABLE;
    // AWK 自动唤醒       0: 通过软件手动的从睡眠工作模式唤醒 1: 通过硬件自动的从睡眠工作模式唤醒
    can_parameter.auto_wake_up = DISABLE;
    // ARD 自动重传模式禁用        0: 使能自动重发        1: 禁用自动重发
    can_parameter.auto_retrans = DISABLE; // DISABLE;
    // RFOD 禁用接收FIFO满时覆盖   0: 使能接收FIFO满时覆盖    1: 禁用接收FIFO满时覆盖
    can_parameter.rec_fifo_overwrite = DISABLE;
    // TFO 发送FIFO顺序           0: 标识符（Identifier）较小的帧先发送   1: 所有等待发送的邮箱按照先进先出（FIFO）的顺序发送
    can_parameter.trans_fifo_order = ENABLE;
    //通信模式
    can_parameter.working_mode = CAN_NORMAL_MODE;

    // 波特率：1M 采样率86.7%
    /**
     * baudrate     = can_freq/((1+can_psc)*(1+(can_bs1)+(can_bs2)))
     * sample_point = (1+can_bs1)/(1+can_bs1+can_bs2)
     */
    // 波特率：1M 采样率86.7%
    /*
     * The table below illustrates the CANopen bit timing, the resulting maximum network length,
     * and the maximum unterminated cable drop lengths. All CANopen devices must at least support
     * one of the defined bit-rates. Optionally, a CANopen device may support  further bit-rates.
     * The location of the sample point must be as close as possible to 87.5 % of the bit time.
     */
    int can_psc = 4;
    int can_bs1 = 12;
    int can_bs2 = 2;
    int can_sjw = 1;
    can_parameter.prescaler = can_psc;
    can_parameter.resync_jump_width = can_sjw - 1;
    can_parameter.time_segment_1 = can_bs1 - 1;
    can_parameter.time_segment_2 = can_bs2 - 1;

    can_init(CAN0, &can_parameter);

    can_filter.filter_number = 0;
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high = 0x0000;
    can_filter.filter_list_low = 0x0000;
    can_filter.filter_mask_high = 0x0000;
    can_filter.filter_mask_low = 0x0000;
    can_filter.filter_fifo_number = CAN_FIFO0;
    can_filter.filter_enable = ENABLE;
    can_filter_init(&can_filter);

    // /* 初始化过滤器*/
    can1_filter_start_bank(14);

    can_filter_mask_mode_init(0, 0, CAN_EXTENDED_FIFO0, 0);
    can_filter_mask_mode_init(0, 0, CAN_STANDARD_FIFO0, 0);

    /* 配置 CAN0 NVIC */
    nvic_irq_enable(CAN0_RX0_IRQn, 3, 1);
    /* enable 可以接收 FIFO0 非空中断 */
    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE0);
}


/**
 * @description: CAN消息发送函数
 * @return {*}
 */
bool bsp_can0_transmit(hal_frame_struct *tx_frame)
{
    const uint8_t dataLen = 8;
    uint16_t transmit_time_out = 0;
    
    uint8_t mailbox_number = CAN_MAILBOX0;

    /* 选择一个空邮箱 */
    if (CAN_TSTAT_TME0 == (CAN_TSTAT(CAN0) & CAN_TSTAT_TME0))
    {
        mailbox_number = CAN_MAILBOX0;
    }
    else if (CAN_TSTAT_TME1 == (CAN_TSTAT(CAN0) & CAN_TSTAT_TME1))
    {
        mailbox_number = CAN_MAILBOX1;
    }
    else if (CAN_TSTAT_TME2 == (CAN_TSTAT(CAN0) & CAN_TSTAT_TME2))
    {
        mailbox_number = CAN_MAILBOX2;
    }
    else
    {
        mailbox_number = CAN_NOMAILBOX;
    }
    /* 不返回邮箱空 */
    if (CAN_NOMAILBOX == mailbox_number)
    {
        return false;
    }
    /* 设置发送邮箱标准标识符 */
    CAN_TMI(CAN0, mailbox_number) = (uint32_t)(TMI_SFID(tx_frame->can_id & 0x7FF));
    /* 设置数据长度 数据长度固定为 8 */
    CAN_TMP(CAN0, mailbox_number) &= ~(CAN_TMP_DLENC | CAN_TMP_ESI | CAN_TMP_BRS | CAN_TMP_FDF);
    CAN_TMP(CAN0, mailbox_number) |= dataLen & 0x0F;
    /* 设置数据 */
    CAN_TMDATA0(CAN0, mailbox_number) = TMDATA0_DB3(tx_frame->data[3]) |
                                        TMDATA0_DB2(tx_frame->data[2]) |
                                        TMDATA0_DB1(tx_frame->data[1]) |
                                        TMDATA0_DB0(tx_frame->data[0]);
    CAN_TMDATA1(CAN0, mailbox_number) = TMDATA1_DB7(tx_frame->data[7]) |
                                        TMDATA1_DB6(tx_frame->data[6]) |
                                        TMDATA1_DB5(tx_frame->data[5]) |
                                        TMDATA1_DB4(tx_frame->data[4]);

    /* 启用传输 */
    CAN_TMI(CAN0, mailbox_number) |= CAN_TMI_TEN;
    transmit_time_out = 64;
    while((can_transmit_states(CAN0,mailbox_number)==CAN_TRANSMIT_FAILED)&&(transmit_time_out))
    {
        transmit_time_out--;
    }
    return true;
}

/**
 * @description: CAN消息接收数据处理函数
 * @param {bsp_frame} *rx_frame
 * @return {*}
 */
bool bsp_can0_receive(bsp_frame *rx_frame)
{
    if ((CAN_RFIFO0(CAN0) & CAN_RFIF_RFL_MASK) != 0)
    {
        rx_frame->can_id = GET_RFIFOMI_SFID(CAN_RFIFOMI(CAN0, 0));

        rx_frame->can_dlc = (uint8_t)(GET_RFIFOMP_DLENC(CAN_RFIFOMP(CAN0, 0)));

        rx_frame->data[0] = (uint8_t)(GET_RFIFOMDATA0_DB0(CAN_RFIFOMDATA0(CAN0, 0)));
        rx_frame->data[1] = (uint8_t)(GET_RFIFOMDATA0_DB1(CAN_RFIFOMDATA0(CAN0, 0)));
        rx_frame->data[2] = (uint8_t)(GET_RFIFOMDATA0_DB2(CAN_RFIFOMDATA0(CAN0, 0)));
        rx_frame->data[3] = (uint8_t)(GET_RFIFOMDATA0_DB3(CAN_RFIFOMDATA0(CAN0, 0)));
        rx_frame->data[4] = (uint8_t)(GET_RFIFOMDATA1_DB4(CAN_RFIFOMDATA1(CAN0, 0)));
        rx_frame->data[5] = (uint8_t)(GET_RFIFOMDATA1_DB5(CAN_RFIFOMDATA1(CAN0, 0)));
        rx_frame->data[6] = (uint8_t)(GET_RFIFOMDATA1_DB6(CAN_RFIFOMDATA1(CAN0, 0)));
        rx_frame->data[7] = (uint8_t)(GET_RFIFOMDATA1_DB7(CAN_RFIFOMDATA1(CAN0, 0)));

        CAN_RFIFO0(CAN0) |= CAN_RFIFO0_RFD0; // release FIFO

        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @description: can 中断回调
 * @return {*}
 */
void bsp_can0_irq_callback(void)
{
    bool ret = false;
    bsp_frame can0_frame;
    can0_frame.msg_type = KMD_MSG_FRAME_CAN_;
    while (bsp_can0_receive(&can0_frame))
    {
        //在此添加处理函数
        ret = bsp_receive_callback(&can0_frame);
    }
}
