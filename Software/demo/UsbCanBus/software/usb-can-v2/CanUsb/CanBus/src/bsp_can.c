/**
 * @License      :
 * @
 * @	<one line to give the program's name and a brief idea of what it does.>
 * @	Copyright (C) 2022  vSeasky.Liu liuwei_seasky@163.com
 * @	This program is free software: you can redistribute it and/or modify
 * @	it under the terms of the GNU General Public License as published by
 * @	the Free Software Foundation, either version 3 of the License, or
 * @	(at your option) any later version.
 * @
 * @	This program is distributed in the hope that it will be useful,
 * @	but WITHOUT ANY WARRANTY; without even the implied warranty of
 * @	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * @	GNU General Public License for more details.
 * @
 * @	You should have received a copy of the GNU General Public License
 * @	along with this program.  If not, see <https://www.gnu.org/licenses/>.
 * @
 * @Author       : Copyright (c) 2022, vSeasky.Liu liuwei_seasky@163.com.
 * @Github       : https://github.com/SEASKY-Master
 * @Date         : 2022-05-27 14:01:11
 * @FilePath     : \MDK-ARMe:\KmdFoc\Tools\UsbCanBus\software\CanUsb-V4\CanUsb\CanBus\src\bsp_can.c
 * @Description  :
 */
#include "bsp_can.h"
#include "bsp_kmd.h"
#include "hal_kmd_interface.h"
/**
 * @Description  : can0 初始化
 * @return        (*)
 */
void bsp_can0_init(void)
{
    can_parameter_struct can_parameter;
    can_filter_parameter_struct can_filter;
    /* 启用 CAN 时钟 */
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_CAN0);
    /* 配置 CAN0 GPIO */
    gpio_pin_remap_config(GPIO_CAN0_PARTIAL_REMAP, ENABLE);
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_8);   // CAN_RX
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9); // CAN_TX

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

    /*
     * The table below illustrates the CANopen bit timing, the resulting maximum network length,
     * and the maximum unterminated cable drop lengths. All CANopen devices must at least support
     * one of the defined bit-rates. Optionally, a CANopen device may support  further bit-rates.
     * The location of the sample point must be as close as possible to 87.5 % of the bit time.
     */
    // 波特率：1M 采样率86.7%
    /**
     * baudrate     = can_freq/((1+can_psc)*(1+(can_bs1)+(can_bs2)))
     * sample_point = (1+can_bs1)/(1+can_bs1+can_bs2)
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
    can1_filter_start_bank(14);
    /* 初始化过滤器 */
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

    /* 配置 CAN0 NVIC */
    nvic_irq_enable(CAN0_RX0_IRQn, 2, 0);
    /* enable 可以接收 FIFO0 非空中断 */
    can_interrupt_enable(CAN0, CAN_INTEN_RFNEIE0);
}

/**
 * @Description  : can1 初始化
 * @return        (*)
 */
void bsp_can1_init(void)
{
    can_parameter_struct can_parameter;
    can_filter_parameter_struct can_filter;
    /* 启用 CAN 时钟 */
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_CAN1);

    /* 配置 CAN0 GPIO */
    gpio_pin_remap_config(GPIO_CAN1_REMAP, DISABLE);
    gpio_pin_remap_config(GPIO_CAN1_REMAP, ENABLE);
    gpio_bit_reset(GPIOB, GPIO_PIN_5);
    gpio_bit_reset(GPIOB, GPIO_PIN_6);
    gpio_init(GPIOB, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, GPIO_PIN_5);   // CAN_RX
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_6); // CAN_TX
                                                                      // 0->没有重映射 (CAN1_RX/PB12,CAN1_TX/PB13)
    // 1->重映射 	(CAN1_RX/PB5, CAN1_TX/PB6)

    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
    can_deinit(CAN1);

    // 重置 CAN 外设
    CAN_CTL(CAN1) |= CAN_CTL_SWRST;
    while ((CAN_CTL(CAN1) & CAN_CTL_SWRST) != 0)
        ; // 复位后复位位设置为零
    while ((CAN_STAT(CAN1) & CAN_STAT_SLPWS) == 0)
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

    /*
     * The table below illustrates the CANopen bit timing, the resulting maximum network length,
     * and the maximum unterminated cable drop lengths. All CANopen devices must at least support
     * one of the defined bit-rates. Optionally, a CANopen device may support  further bit-rates.
     * The location of the sample point must be as close as possible to 87.5 % of the bit time.
     */
    // 波特率：1M 采样率86.7%
    /**
     * baudrate     = can_freq/((1+can_psc)*(1+(can_bs1)+(can_bs2)))
     * sample_point = (1+can_bs1)/(1+can_bs1+can_bs2)
     */
    int can_psc = 4;
    int can_bs1 = 12;
    int can_bs2 = 2;
    int can_sjw = 1;
    can_parameter.prescaler = can_psc;
    can_parameter.resync_jump_width = can_sjw - 1;
    can_parameter.time_segment_1 = can_bs1 - 1;
    can_parameter.time_segment_2 = can_bs2 - 1;

    can_init(CAN1, &can_parameter);
    can1_filter_start_bank(14);
    /* 初始化过滤器 */
    can_filter.filter_number = 15;
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high = 0x0000;
    can_filter.filter_list_low = 0x0000;
    can_filter.filter_mask_high = 0x0000;
    can_filter.filter_mask_low = 0x0000;
    can_filter.filter_fifo_number = CAN_FIFO0;
    can_filter.filter_enable = ENABLE;
    can_filter_init(&can_filter);

    /* 配置 CAN0 NVIC */
    nvic_irq_enable(CAN1_RX0_IRQn, 2, 1);
    /* enable 可以接收 FIFO0 非空中断 */
    can_interrupt_enable(CAN1, CAN_INTEN_RFNEIE0);
}

/**
 * @Description  : can0 发送函数
 * @param         (uint32_t) frameID
 * @param         (uint8_t) *pData
 * @param         (uint8_t) len
 * @return        (*)
 */
bool bsp_can0_transmit(uint32_t frameID, uint8_t *pData, uint8_t len)
{
    uint8_t mailbox_number = CAN_MAILBOX0;
    uint16_t transmit_time_out = 0;
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
    CAN_TMI(CAN0, mailbox_number) = (uint32_t)(TMI_SFID(frameID & 0x7FF));

    /* 设置数据长度 */
    CAN_TMP(CAN0, mailbox_number) &= ~(CAN_TMP_DLENC | CAN_TMP_ESI | CAN_TMP_BRS | CAN_TMP_FDF);
    CAN_TMP(CAN0, mailbox_number) |= len & 0x0F;

    /* 设置数据 */
    CAN_TMDATA0(CAN0, mailbox_number) = TMDATA0_DB3(pData[3]) |
                                        TMDATA0_DB2(pData[2]) |
                                        TMDATA0_DB1(pData[1]) |
                                        TMDATA0_DB0(pData[0]);
    CAN_TMDATA1(CAN0, mailbox_number) = TMDATA1_DB7(pData[7]) |
                                        TMDATA1_DB6(pData[6]) |
                                        TMDATA1_DB5(pData[5]) |
                                        TMDATA1_DB4(pData[4]);

    /* 启用传输 */
    CAN_TMI(CAN0, mailbox_number) |= CAN_TMI_TEN;
    transmit_time_out = 1000;
    while((can_transmit_states(CAN0,mailbox_number)==CAN_TRANSMIT_FAILED)&&(transmit_time_out))transmit_time_out--;
    return true;
}

/* can 接收函数 */
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
 * @Description  : can1 发送函数
 * @param         (uint32_t) frameID
 * @param         (uint8_t) *pData
 * @param         (uint8_t) len
 * @return        (*)
 */
bool bsp_can1_transmit(uint32_t frameID, uint8_t *pData, uint8_t len)
{
    uint8_t mailbox_number = CAN_MAILBOX0;
    uint16_t transmit_time_out = 0;
    /* 选择一个空邮箱 */
    if (CAN_TSTAT_TME0 == (CAN_TSTAT(CAN1) & CAN_TSTAT_TME0))
    {
        mailbox_number = CAN_MAILBOX0;
    }
    else if (CAN_TSTAT_TME1 == (CAN_TSTAT(CAN1) & CAN_TSTAT_TME1))
    {
        mailbox_number = CAN_MAILBOX1;
    }
    else if (CAN_TSTAT_TME2 == (CAN_TSTAT(CAN1) & CAN_TSTAT_TME2))
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
    CAN_TMI(CAN1, mailbox_number) = (uint32_t)(TMI_SFID(frameID & 0x7FF));

    /* 设置数据长度 */
    CAN_TMP(CAN1, mailbox_number) &= ~(CAN_TMP_DLENC | CAN_TMP_ESI | CAN_TMP_BRS | CAN_TMP_FDF);
    CAN_TMP(CAN1, mailbox_number) |= len & 0x0F;

    /* 设置数据 */
    CAN_TMDATA0(CAN1, mailbox_number) = TMDATA0_DB3(pData[3]) |
                                        TMDATA0_DB2(pData[2]) |
                                        TMDATA0_DB1(pData[1]) |
                                        TMDATA0_DB0(pData[0]);
    CAN_TMDATA1(CAN1, mailbox_number) = TMDATA1_DB7(pData[7]) |
                                        TMDATA1_DB6(pData[6]) |
                                        TMDATA1_DB5(pData[5]) |
                                        TMDATA1_DB4(pData[4]);

    /* 启用传输 */
    CAN_TMI(CAN1, mailbox_number) |= CAN_TMI_TEN;
    transmit_time_out = 1000;
    while((can_transmit_states(CAN1,mailbox_number)==CAN_TRANSMIT_FAILED)&&(transmit_time_out))transmit_time_out--;
    return true;
}

/* can 接收函数 */
bool bsp_can1_receive(bsp_frame *rx_frame)
{
    if ((CAN_RFIFO0(CAN1) & CAN_RFIF_RFL_MASK) != 0)
    {
        rx_frame->can_id = GET_RFIFOMI_SFID(CAN_RFIFOMI(CAN1, 0));

        rx_frame->can_dlc = (uint8_t)(GET_RFIFOMP_DLENC(CAN_RFIFOMP(CAN1, 0)));

        rx_frame->data[0] = (uint8_t)(GET_RFIFOMDATA0_DB0(CAN_RFIFOMDATA0(CAN1, 0)));
        rx_frame->data[1] = (uint8_t)(GET_RFIFOMDATA0_DB1(CAN_RFIFOMDATA0(CAN1, 0)));
        rx_frame->data[2] = (uint8_t)(GET_RFIFOMDATA0_DB2(CAN_RFIFOMDATA0(CAN1, 0)));
        rx_frame->data[3] = (uint8_t)(GET_RFIFOMDATA0_DB3(CAN_RFIFOMDATA0(CAN1, 0)));
        rx_frame->data[4] = (uint8_t)(GET_RFIFOMDATA1_DB4(CAN_RFIFOMDATA1(CAN1, 0)));
        rx_frame->data[5] = (uint8_t)(GET_RFIFOMDATA1_DB5(CAN_RFIFOMDATA1(CAN1, 0)));
        rx_frame->data[6] = (uint8_t)(GET_RFIFOMDATA1_DB6(CAN_RFIFOMDATA1(CAN1, 0)));
        rx_frame->data[7] = (uint8_t)(GET_RFIFOMDATA1_DB7(CAN_RFIFOMDATA1(CAN1, 0)));

        CAN_RFIFO0(CAN1) |= CAN_RFIFO0_RFD0; // release FIFO

        return true;
    }
    else
    {
        return false;
    }
}

/**
 * @Description  : can0 中断回调
 * @return        (*)
 */
void bsp_can0_irq_callback(void)
{
    bsp_frame frame0;
    while (bsp_can0_receive(&frame0))
    {
        // 在此添加处理函数
#if (defined(KMD_FOC_USB_CAN)) && (KMD_FOC_USB_CAN == 0)
        // 转换为USB通信
        frame0.msg_type = KMD_MSG_FRAME_USB_;
        bsp_can_usb_transmit(&frame0);
#else
        // 在此添加处理函数
        frame0.msg_type = KMD_MSG_FRAME_CAN_;
        kmd_interface_receive_callback(&frame0);
#endif
    }
}

/**
 * @Description  : can1 中断回调
 * @return        (*)
 */
void bsp_can1_irq_callback(void)
{
    bsp_frame frame1;
    while (bsp_can1_receive(&frame1))
    {
#if (defined(KMD_FOC_USB_CAN)) && (KMD_FOC_USB_CAN == 1)
        // 转换为USB通信
        frame1.msg_type = KMD_MSG_FRAME_USB_;
        bsp_can_usb_transmit(&frame1);
#else
        // 在此添加处理函数
        frame1.msg_type = KMD_MSG_FRAME_CAN_;
        kmd_interface_receive_callback(&frame1);
#endif
    }
}

