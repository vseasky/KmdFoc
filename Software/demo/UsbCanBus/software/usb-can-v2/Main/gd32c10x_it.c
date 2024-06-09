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
 * @Date         : 2022-07-13 09:39:53
 * @FilePath     : \usb-can-v1\Main\gd32c10x_it.c
 * @Description  :
 */
#include "gd32c10x_it.h"
#include "main.h"

#ifdef KMD_USB_USER
#include "drv_usbd_int.h"
#include "cdc_acm_core.h"
#endif

#include "bsp_systick.h"
#include "bsp_can.h"
#include "bsp_kmd.h"

#ifdef KMD_USB_USER
extern usb_core_driver cdc_acm;
extern uint32_t usbfs_prescaler;
extern void usb_timer_irq(void);
#endif

void NMI_Handler(void)
{
    Error_Handler();
}

void HardFault_Handler(void)
{
    Error_Handler();
}

void MemManage_Handler(void)
{
    Error_Handler();
}
void BusFault_Handler(void)
{
    Error_Handler();
}

void UsageFault_Handler(void)
{
    Error_Handler();
}

void SVC_Handler(void)
{
    Error_Handler();
}

void DebugMon_Handler(void)
{
    Error_Handler();
}

void PendSV_Handler(void)
{
    Error_Handler();
}

void SysTick_Handler(void)
{
    SysTickCnt++;
}

/**
 * @Description  : can0中断服务函数
 * @return        (*)
 */
void CAN0_RX0_IRQHandler(void)
{
    bsp_can0_irq_callback();
}

/**
 * @Description  : can1中断服务函数
 * @return        (*)
 */
void CAN1_RX0_IRQHandler(void)
{
    bsp_can1_irq_callback();
}

/**
 * @Description  : 定时器5中断服务函数
 * @return        (*)
 */
void TIMER5_IRQHandler(void)
{
    static uint32_t ledTick = 0;
    static uint32_t timer5_count = 0;
    if (SET == timer_interrupt_flag_get(TIMER5, TIMER_INT_FLAG_UP))
    {
        // 清除中断标志
        timer_interrupt_flag_clear(TIMER5, TIMER_INT_FLAG_UP);
        // 按键扫描
        key_enc_scanf();
        // LED状态灯控制
        if (bsp_systick_get_ms_since(ledTick) == 250)
        {
            gpio_bit_reset(GPIOB, GPIO_PIN_3 | GPIO_PIN_4);
        }
        else if (bsp_systick_get_ms_since(ledTick) >= 500)
        {
            gpio_bit_set(GPIOB, GPIO_PIN_3 | GPIO_PIN_4);
            ledTick = bsp_systick_get_tick_ms();
        }
    }
}

#ifdef KMD_USB_USER
void TIMER2_IRQHandler(void)
{
    usb_timer_irq();
}

void USBFS_IRQHandler(void)
{
    usbd_isr(&cdc_acm);
    usb_vcp_auto_receive();
}

void USBFS_WKUP_IRQHandler(void)
{
    if (cdc_acm.bp.low_power)
    {

        SystemInit();

        rcu_usb_clock_config(usbfs_prescaler);

        rcu_periph_clock_enable(RCU_USBFS);

        usb_clock_active(&cdc_acm);
    }

    exti_interrupt_flag_clear(EXTI_18);
}
#endif