#include "gd32c10x_it.h"
#include "main.h"

#ifdef KMD_USB_USER
#include "drv_usbd_int.h"
#include "cdc_acm_core.h"
#endif

#include "hal_foc_current.h"
#include "hal_foc_fsm.h"
#include "bsp_encoder.h"
#include "bsp_systick.h"
#include "bsp_svpwm.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "bsp_svpwm.h"
#include "bsp_gpio.h"
#include "bsp_kmd.h"
#include "bsp_uart.h"

#ifdef KMD_USB_USER
extern usb_core_driver cdc_acm;
extern uint32_t usbfs_prescaler;
#endif
extern void usb_timer_irq (void);


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

void CAN0_RX0_IRQHandler(void)
{
	bsp_can0_irq_callback();
}

void USART2_IRQHandler(void)
{
    bsp_uart_irq_callback();
}

void ADC0_1_IRQHandler(void)
{
    bsp_svpwm_irq_callback();
}

#ifdef KMD_USB_USER
void TIMER2_IRQHandler(void)
{
    usb_timer_irq();
}

void  USBFS_IRQHandler (void)
{
    usbd_isr(&cdc_acm);
}

/*!
    \brief      this function handles USBFS wakeup interrupt request.
    \param[in]  none
    \param[out] none
    \retval     none
*/
void USBFS_WKUP_IRQHandler(void)
{
    if (cdc_acm.bp.low_power) {

        SystemInit();

        rcu_usb_clock_config(usbfs_prescaler);

        rcu_periph_clock_enable(RCU_USBFS);

        usb_clock_active(&cdc_acm);
    }

    exti_interrupt_flag_clear(EXTI_18);
}
#endif
