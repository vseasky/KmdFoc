#include "bsp_systick.h"

#define SYSTICK_US_TICK         (SystemCoreClock / 1000000U)
#define SYSTICK_MS_TICK         (SystemCoreClock / 1000U)
  

volatile uint64_t SysTickCnt = 0;

/**
 * @description: 初始化Systick
 * @return {*}
 */
void bsp_systick_init(void)
{
    // 为 1000Hz 中断设置 systick 定时器
    if (SysTick_Config(SYSTICK_MS_TICK))
    {
        /* capture error */
        while (1)
        {
        }
    }
    // 配置 systick 处理程序优先级
    NVIC_SetPriority(SysTick_IRQn, 0x02U);

    /* ----------------------------------------------------------------------------
    TIMER1 Configuration:
    TIMER1CLK = SystemCoreClock/120 = 1MHz
    ---------------------------------------------------------------------------- */
    timer_parameter_struct timer_initpara;
    rcu_periph_clock_enable(RCU_TIMER1);
    timer_deinit(TIMER1);
    // 初始化 TIMER 初始化参数结构
    timer_struct_para_init(&timer_initpara);
    // TIMER1 配置
    timer_initpara.prescaler = 120 - 1;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = 0xFFFF;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_init(TIMER1, &timer_initpara);

    timer_enable(TIMER1);
}


/**
 * @description: 获取系统时间us
 * @return {*}
 */
uint64_t bsp_systick_get_tick_us(void)
{
	uint64_t vSysTick,vSysTickUs;
	vSysTick = SysTick->LOAD+1 - SysTick->VAL;
	vSysTickUs = (uint64_t)(vSysTick/SYSTICK_US_TICK+SysTickCnt*1000);
	return vSysTickUs;
}


/**
 * @description: 获取系统时间差us
 * @param {uint64_t} tick_us
 * @return {*}
 */
uint64_t bsp_systick_get_us_since(uint64_t tick_us)
{
    return (bsp_systick_get_tick_us()-tick_us);
}


/**
 * @description: 获取系统时间ms
 * @return {*}
 */
uint64_t bsp_systick_get_tick_ms(void)
{
    return SysTickCnt;
}

/**
 * @description: 计算系统时间差ms
 * @param {uint32_t} tick
 * @return {*}
 */
uint64_t bsp_systick_get_ms_since(uint64_t tick_ms)
{
    return (uint64_t)(SysTickCnt - tick_ms);
}

/**
 * @description: systick us延时函数
 * @param {uint32_t} uS
 * @return {*}
 */
void bsp_systick_delay_us(uint64_t us)
{
    uint64_t elapsed = 0;
    uint64_t last_count = SysTick->VAL;
    for (;;)
    {
        uint64_t current_count = SysTick->VAL;
        uint64_t elapsed_uS;
        // 测量自我们上次检查以来经过的时间
        if (last_count > current_count)
        {
            elapsed += last_count - current_count;
        }
        else if (last_count < current_count)
        {
            elapsed += last_count + (SYSTICK_MS_TICK - current_count);
        }
        last_count = current_count;
        // 转换为微秒
        elapsed_uS = elapsed / SYSTICK_US_TICK;
        if (elapsed_uS >= us)
        {
            break;
        }
        // 通过经过的时间减少延迟
        us -= elapsed_uS;
        // 为下一次迭代保留小数微秒
        elapsed %= SYSTICK_US_TICK;
    }
}

/**
 * @description: ms延时函数
 * @param {uint32_t} mS
 * @return {*}
 */
void bsp_systick_delay_ms(uint64_t ms)
{
    while (ms--)
    {
        bsp_systick_delay_us(1000);
    }
}
