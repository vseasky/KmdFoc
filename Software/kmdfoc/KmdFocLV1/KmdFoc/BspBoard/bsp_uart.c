#include "bsp_uart.h"
#include "bsp_kmd.h"

#include "main.h"

#define USART2_TX_BUFF_SIZE 128U
#define USART2_RX_BUFF_SIZE 128U

#define MX_UART2_BAUDRATE (256000)
#define USART2_DATA_ADDRESS ((uint32_t)&USART_DATA(USART2))

// 分配串口收发所需内存
uint8_t UartRxBuffer[USART2_RX_BUFF_SIZE];
uint8_t UartTxBuffer[USART2_TX_BUFF_SIZE];

/**
 * @description: 初始化串口
 * @return {*}
 */
void bsp_uart_init(void)
{
    /* 初始化时钟 */
    rcu_periph_clock_enable(RCU_AF);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_USART2);
    /* 设置串口 Tx */
    gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
    /* 设置串口 Rx */
    gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_11);

    /* 串口配置 */
    usart_deinit(USART2);
    usart_word_length_set(USART2, USART_WL_8BIT);
    usart_stop_bit_set(USART2, USART_STB_2BIT);
    usart_baudrate_set(USART2, MX_UART2_BAUDRATE);
    usart_parity_config(USART2, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART2, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART2, USART_CTS_DISABLE);
    usart_receive_config(USART2, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);

    nvic_irq_enable(USART2_IRQn, 3, 1);

    usart_interrupt_enable(USART2, USART_INT_IDLE);

    usart_enable(USART2);
}

/**
 * @description: 初始化串口所需dma
 * @return {*}
 */
void bsp_dma_init(void)
{
    // USART0_Tx DMA0 通道3
    // USART0_Rx DMA0 通道4
    dma_parameter_struct dma_init_struct;
    /* 初始化DMA0 */
    rcu_periph_clock_enable(RCU_DMA0);

    /* 失能DMA0 ->DMA_CH1->USART2_Tx*/
    dma_deinit(DMA0, DMA_CH1);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_addr = NULL;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = USART2_TX_BUFF_SIZE;
    dma_init_struct.periph_addr = USART2_DATA_ADDRESS;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH1, &dma_init_struct);

    /* 失能DMA0 ->DMA_CH2->USART2_Rx*/
    dma_deinit(DMA0, DMA_CH2);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    dma_init_struct.memory_addr = (uint32_t)UartRxBuffer;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.number = USART2_RX_BUFF_SIZE;
    dma_init_struct.periph_addr = USART2_DATA_ADDRESS;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA0, DMA_CH2, &dma_init_struct);

    /* 配置DMA */
    dma_circulation_disable(DMA0, DMA_CH1);
    dma_memory_to_memory_disable(DMA0, DMA_CH1);
    dma_circulation_enable(DMA0, DMA_CH2);
    dma_memory_to_memory_disable(DMA0, DMA_CH2);

    /* USART DMA0 启用接收 */
    usart_dma_receive_config(USART2, USART_DENR_ENABLE);
    /* 启用 DMA0 通道 2 传输完全中断 */
    // dma_interrupt_enable(DMA0, DMA_CH2, DMA_INT_FTF);
    /* 启用 DMA0 通道2 */
    dma_channel_enable(DMA0, DMA_CH2);
    /* USART DMA0 使能传输 */
    usart_dma_transmit_config(USART2, USART_DENT_ENABLE);
}

/**
 * @description: uart 就收函数
 * @param {uint8_t} *rxData
 * @param {uint8_t} this_rx_buff_len
 * @return {*}
 */
bool bsp_uart_receive(uint8_t *rxData, uint8_t this_rx_buff_len)
{
    return bsp_kmd_callback(rxData, this_rx_buff_len, KMD_MSG_FRAME_UART);
}

/**
 * @description: uart 发送所需函数
 * @param {uint8_t} *pData
 * @param {uint16_t} len
 * @return {*}
 */
bool bsp_uart_transmit(uint8_t *pData, uint16_t len)
{
    uint16_t transmit_time_out = 0;
    DMA_CHCTL(DMA0, DMA_CH1) &= ~DMA_CHXCTL_CHEN;
    DMA_CHMADDR(DMA0, DMA_CH1) = (uint32_t)pData;
    DMA_CHCNT(DMA0, DMA_CH1) = (len & DMA_CHANNEL_CNT_MASK);
    DMA_CHCTL(DMA0, DMA_CH1) |= DMA_CHXCTL_CHEN;
    /* configure DMA transmission */
    USART_CTL2(USART2) = (USART_CTL2(USART2) & (~USART_CTL2_DENT)) | (USART_DENT_ENABLE);
    // 等待发送完成，如果使用，这段时间就浪费了，DMA发送是自动进行的，CPU空等待->>为非有效空闲
    // 在HAL层已经做了发送频率控制，因此调用后，无需等待，CPU可以更快的恢复有效空闲，进而可以更高效的处理FOC计算，以及减小FOC计算因为CPU资源占用而出错概率
//    while(RESET == dma_flag_get(DMA0,DMA_CH1, DMA_FLAG_FTF));
    return true;
}

/**
 * @description: uart 中断回调
 * @return {*}
 */
void bsp_uart_irq_callback(void)
{
    bool ret = false;
    static uint8_t this_rx_buff_len;
    if (usart_interrupt_flag_get(USART2, USART_INT_FLAG_IDLE))
    {
        // 清除接收完成标志位
        (uint16_t)(GET_BITS(USART_DATA(USART2), 0U, 8U));
        // 关闭DMA传输
        DMA_CHCTL(DMA0, DMA_CH2) &= ~DMA_CHXCTL_CHEN;
        // 获取此次接收长度
        this_rx_buff_len = USART2_RX_BUFF_SIZE - (uint32_t)DMA_CHCNT(DMA0, DMA_CH2);
        ret = bsp_uart_receive(UartRxBuffer, this_rx_buff_len);
        // 重新设置DMA传输
        DMA_CHCNT(DMA0, DMA_CH2) = (USART2_RX_BUFF_SIZE & DMA_CHANNEL_CNT_MASK);
        // 开启DMA传输
        DMA_CHCTL(DMA0, DMA_CH2) |= DMA_CHXCTL_CHEN;
    }
}
