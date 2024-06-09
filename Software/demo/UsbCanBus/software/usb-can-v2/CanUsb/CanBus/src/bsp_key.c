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
 * @FilePath     : \MDK-ARMe:\KmdFoc\Tools\UsbCanBus\software\CanUsb-V4\CanUsb\CanBus\src\bsp_key.c
 * @Description  :  此案例展示了基于vSeasky自己设计的按键扫描算法的实现案例，通过此按键扫描算法可以用最低的资源占用（不依赖任何延时）实现稳定的按键消抖，
 *                  并计算出按键处于短按还是长按，又或是连续按下的次数。
 */
#include "bsp_key.h"
#include "gd32c10x.h"

mx_key_task_info key_t[MAX_KEY_NUM];
/** 
 * @Description  : 初始化按键所需Gpio
 * @return        (*)
 */
void bsp_key_init(void)
{
	rcu_periph_clock_enable(RCU_GPIOB);
	gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ,GPIO_PIN_1);
	key_t[0].GPIO_Group = GPIOB;
	key_t[0].GPIO_Pin = GPIO_PIN_1;
}

/** 
 * @Description  : 获取按键电平状态，未消抖
 * @param         (mx_key_task_info) key_scan_t
 * @return        (*)
 */
uint8_t key_task_scan(mx_key_task_info key_scan_t)
{
    if (gpio_input_bit_get(key_scan_t.GPIO_Group, key_scan_t.GPIO_Pin) == KEY_TASK_PRESS_LEVEL)
    {
        return KEY_TASK_PRESS_S;
    }
    return KEY_TASK_UPSPRING_S;
}

/** 
 * @Description  : 按键扫描算法，应定时2ms周期调用
 * @return        (*)
 */
void key_enc_scanf(void)
{
	uint8_t key_count;
	uint8_t key[MAX_KEY_NUM];
    for (key_count = 0; key_count < MAX_KEY_NUM; key_count++)
    {
        // 读取按键电平
        key[key_count] = key_task_scan(key_t[key_count]);
        if (key[key_count] == KEY_TASK_PRESS_S)
        {
            // 按键按下计数
            if (key_t[key_count].key_info.time_count < KEY_PRESS_MAX_COUNT)
            {
                key_t[key_count].key_info.time_count++;
            }
        }
        else
        {
            // 按键弹起计数
            if (key_t[key_count].key_info.res_count < KEY_RES_MAX_COUNT)
            {
                key_t[key_count].key_info.res_count++;
            }
        }
        // 到达长按时间
        if (key_t[key_count].key_info.time_count >= KEY_TASK_TIME_L_COUNT)
        {
            if (key_t[key_count].key_info.states == KEY_TASK_PRESS)
            {
                key_t[key_count].key_info.states = KEY_TASK_PRESS_L_TIME;
                key_t[key_count].key_info.click_num = 0;
                key_t[key_count].key_info.click_state_num = 0;
                // 按键复位计数清零
                key_t[key_count].key_info.res_count = 0;
            }
        }
        // 短按时间
        else if (key_t[key_count].key_info.time_count >= KEY_TASK_TIME_P_COUNT)
        {
            // 首次更新按键事件
            if (key_t[key_count].key_info.states == KEY_TASK_UPSPRING)
            {
                key_t[key_count].key_info.states = KEY_TASK_PRESS;
                // 记录按键按下次数
                if (key_t[key_count].key_info.click_num < KEY_MAX_NUM)
                {
                    key_t[key_count].key_info.click_num++;
                }
                // 按键复位计数清零
                key_t[key_count].key_info.res_count = 0;
            }
        }
        // 连续按超时
        if (key_t[key_count].key_info.res_count >= KEY_RES_NUM_COUNT)
        {
            if (key_t[key_count].key_info.click_num != 0)
            {
                key_t[key_count].key_info.click_state_num =
                    key_t[key_count].key_info.click_num;
                key_t[key_count].key_info.click_num = 0;
            }
        }
        // 按键复位弹起
        else if (key_t[key_count].key_info.res_count >= KEY_RES_COUNT)
        {
            if (key_t[key_count].key_info.states != KEY_TASK_UPSPRING)
            {
                key_t[key_count].key_info.states = KEY_TASK_UPSPRING; // 按键弹起复位
                // 按键按下计数清零
                key_t[key_count].key_info.time_count = 0; // 按键计数清零
            }
        }
    }
}