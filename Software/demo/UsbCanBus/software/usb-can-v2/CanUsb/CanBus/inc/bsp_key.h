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
 * @FilePath     : \MDK-ARMe:\KmdFoc\Tools\UsbCanBus\software\CanUsb-V4\CanUsb\CanBus\inc\bsp_key.h
 * @Description  : 
 */
#ifndef _BSP_KEY_H_
#define _BSP_KEY_H_

#include "stdio.h"
#include "stdint.h"

#define MAX_KEY_NUM 1 //按键个数
#define KEY_TASK_TIME_CYCLE   2   //单位ms
#define KEY_TASK_TIME_P_COUNT 10*2  // TIME_P_TIME/TIME_CYCLE	20ms
#define KEY_TASK_TIME_L_COUNT 750*2 // TIME_L_TIME/TIME_CYCLE	1500ms

#define KEY_PRESS_MAX_COUNT 2000*2 //按下最长计时	4s
#define KEY_RES_MAX_COUNT 	2000*2 //复位最长计时	4s
#define KEY_RES_COUNT 		6*2    //按键复位弹起计数时间		12ms
#define KEY_RES_NUM_COUNT 	250*2  //按键计数停止计数时间		500ms
#define KEY_MAX_NUM 100

#define KEY_TASK_PRESS_S 1    //按键按下返回信号
#define KEY_TASK_UPSPRING_S 0 //按键弹起返回信号

#define KEY_TASK_PRESS_LEVEL 0 //定义按键按下时的电平

typedef enum
{
	KEY_TASK_UPSPRING = 0,     //按键弹起
	KEY_TASK_PRESS,        	   //按键按下
	KEY_TASK_PRESS_L_TIME,     //按键长按
} key_task_states;
typedef struct
{
	key_task_states states;  //按键状态
	uint8_t click_state_num; //连续短按次数
	uint16_t time_count;     //按键按下时间计数
	uint16_t res_count;      //按键弹起时间计数
	uint8_t click_num;       //按键状态为短按计数
} key_task_time_info;
typedef struct
{
	uint32_t GPIO_Group;
	uint32_t GPIO_Pin;
	uint16_t key_gpio_pin;       //定义的按键GPIO
	key_task_time_info key_info; //按键算法信息
} mx_key_task_info;

/** 
 * @Description  : 初始化按键所需Gpio
 * @return        (*)
 */	
void bsp_key_init(void);


/** 
 * @Description  : 按键扫描算法，应定时2ms周期调用
 * @return        (*)
 */
void key_enc_scanf(void);
#endif