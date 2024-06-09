#include "led_task.h"

#include "cmsis_os.h"
#include "bsp_led.h"
#include "bsp_buzzer.h"

void led_task(void const *pvParameters);

#define RGB_FLOW_COLOR_CHANGE_TIME  1000
#define RGB_FLOW_COLOR_LENGHT   6
//blue-> green(dark)-> red -> blue(dark) -> green(dark) -> red(dark) -> blue
//À¶ -> ÂÌ(Ãð) -> ºì -> À¶(Ãð) -> ÂÌ -> ºì(Ãð) -> À¶ 
uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGHT + 1] = {0xFF0000FF, 0x0000FF00, 0xFFFF0000, 0x000000FF, 0xFF00FF00, 0x00FF0000, 0xFF0000FF};

osThreadId ledTaskHandle;
void led_task_creat(void)
{
  osThreadDef(ledTask,led_task,osPriorityRealtime,0,128);
  ledTaskHandle = osThreadCreate(osThread(ledTask), NULL);
}
void led_task(void const *pvParameters)
{
	uint16_t i, j;
    float delta_alpha, delta_red, delta_green, delta_blue;
    float alpha,red,green,blue;
    uint32_t aRGB;
	while(1)
	{
		for(i = 0; i < RGB_FLOW_COLOR_LENGHT; i++)
        {
            alpha = (RGB_flow_color[i] & 0xFF000000) >> 24;
            red = ((RGB_flow_color[i] & 0x00FF0000) >> 16);
            green = ((RGB_flow_color[i] & 0x0000FF00) >> 8);
            blue = ((RGB_flow_color[i] & 0x000000FF) >> 0);

            delta_alpha = (float)((RGB_flow_color[i + 1] & 0xFF000000) >> 24) - (float)((RGB_flow_color[i] & 0xFF000000) >> 24);
            delta_red = (float)((RGB_flow_color[i + 1] & 0x00FF0000) >> 16) - (float)((RGB_flow_color[i] & 0x00FF0000) >> 16);
            delta_green = (float)((RGB_flow_color[i + 1] & 0x0000FF00) >> 8) - (float)((RGB_flow_color[i] & 0x0000FF00) >> 8);
            delta_blue = (float)((RGB_flow_color[i + 1] & 0x000000FF) >> 0) - (float)((RGB_flow_color[i] & 0x000000FF) >> 0);

            delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
            delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
            delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
            delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;
            for(j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; j++)
            {
                alpha += delta_alpha;
                red += delta_red;
                green += delta_green;
                blue += delta_blue;

                aRGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 | ((uint32_t)(green)) << 8 | ((uint32_t)(blue)) << 0;
                aRGB_led_show(aRGB);
                osDelay(1);
            }
        }
	}

	

}
