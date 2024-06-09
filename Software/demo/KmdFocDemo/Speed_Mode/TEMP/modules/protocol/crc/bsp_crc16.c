/**
  ****************************(C) COPYRIGHT 2022 Seasky****************************
  * @file       bsp_crc16.c/h
  * @brief      bsp_crc16ʵ�ֺ�����������ʼ�������㺯����
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V0.0.1     3-23-2022     	Liuwei          1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2021 Seasky****************************
  */

#include "bsp_crc16.h"

static uint8_t crc_tab16_init = 0;
static uint16_t crc_tab16[256];

/// <summary>
/// ����crc_16()һ�μ���һ���ֽڵ�16λCRC16
/// </summary>
/// <param name="input_str">�ַ���</param>
/// <param name="num_bytes">�ֽ���</param>
/// <returns></returns>
uint16_t crc_16(const uint8_t *input_str, uint16_t num_bytes)
{
    uint16_t crc;
    const uint8_t *ptr;
    uint16_t a;
    if (!crc_tab16_init)
        init_crc16_tab();
    crc = CRC_START_16;
    ptr = input_str;
    if (ptr != NULL)
        for (a = 0; a < num_bytes; a++)
            {
                crc = (crc >> 8) ^ crc_tab16[(crc ^ (uint16_t)*ptr++) & 0x00FF];
            }
    return crc;
}

/// <summary>
/// һ�μ���16λmodbusѭ������У��
/// </summary>
/// <param name="input_str">�ַ���</param>
/// <param name="num_bytes">�ֽ���</param>
/// <returns></returns>
uint16_t crc_modbus(const uint8_t *input_str, uint16_t num_bytes)
{
    uint16_t crc;
    const uint8_t *ptr;
    uint16_t a;

    if (!crc_tab16_init)
        init_crc16_tab();

    crc = CRC_START_MODBUS;
    ptr = input_str;
    if (ptr != NULL)
        for (a = 0; a < num_bytes; a++)
            {

                crc = (crc >> 8) ^ crc_tab16[(crc ^ (uint16_t)*ptr++) & 0x00FF];
            }
    return crc;
}

/// <summary>
/// ǰһ��ѭ������У��ֵ����һ��Ҫ���������ֽڡ�
/// </summary>
/// <param name="crc"></param>
/// <param name="c"></param>
/// <returns></returns>
uint16_t update_crc_16(uint16_t crc, uint8_t c)
{
    if (!crc_tab16_init)
        init_crc16_tab();
    return (crc >> 8) ^ crc_tab16[(crc ^ (uint16_t)c) & 0x00FF];
}

/// <summary>
/// Ϊ�˻��������ܣ�ʹ��CRC16���̲��Ҵ���ֵ�ı�
/// </summary>
/// <param name=""></param>
void init_crc16_tab(void)
{
    uint16_t i;
    uint16_t j;
    uint16_t crc;
    uint16_t c;
    if (crc_tab16_init == 0)
        {
            for (i = 0; i < 256; i++)
                {
                    crc = 0;
                    c = i;
                    for (j = 0; j < 8; j++)
                        {
                            if ((crc ^ c) & 0x0001)
                                crc = (crc >> 1) ^ CRC_POLY_16;
                            else
                                crc = crc >> 1;
                            c = c >> 1;
                        }
                    crc_tab16[i] = crc;
                }
            crc_tab16_init = 1;
        }
}