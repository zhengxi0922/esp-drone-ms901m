/**
 ****************************************************************************************************
 * @file        demo.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2022-06-21
 * @brief       ATK-MS901M模块测试实验
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 STM32F103开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "demo.h"
#include "./BSP/ATK_MS901M/atk_ms901m.h"
#include "./SYSTEM/usart/usart.h"
#include "./BSP/LCD/lcd.h"
#include "./BSP/key/key.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"

/**
 * @brief       LCD UI初始化
 * @param       无
 * @retval      无
 */
static void demo_lcd_ui_init(void)
{
    lcd_show_string(20, 130, 200, 16, 16, "Roll       :   0.00'", BLUE);
    lcd_show_string(20, 151, 200, 16, 16, "Pitch      :   0.00'", BLUE);
    lcd_show_string(20, 172, 200, 16, 16, "Yaw        :   0.00'", BLUE);
    lcd_show_string(20, 193, 200, 16, 16, "Pressure   :      0Pa", BLUE);
    lcd_show_string(20, 214, 200, 16, 16, "Altitude   :      0cm", BLUE);
    lcd_show_string(20, 235, 200, 16, 16, "temperature:   0.00'", BLUE);
}

/**
 * @brief       LCD显示相关数据信息
 * @param       无
 * @retval      无
 */
static void demo_show_data_lcd(atk_ms901m_attitude_data_t *attitude_dat, atk_ms901m_barometer_data_t *barometer_dat)
{
    int32_t tmp;
    
    /* 横滚角 */
    tmp = attitude_dat->roll * 100;
    if (tmp < 0)
    {
        tmp = -tmp;
        lcd_show_string(116, 130, 8, 16, 16, "-", BLUE);
    }
    else
    {
        lcd_show_string(116, 130, 8, 16, 16, " ", BLUE);
    }
    lcd_show_num(124, 130, tmp / 100, 3, 16, BLUE);
    lcd_show_xnum(156, 130, tmp % 100, 2, 16, 0x80, BLUE);
    
    /* 俯仰角 */
    tmp = attitude_dat->pitch * 100;
    if (tmp < 0)
    {
        tmp = -tmp;
        lcd_show_string(116, 151, 8, 16, 16, "-", BLUE);
    }
    else
    {
        lcd_show_string(116, 151, 8, 16, 16, " ", BLUE);
    }
    lcd_show_num(124, 151, tmp / 100, 3, 16, BLUE);
    lcd_show_xnum(156, 151, tmp % 100, 2, 16, 0x80, BLUE);
    
    /* 航向角 */
    tmp = attitude_dat->yaw * 100;
    if (tmp < 0)
    {
        tmp = -tmp;
        lcd_show_string(116, 172, 8, 16, 16, "-", BLUE);
    }
    else
    {
        lcd_show_string(116, 172, 8, 16, 16, " ", BLUE);
    }
    lcd_show_num(124, 172, tmp / 100, 3, 16, BLUE);
    lcd_show_xnum(156, 172, tmp % 100, 2, 16, 0x80, BLUE);
    
    /* 气压 */
    tmp = barometer_dat->pressure;
    if (tmp < 0)
    {
        tmp = -tmp;
        lcd_show_string(116, 193, 8, 16, 16, "-", BLUE);
    }
    else
    {
        lcd_show_string(116, 193, 8, 16, 16, " ", BLUE);
    }
    lcd_show_num(124, 193, tmp, 6, 16, BLUE);
    
    /* 海拔 */
    tmp = barometer_dat->altitude;
    if (tmp < 0)
    {
        tmp = -tmp;
        lcd_show_string(116, 214, 8, 16, 16, "-", BLUE);
    }
    else
    {
        lcd_show_string(116, 214, 8, 16, 16, " ", BLUE);
    }
    lcd_show_num(124, 214, tmp, 6, 16, BLUE);
    
    /* 温度 */
    tmp = barometer_dat->temperature * 100;
    if (tmp < 0)
    {
        tmp = -tmp;
        lcd_show_string(116, 235, 8, 16, 16, "-", BLUE);
    }
    else
    {
        lcd_show_string(116, 235, 8, 16, 16, " ", BLUE);
    }
    lcd_show_num(124, 235, tmp / 100, 3, 16, BLUE);
    lcd_show_xnum(156, 235, tmp % 100, 2, 16, 0x80, BLUE);
}

/**
 * @brief       按键0功能，获取并显示ATK-MS901M数据
 * @param       无
 * @retval      无
 */
static void demo_key0_fun(void)
{
    atk_ms901m_attitude_data_t attitude_dat;           /* 姿态角数据 */
    atk_ms901m_gyro_data_t gyro_dat;                   /* 陀螺仪数据 */
    atk_ms901m_accelerometer_data_t accelerometer_dat; /* 加速度计数据 */
    atk_ms901m_magnetometer_data_t magnetometer_dat;   /* 磁力计数据 */
    atk_ms901m_barometer_data_t barometer_dat;         /* 气压计数据 */
    
    /* 获取ATK-MS901数据 */
    atk_ms901m_get_attitude(&attitude_dat, 100);                            /* 获取姿态角数据 */
    atk_ms901m_get_gyro_accelerometer(&gyro_dat, &accelerometer_dat, 100);  /* 获取陀螺仪、加速度计数据 */
    atk_ms901m_get_magnetometer(&magnetometer_dat, 100);                    /* 获取磁力计数据 */
    atk_ms901m_get_barometer(&barometer_dat, 100);                          /* 获取气压计数据 */
    
    /* LCD显示数据 */
    demo_show_data_lcd(&attitude_dat, &barometer_dat);
    
    /* 串口打印数据 */
    printf("Roll: %.02f° Pitch: %.02f° Yaw: %.02f°\r\n", attitude_dat.roll, attitude_dat.pitch, attitude_dat.yaw);
    printf("Gx: %.02f°/s Gy: %.02f°/s Gz: %.02f°/s\r\n", gyro_dat.x, gyro_dat.y, gyro_dat.z);
    printf("Ax: %.02fG Ay: %.02fG Az: %.02fG\r\n", accelerometer_dat.x, accelerometer_dat.y, accelerometer_dat.z);
    printf("Mx: %d My: %d Mz: %d, Temp: %.02f°\r\n", magnetometer_dat.x, magnetometer_dat.y, magnetometer_dat.z, magnetometer_dat.temperature);
    printf("Pres: %dPa Alt: %dcm Temp: %.02f°\r\n", barometer_dat.pressure, barometer_dat.altitude, barometer_dat.temperature);
    printf("****************************************\r\n\r\n");
}

/**
 * @brief       例程演示入口函数
 * @param       无
 * @retval      无
 */
void demo_run(void)
{
    uint8_t ret;
    uint8_t key;
    
    /* 初始化ATK-MS901M */
    ret = atk_ms901m_init(115200);
    if (ret != 0)
    {
        printf("ATK-MS901M init failed!\r\n");
        while (1)
        {
            LED0_TOGGLE();
            delay_ms(200);
        }
    }
    
    printf("ATK-MS901M init success!\r\n\n");
    
    /* LCD UI初始化 */
    demo_lcd_ui_init();
    
    while (1)
    {
        key = key_scan(0);
        
        switch (key)
        {
            case KEY0_PRES:
            {
                /* 获取并显示ATK-MS901数据 */
                demo_key0_fun();
                break;
            }
            default:
            {
                break;
            }
        }
        
        delay_ms(10);
    }
}
