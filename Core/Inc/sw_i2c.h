#ifndef __SW_I2C_H
#define __SW_I2C_H

#include "stm32f1xx_hal.h"

/* ==================== 软件模拟 I2C（GPIO bit-bang） ====================
 * 平台：STM32F103C8T6 @72MHz
 * 引脚：PB6 = SCL，PB7 = SDA（已在 CubeMX 中配为开漏+上拉）
 * 原理：开漏模式下写 1 = 释放（被上拉拉高），写 0 = 拉低；
 *        读 IDR 始终反映引脚真实电平，故无需切换输入模式即可读 SDA。
 * 用途：供 eMPL（inv_mpu / dmp）库的平台层 i2c_write / i2c_read 调用，
 *        接口签名与库要求一致：slave_addr 为 7 位地址（0x68）。
 * ======================================================================== */

#define SW_I2C_SCL_PORT    GPIOB
#define SW_I2C_SCL_PIN     GPIO_PIN_6
#define SW_I2C_SDA_PORT    GPIOB
#define SW_I2C_SDA_PIN     GPIO_PIN_7

/* 初始化（拉高总线，确认空闲） */
int sw_i2c_init(void);

/* 写：向 slave_addr(7bit) 的 reg_addr 连续写 length 字节 data
 * 返回 0=成功，-1=失败（无应答或总线异常） */
int sw_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                 unsigned char length, unsigned char *data);

/* 读：从 slave_addr(7bit) 的 reg_addr 连续读 length 字节到 data
 * 返回 0=成功，-1=失败 */
int sw_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                unsigned char length, unsigned char *data);

#endif /* __SW_I2C_H */
