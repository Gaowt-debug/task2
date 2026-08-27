#include "sw_i2c.h"

/* ==================== 软件模拟 I2C 实现 ====================
 * 引脚：PB6=SCL，PB7=SDA，开漏+上拉（CubeMX 已配）
 * 时钟：72MHz，半周期延时约 5us，总线速率约 100kHz
 * 地址约定：调用方传入 7 位 slave_addr（MPU6050 = 0x68），
 *           内部左移 1 位再拼 R/W 位
 * ========================================================= */

/* ---- 引脚操作宏：开漏模式下写 1=释放(上拉拉高)，写 0=拉低 ---- */
#define SCL_HIGH()   HAL_GPIO_WritePin(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN, GPIO_PIN_SET)
#define SCL_LOW()    HAL_GPIO_WritePin(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN, GPIO_PIN_RESET)
#define SDA_HIGH()   HAL_GPIO_WritePin(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN, GPIO_PIN_SET)
#define SDA_LOW()    HAL_GPIO_WritePin(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN, GPIO_PIN_RESET)
#define SDA_READ()   HAL_GPIO_ReadPin(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN)
#define SCL_READ()   HAL_GPIO_ReadPin(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN)

/* ---- 半位延时：约 1.2us（72MHz，总线约 400kHz，MPU6050 fast mode） ---- */
static void sw_i2c_delay(void)
{
    volatile unsigned char i = 0;
    for (i = 0; i < 7; i++)
    {
        __NOP();   /* 空操作，让编译器不优化掉循环 */
    }
}

/* ---- 等待 SCL 实际拉高（开漏模式下从机可能拉低时钟做时钟拉伸） ---- */
static void sw_i2c_wait_scl_high(void)
{
    unsigned int timeout = 200;   /* 超时保护，防止从机一直拉低 SCL 卡死 */
    while (SCL_READ() == 0 && timeout--)
    {
        sw_i2c_delay();
    }
}



/* ==================== 基本时序 ==================== */

/* 起始条件：SCL 高时 SDA 由高到低跳变 */
static void sw_i2c_start(void)
{
    SDA_HIGH();
    SCL_HIGH();
    sw_i2c_delay();
    SDA_LOW();      /* SCL 仍为高，SDA 下降沿 = START */
    sw_i2c_delay();
    SCL_LOW();      /* 钳住总线，准备发数据 */
    sw_i2c_delay();
}

/* 停止条件：SCL 高时 SDA 由低到高跳变 */
static void sw_i2c_stop(void)
{
    SCL_LOW();
    SDA_LOW();
    sw_i2c_delay();
    SCL_HIGH();
    sw_i2c_wait_scl_high();
    sw_i2c_delay();
    SDA_HIGH();     /* SCL 为高，SDA 上升沿 = STOP */
    sw_i2c_delay();
}

/* 发送一个字节（MSB 先发），返回从机 ACK：0=有应答，1=无应答 */
static unsigned char sw_i2c_write_byte(unsigned char byte)
{
    unsigned char i;
    unsigned char ack;

    for (i = 0; i < 8; i++)
    {
        if (byte & 0x80)
            SDA_HIGH();
        else
            SDA_LOW();
        byte <<= 1;

        sw_i2c_delay();
        SCL_HIGH();
        sw_i2c_wait_scl_high();
        sw_i2c_delay();
        SCL_LOW();
        sw_i2c_delay();
    }

    /* 第 9 个时钟：读从机应答 */
    SDA_HIGH();          /* 释放 SDA，让从机能驱动 */
    sw_i2c_delay();
    SCL_HIGH();
    sw_i2c_wait_scl_high();
    sw_i2c_delay();
    ack = SDA_READ();   /* 0=ACK，1=NACK */
    SCL_LOW();
    sw_i2c_delay();

    return ack;          /* 0 表示有应答 */
}

/* 读取一个字节（MSB 先收），ack=0 主机回 ACK（继续读），ack=1 回 NACK（最后一字节） */
static unsigned char sw_i2c_read_byte(unsigned char ack)
{
    unsigned char i;
    unsigned char byte = 0;

    SDA_HIGH();          /* 释放 SDA，让从机驱动 */

    for (i = 0; i < 8; i++)
    {
        byte <<= 1;
        SCL_HIGH();
        sw_i2c_wait_scl_high();
        sw_i2c_delay();
        if (SDA_READ())
            byte |= 0x01;
        SCL_LOW();
        sw_i2c_delay();
    }

    /* 主机应答 */
    if (ack)
        SDA_HIGH();     /* NACK */
    else
        SDA_LOW();      /* ACK */
    sw_i2c_delay();
    SCL_HIGH();
    sw_i2c_wait_scl_high();
    sw_i2c_delay();
    SCL_LOW();
    sw_i2c_delay();

    SDA_HIGH();         /* 释放 SDA */
    return byte;
}

/* ==================== 对外接口 ==================== */

int sw_i2c_init(void)
{
    /* 开漏模式下写 1 = 释放总线，确保空闲 */
    SCL_HIGH();
    SDA_HIGH();
    sw_i2c_delay();
    return 0;
}

int sw_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                 unsigned char length, unsigned char *data)
{
    unsigned char i;

    sw_i2c_start();

    /* 发地址 + W（7 位地址左移 1，最低位 0 = 写） */
    if (sw_i2c_write_byte((slave_addr << 1) | 0x00))
    {
        sw_i2c_stop();
        return -1;       /* 无应答 */
    }

    /* 发寄存器地址 */
    if (sw_i2c_write_byte(reg_addr))
    {
        sw_i2c_stop();
        return -1;
    }

    /* 发数据 */
    for (i = 0; i < length; i++)
    {
        if (sw_i2c_write_byte(data[i]))
        {
            sw_i2c_stop();
            return -1;
        }
    }

    sw_i2c_stop();
    return 0;
}

int sw_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                unsigned char length, unsigned char *data)
{
    unsigned char i;

    /* ---- 第一阶段：写寄存器地址（dummy write） ---- */
    sw_i2c_start();
    if (sw_i2c_write_byte((slave_addr << 1) | 0x00))
    {
        sw_i2c_stop();
        return -1;
    }
    if (sw_i2c_write_byte(reg_addr))
    {
        sw_i2c_stop();
        return -1;
    }

    /* ---- 第二阶段：重启总线，切换为读 ---- */
    sw_i2c_start();
    if (sw_i2c_write_byte((slave_addr << 1) | 0x01))   /* 地址 + R */
    {
        sw_i2c_stop();
        return -1;
    }

    /* 读数据：前 length-1 个字节回 ACK，最后一个字节回 NACK */
    for (i = 0; i < length; i++)
    {
        data[i] = sw_i2c_read_byte(i == (length - 1) ? 1 : 0);
    }

    sw_i2c_stop();
    return 0;
}
