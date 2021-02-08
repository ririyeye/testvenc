/*
* Copyright (C) Hisilicon Technologies Co., Ltd. 2012-2019. All rights reserved.
* Description:
* Author: Hisilicon multimedia software group
* Create: 2011/06/28
*/

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include "hi_comm_video.h"
#include "hi_sns_ctrl.h"

#ifdef HI_GPIO_I2C
#include "gpioi2c_ex.h"
#else
#include "hi_i2c.h"
#endif

const unsigned char sc4210_i2c_addr     =    0x60;        /* I2C Address of SC4210 */
const unsigned int  sc4210_addr_byte    =    2;
const unsigned int  sc4210_data_byte    =    1;
static int g_fd[ISP_MAX_PIPE_NUM] = {[0 ...(ISP_MAX_PIPE_NUM - 1)] = -1};

extern ISP_SNS_STATE_S       *g_pastSc4210[ISP_MAX_PIPE_NUM];
extern ISP_SNS_COMMBUS_U      g_aunSc4210BusInfo[];

int sc4210_i2c_init(VI_PIPE ViPipe)
{
    char acDevFile[16] = {0};
    HI_U8 u8DevNum;

    if (g_fd[ViPipe] >= 0) {
        return HI_SUCCESS;
    }
#ifdef HI_GPIO_I2C
    int ret;

    g_fd[ViPipe] = open("/dev/gpioi2c_ex", O_RDONLY, S_IRUSR);
    if (g_fd[ViPipe] < 0) {
        ISP_ERR_TRACE("Open gpioi2c_ex error!\n");
        return HI_FAILURE;
    }
#else
    int ret;

    u8DevNum = g_aunSc4210BusInfo[ViPipe].s8I2cDev;
    snprintf(acDevFile, sizeof(acDevFile),  "/dev/i2c-%u", u8DevNum);

    g_fd[ViPipe] = open(acDevFile, O_RDWR, S_IRUSR | S_IWUSR);

    if (g_fd[ViPipe] < 0) {
        ISP_ERR_TRACE("Open /dev/hi_i2c_drv-%u error!\n", u8DevNum);
        return HI_FAILURE;
    }

    ret = ioctl(g_fd[ViPipe], I2C_SLAVE_FORCE, (sc4210_i2c_addr >> 1));
    if (ret < 0) {
        ISP_ERR_TRACE("I2C_SLAVE_FORCE error!\n");
        close(g_fd[ViPipe]);
        g_fd[ViPipe] = -1;
        return ret;
    }
#endif

    return HI_SUCCESS;
}

int sc4210_i2c_exit(VI_PIPE ViPipe)
{
    if (g_fd[ViPipe] >= 0) {
        close(g_fd[ViPipe]);
        g_fd[ViPipe] = -1;
        return HI_SUCCESS;
    }
    return HI_FAILURE;
}

int sc4210_read_register(VI_PIPE ViPipe, int addr)
{
    return HI_SUCCESS;
}


int sc4210_write_register(VI_PIPE ViPipe, int addr, int data)
{
    if (g_fd[ViPipe] < 0) {
        return HI_SUCCESS;
    }

#ifdef HI_GPIO_I2C
    i2c_data.dev_addr = sc4210_i2c_addr;
    i2c_data.reg_addr = addr;
    i2c_data.addr_byte_num = sc4210_addr_byte;
    i2c_data.data = data;
    i2c_data.data_byte_num = sc4210_data_byte;

    ret = ioctl(g_fd[ViPipe], GPIO_I2C_WRITE, &i2c_data);

    if (ret) {
        ISP_ERR_TRACE("GPIO-I2C write faild!\n");
        return ret;
    }
#else
    int idx = 0;
    int ret;
    char buf[8];

    if (sc4210_addr_byte == 2) {
        buf[idx] = (addr >> 8) & 0xff;
        idx++;
        buf[idx] = addr & 0xff;
        idx++;
    } else {
    }

    if (sc4210_data_byte == 2) {
    } else {
        buf[idx] = data & 0xff;
        idx++;
    }

    ret = write(g_fd[ViPipe], buf, sc4210_addr_byte + sc4210_data_byte);
    if (ret < 0) {
        ISP_ERR_TRACE("I2C_WRITE error!\n");
        return HI_FAILURE;
    }

#endif
    return HI_SUCCESS;
}

static void delay_ms(int ms)
{
    usleep(ms * 1000);
}

void sc4210_prog(VI_PIPE ViPipe, const int *rom)
{
    int i = 0;
    while (1) {
        int lookup = rom[i++];
        int addr = (lookup >> 16) & 0xFFFF;
        int data = lookup & 0xFFFF;
        if (addr == 0xFFFE) {
            delay_ms(data);
        } else if (addr == 0xFFFF) {
            return;
        } else {
            sc4210_write_register(ViPipe, addr, data);
        }
    }
}

void sc4210_standby(VI_PIPE ViPipe)
{
    return;
}

void sc4210_restart(VI_PIPE ViPipe)
{
    return;
}

#define SC4210_SENSOR_1440P_30FPS_LINEAR_MODE (1)

void sc4210_linear_1440p30_init(VI_PIPE ViPipe);
void sc4210_wdr_1440p30_init(VI_PIPE ViPipe);


void sc4210_default_reg_init(VI_PIPE ViPipe)
{
    HI_U32 i;

    for (i = 0; i < g_pastSc4210[ViPipe]->astRegsInfo[0].u32RegNum; i++) {
        sc4210_write_register(ViPipe, g_pastSc4210[ViPipe]->astRegsInfo[0].astI2cData[i].u32RegAddr, g_pastSc4210[ViPipe]->astRegsInfo[0].astI2cData[i].u32Data);
    }
}

void sc4210_init(VI_PIPE ViPipe)
{
    HI_BOOL bInit;
    WDR_MODE_E       enWDRMode;

    bInit = g_pastSc4210[ViPipe]->bInit;

    sc4210_i2c_init(ViPipe);

    enWDRMode = g_pastSc4210[ViPipe]->enWDRMode;

    /* When sensor first init, config all registers */
    if (bInit == HI_FALSE) {
        if (WDR_MODE_2To1_LINE == enWDRMode) {
            sc4210_wdr_1440p30_init(ViPipe);
        } else {
            sc4210_linear_1440p30_init(ViPipe);
        }
    } else {
        if (WDR_MODE_2To1_LINE == enWDRMode) {
            sc4210_wdr_1440p30_init(ViPipe);
        } else {
            sc4210_linear_1440p30_init(ViPipe);
        }
    }

    g_pastSc4210[ViPipe]->bInit = HI_TRUE;
    return;
}

void sc4210_exit(VI_PIPE ViPipe)
{
    sc4210_i2c_exit(ViPipe);

    return;
}

/* 1440P30 and 1440P25 */
void sc4210_linear_1440p30_init(VI_PIPE ViPipe)
{
    sc4210_write_register(ViPipe, 0x0103, 0x01);
    sc4210_write_register(ViPipe, 0x0100, 0x00);

    sc4210_write_register(ViPipe, 0x36e9, 0xa7);
    sc4210_write_register(ViPipe, 0x36f9, 0xa0);

    sc4210_write_register(ViPipe, 0x335d, 0x00);  /* Timing */
    sc4210_write_register(ViPipe, 0x3312, 0x02);

    sc4210_write_register(ViPipe, 0x3309, 0x68);
    sc4210_write_register(ViPipe, 0x331f, 0x59);
    sc4210_write_register(ViPipe, 0x3306, 0x58);
    sc4210_write_register(ViPipe, 0x3302, 0x18);
    sc4210_write_register(ViPipe, 0x3301, 0x30);
    sc4210_write_register(ViPipe, 0x3320, 0x01);
    sc4210_write_register(ViPipe, 0x3633, 0x33);
    sc4210_write_register(ViPipe, 0x3634, 0x64);
    sc4210_write_register(ViPipe, 0x3624, 0x07);  /* Cnt_Clk Delay */

    sc4210_write_register(ViPipe, 0x4418, 0x3a);  /* FullWell */

    sc4210_write_register(ViPipe, 0x3622, 0xff);
    sc4210_write_register(ViPipe, 0x3630, 0xc4);
    sc4210_write_register(ViPipe, 0x3631, 0x80);

    sc4210_write_register(ViPipe, 0x3635, 0x42);
    sc4210_write_register(ViPipe, 0x363b, 0x08);
    sc4210_write_register(ViPipe, 0x363c, 0x06);
    sc4210_write_register(ViPipe, 0x363d, 0x06);
    sc4210_write_register(ViPipe, 0x3638, 0x22);

    sc4210_write_register(ViPipe, 0x3253, 0x08); /* Power save mode */
    sc4210_write_register(ViPipe, 0x325e, 0x00);
    sc4210_write_register(ViPipe, 0x325f, 0x00);
    sc4210_write_register(ViPipe, 0x3251, 0x08);

    sc4210_write_register(ViPipe, 0x4418, 0x0e);  /* 12bit */
    sc4210_write_register(ViPipe, 0x330a, 0x02);
    sc4210_write_register(ViPipe, 0x330b, 0xf8);
    sc4210_write_register(ViPipe, 0x3305, 0x01);
    sc4210_write_register(ViPipe, 0x3306, 0x60);
    sc4210_write_register(ViPipe, 0x320c, 0x0a);
    sc4210_write_register(ViPipe, 0x320d, 0x8c);
    sc4210_write_register(ViPipe, 0x4501, 0xc4);

    sc4210_write_register(ViPipe, 0x33e0, 0xa0);
    sc4210_write_register(ViPipe, 0x33e1, 0x08);
    sc4210_write_register(ViPipe, 0x33e2, 0x02);
    sc4210_write_register(ViPipe, 0x33e3, 0x10);
    sc4210_write_register(ViPipe, 0x33e4, 0x20);
    sc4210_write_register(ViPipe, 0x33e5, 0x00);
    sc4210_write_register(ViPipe, 0x33e6, 0x06);
    sc4210_write_register(ViPipe, 0x33e7, 0x10);
    sc4210_write_register(ViPipe, 0x33e8, 0x00);
    sc4210_write_register(ViPipe, 0x33e9, 0x02);
    sc4210_write_register(ViPipe, 0x33ea, 0x20);
    sc4210_write_register(ViPipe, 0x33eb, 0x00);
    sc4210_write_register(ViPipe, 0x33ec, 0x04);
    sc4210_write_register(ViPipe, 0x33ed, 0x10);

    sc4210_write_register(ViPipe, 0x3225, 0x01);
    sc4210_write_register(ViPipe, 0x3227, 0x03);

    sc4210_write_register(ViPipe, 0x391b, 0x80);  /* Blc */
    sc4210_write_register(ViPipe, 0x391c, 0x0f);
    sc4210_write_register(ViPipe, 0x3935, 0x80);
    sc4210_write_register(ViPipe, 0x3936, 0x1f);
    sc4210_write_register(ViPipe, 0x3908, 0x11);

    sc4210_write_register(ViPipe, 0x3273, 0x01); /* Dummy addr & Blc addr */
    sc4210_write_register(ViPipe, 0x3241, 0x02);
    sc4210_write_register(ViPipe, 0x3243, 0x03);
    sc4210_write_register(ViPipe, 0x3249, 0x17);
    sc4210_write_register(ViPipe, 0x3229, 0x08);
    sc4210_write_register(ViPipe, 0x3905, 0xd8);

    sc4210_write_register(ViPipe, 0x300f, 0xff);

    sc4210_write_register(ViPipe, 0x3018, 0x73);  /* MIPI */
    sc4210_write_register(ViPipe, 0x3031, 0x0c);  /* Bit Mode */
    sc4210_write_register(ViPipe, 0x3000, 0x00);
    sc4210_write_register(ViPipe, 0x3001, 0x07);
    sc4210_write_register(ViPipe, 0x3002, 0xc0);
    sc4210_write_register(ViPipe, 0x300a, 0x2c);
    sc4210_write_register(ViPipe, 0x4603, 0x00);
    sc4210_write_register(ViPipe, 0x4837, 0x2c);
    sc4210_write_register(ViPipe, 0x36ea, 0x3a);

    sc4210_write_register(ViPipe, 0x300f, 0x00);

    sc4210_write_register(ViPipe, 0x3e01, 0xba);  /* Exp & Gain */
    sc4210_write_register(ViPipe, 0x3e02, 0xe0);
    sc4210_write_register(ViPipe, 0x3e08, 0x03);
    sc4210_write_register(ViPipe, 0x3e09, 0x20);
    sc4210_write_register(ViPipe, 0x3e06, 0x00);

    sc4210_write_register(ViPipe, 0x4418, 0x0d);
    sc4210_write_register(ViPipe, 0x3638, 0x27);
    sc4210_write_register(ViPipe, 0x3635, 0x40);
    sc4210_write_register(ViPipe, 0x363b, 0x06);

    sc4210_write_register(ViPipe, 0x3622, 0xff);  /* Logic */
    sc4210_write_register(ViPipe, 0x3633, 0x22);
    //sc4210_write_register (ViPipe, 0x3221, 0xff);

    sc4210_write_register(ViPipe, 0x366e, 0x04);  /* Internal Logic */
    sc4210_write_register(ViPipe, 0x360f, 0x05);
    sc4210_write_register(ViPipe, 0x367a, 0x08);
    sc4210_write_register(ViPipe, 0x367b, 0x08);
    sc4210_write_register(ViPipe, 0x3671, 0xff);
    sc4210_write_register(ViPipe, 0x3672, 0x1f);
    sc4210_write_register(ViPipe, 0x3673, 0x1f);

    sc4210_write_register(ViPipe, 0x3670, 0x08);
    sc4210_write_register(ViPipe, 0x369c, 0x08);
    sc4210_write_register(ViPipe, 0x369d, 0x08);
    sc4210_write_register(ViPipe, 0x3690, 0x22);
    sc4210_write_register(ViPipe, 0x3691, 0x44);
    sc4210_write_register(ViPipe, 0x3692, 0x44);

    sc4210_write_register(ViPipe, 0x3817, 0x20);  /* High Temp */
    sc4210_write_register(ViPipe, 0x39a0, 0x08);
    sc4210_write_register(ViPipe, 0x39a1, 0x10);
    sc4210_write_register(ViPipe, 0x39a2, 0x20);
    sc4210_write_register(ViPipe, 0x39a3, 0x40);
    sc4210_write_register(ViPipe, 0x39a4, 0x20);
    sc4210_write_register(ViPipe, 0x39a5, 0x10);
    sc4210_write_register(ViPipe, 0x39a6, 0x08);
    sc4210_write_register(ViPipe, 0x39a7, 0x04);
    sc4210_write_register(ViPipe, 0x39a8, 0x18);
    sc4210_write_register(ViPipe, 0x39a9, 0x30);
    sc4210_write_register(ViPipe, 0x39aa, 0x40);
    sc4210_write_register(ViPipe, 0x39ab, 0x60);
    sc4210_write_register(ViPipe, 0x39ac, 0x38);
    sc4210_write_register(ViPipe, 0x39ad, 0x20);
    sc4210_write_register(ViPipe, 0x39ae, 0x10);
    sc4210_write_register(ViPipe, 0x39af, 0x08);
    sc4210_write_register(ViPipe, 0x3980, 0x00);
    sc4210_write_register(ViPipe, 0x3981, 0x50);
    sc4210_write_register(ViPipe, 0x3982, 0x00);
    sc4210_write_register(ViPipe, 0x3983, 0x40);
    sc4210_write_register(ViPipe, 0x3984, 0x00);
    sc4210_write_register(ViPipe, 0x3985, 0x20);
    sc4210_write_register(ViPipe, 0x3986, 0x00);
    sc4210_write_register(ViPipe, 0x3987, 0x10);
    sc4210_write_register(ViPipe, 0x3988, 0x00);
    sc4210_write_register(ViPipe, 0x3989, 0x20);
    sc4210_write_register(ViPipe, 0x398a, 0x00);
    sc4210_write_register(ViPipe, 0x398b, 0x30);
    sc4210_write_register(ViPipe, 0x398c, 0x00);
    sc4210_write_register(ViPipe, 0x398d, 0x50);
    sc4210_write_register(ViPipe, 0x398e, 0x00);
    sc4210_write_register(ViPipe, 0x398f, 0x60);
    sc4210_write_register(ViPipe, 0x3990, 0x00);
    sc4210_write_register(ViPipe, 0x3991, 0x70);
    sc4210_write_register(ViPipe, 0x3992, 0x00);
    sc4210_write_register(ViPipe, 0x3993, 0x36);
    sc4210_write_register(ViPipe, 0x3994, 0x00);
    sc4210_write_register(ViPipe, 0x3995, 0x20);
    sc4210_write_register(ViPipe, 0x3996, 0x00);
    sc4210_write_register(ViPipe, 0x3997, 0x14);
    sc4210_write_register(ViPipe, 0x3998, 0x00);
    sc4210_write_register(ViPipe, 0x3999, 0x20);
    sc4210_write_register(ViPipe, 0x399a, 0x00);
    sc4210_write_register(ViPipe, 0x399b, 0x50);
    sc4210_write_register(ViPipe, 0x399c, 0x00);
    sc4210_write_register(ViPipe, 0x399d, 0x90);
    sc4210_write_register(ViPipe, 0x399e, 0x00);
    sc4210_write_register(ViPipe, 0x399f, 0xf0);
    sc4210_write_register(ViPipe, 0x39b9, 0x00);  /* TopV */
    sc4210_write_register(ViPipe, 0x39ba, 0xa0);
    sc4210_write_register(ViPipe, 0x39bb, 0x80);
    sc4210_write_register(ViPipe, 0x39bc, 0x00);
    sc4210_write_register(ViPipe, 0x39bd, 0x44);
    sc4210_write_register(ViPipe, 0x39be, 0x00);
    sc4210_write_register(ViPipe, 0x39bf, 0x00);
    sc4210_write_register(ViPipe, 0x39c0, 0x00);
    sc4210_write_register(ViPipe, 0x3933, 0x24);
    sc4210_write_register(ViPipe, 0x3934, 0xb0);
    sc4210_write_register(ViPipe, 0x3942, 0x04);
    sc4210_write_register(ViPipe, 0x3943, 0xc0);
    sc4210_write_register(ViPipe, 0x3940, 0x68);
    sc4210_write_register(ViPipe, 0x39c5, 0x41);

    sc4210_write_register(ViPipe, 0x36e9, 0xa7);  /* 30fps, 60.75M sysclk */
    sc4210_write_register(ViPipe, 0x36ea, 0x37);
    sc4210_write_register(ViPipe, 0x36eb, 0x16);
    sc4210_write_register(ViPipe, 0x36ec, 0x03);
    sc4210_write_register(ViPipe, 0x36ed, 0x04);

    sc4210_write_register(ViPipe, 0x36f9, 0xa0);
    sc4210_write_register(ViPipe, 0x36fa, 0x37);
    sc4210_write_register(ViPipe, 0x36fb, 0x14);
    sc4210_write_register(ViPipe, 0x36fc, 0x00);
    sc4210_write_register(ViPipe, 0x36fd, 0x24);

    sc4210_write_register(ViPipe, 0x320c, 0x05);
    sc4210_write_register(ViPipe, 0x320d, 0x46);
    sc4210_write_register(ViPipe, 0x4501, 0xb4);
    sc4210_write_register(ViPipe, 0x4418, 0x1a);
    sc4210_write_register(ViPipe, 0x3638, 0x2a);
    sc4210_write_register(ViPipe, 0x3304, 0x20);
    sc4210_write_register(ViPipe, 0x331e, 0x19);
    sc4210_write_register(ViPipe, 0x3309, 0x40);
    sc4210_write_register(ViPipe, 0x331f, 0x39);
    sc4210_write_register(ViPipe, 0x3305, 0x00);
    sc4210_write_register(ViPipe, 0x3306, 0x70);
    sc4210_write_register(ViPipe, 0x330a, 0x00);
    sc4210_write_register(ViPipe, 0x330b, 0xd8);

    sc4210_write_register(ViPipe, 0x3301, 0x30);
    sc4210_write_register(ViPipe, 0x3320, 0x05);

    sc4210_write_register(ViPipe, 0x337a, 0x08);
    sc4210_write_register(ViPipe, 0x337b, 0x10);
    sc4210_write_register(ViPipe, 0x33a3, 0x0c);

    sc4210_write_register(ViPipe, 0x3308, 0x10);
    sc4210_write_register(ViPipe, 0x3366, 0x92);
    sc4210_write_register(ViPipe, 0x4509, 0x20);

    sc4210_write_register(ViPipe, 0x3314, 0x84);
    sc4210_write_register(ViPipe, 0x330e, 0x18);
    sc4210_write_register(ViPipe, 0x334c, 0x10);

    sc4210_write_register(ViPipe, 0x3636, 0x28);
    sc4210_write_register(ViPipe, 0x4418, 0x0d);

    sc4210_write_register(ViPipe, 0x335d, 0x00);  /* Timing */
    sc4210_write_register(ViPipe, 0x3312, 0x02);
    sc4210_write_register(ViPipe, 0x336c, 0xc2);
    sc4210_write_register(ViPipe, 0x337e, 0x40);
    sc4210_write_register(ViPipe, 0x3338, 0x10);

    sc4210_write_register(ViPipe, 0x4418, 0x0c);
    sc4210_write_register(ViPipe, 0x3301, 0x28);
    sc4210_write_register(ViPipe, 0x330b, 0xe8);

    sc4210_write_register(ViPipe, 0x3622, 0xff);
    sc4210_write_register(ViPipe, 0x3633, 0x22);

    sc4210_write_register(ViPipe, 0x366e, 0x04);
    sc4210_write_register(ViPipe, 0x360f, 0x05);
    sc4210_write_register(ViPipe, 0x367a, 0x08);
    sc4210_write_register(ViPipe, 0x367b, 0x08);
    sc4210_write_register(ViPipe, 0x3671, 0xff);
    sc4210_write_register(ViPipe, 0x3672, 0x1f);
    sc4210_write_register(ViPipe, 0x3673, 0x1f);
    sc4210_write_register(ViPipe, 0x3670, 0x48);
    sc4210_write_register(ViPipe, 0x369c, 0x08);
    sc4210_write_register(ViPipe, 0x369d, 0x08);
    sc4210_write_register(ViPipe, 0x3690, 0x42);
    sc4210_write_register(ViPipe, 0x3691, 0x44);
    sc4210_write_register(ViPipe, 0x3692, 0x44);
    sc4210_write_register(ViPipe, 0x36a2, 0x08);
    sc4210_write_register(ViPipe, 0x36a3, 0x08);
    sc4210_write_register(ViPipe, 0x3699, 0x80);
    sc4210_write_register(ViPipe, 0x369a, 0x9f);
    sc4210_write_register(ViPipe, 0x369b, 0x9f);

    sc4210_write_register(ViPipe, 0x4509, 0x10);  /* Init Code */
    sc4210_write_register(ViPipe, 0x3231, 0x01);
    sc4210_write_register(ViPipe, 0x3220, 0x10);
    sc4210_write_register(ViPipe, 0x3e0e, 0x6a);
    sc4210_write_register(ViPipe, 0x3625, 0x02);
    sc4210_write_register(ViPipe, 0x3636, 0x20);
    sc4210_write_register(ViPipe, 0x4418, 0x0c);

    sc4210_write_register(ViPipe, 0x366e, 0x04);
    sc4210_write_register(ViPipe, 0x360f, 0x05);
    sc4210_write_register(ViPipe, 0x367a, 0x40);
    sc4210_write_register(ViPipe, 0x367b, 0x40);
    sc4210_write_register(ViPipe, 0x3671, 0xff);
    sc4210_write_register(ViPipe, 0x3672, 0x1f);
    sc4210_write_register(ViPipe, 0x3673, 0x1f);
    sc4210_write_register(ViPipe, 0x3670, 0x48);
    sc4210_write_register(ViPipe, 0x369c, 0x40);
    sc4210_write_register(ViPipe, 0x369d, 0x40);
    sc4210_write_register(ViPipe, 0x3690, 0x42);
    sc4210_write_register(ViPipe, 0x3691, 0x44);
    sc4210_write_register(ViPipe, 0x3692, 0x44);
    sc4210_write_register(ViPipe, 0x36a2, 0x40);
    sc4210_write_register(ViPipe, 0x36a3, 0x40);
    sc4210_write_register(ViPipe, 0x3699, 0x80);
    sc4210_write_register(ViPipe, 0x369a, 0x9f);
    sc4210_write_register(ViPipe, 0x369b, 0x9f);
    sc4210_write_register(ViPipe, 0x36d0, 0x20);
    sc4210_write_register(ViPipe, 0x36d1, 0x40);  /* Gain0 */
    sc4210_write_register(ViPipe, 0x36d2, 0x40);  /* Gain1 */
    sc4210_write_register(ViPipe, 0x36cc, 0x2c);  /* Sel0 */
    sc4210_write_register(ViPipe, 0x36cd, 0x30);  /* Sel1 */
    sc4210_write_register(ViPipe, 0x36ce, 0x30);  /* Sel2 */

    sc4210_write_register(ViPipe, 0x33e0, 0xa0);
    sc4210_write_register(ViPipe, 0x33e1, 0x08);
    sc4210_write_register(ViPipe, 0x33e2, 0x02);
    sc4210_write_register(ViPipe, 0x33e3, 0x06);
    sc4210_write_register(ViPipe, 0x33e4, 0x07);
    sc4210_write_register(ViPipe, 0x33e5, 0x02);
    sc4210_write_register(ViPipe, 0x33e6, 0x04);
    sc4210_write_register(ViPipe, 0x33e7, 0x08);
    sc4210_write_register(ViPipe, 0x33e8, 0x00);
    sc4210_write_register(ViPipe, 0x33e9, 0x06);
    sc4210_write_register(ViPipe, 0x33ea, 0x10);
    sc4210_write_register(ViPipe, 0x33eb, 0x00);
    sc4210_write_register(ViPipe, 0x33ec, 0x04);
    sc4210_write_register(ViPipe, 0x33ed, 0x10);
    sc4210_write_register(ViPipe, 0x33ee, 0xa0);
    sc4210_write_register(ViPipe, 0x33ef, 0x08);
    sc4210_write_register(ViPipe, 0x33f4, 0x02);
    sc4210_write_register(ViPipe, 0x33f5, 0x06);
    sc4210_write_register(ViPipe, 0x33f6, 0x07);
    sc4210_write_register(ViPipe, 0x33f7, 0x02);
    sc4210_write_register(ViPipe, 0x33f8, 0x04);
    sc4210_write_register(ViPipe, 0x33f9, 0x08);
    sc4210_write_register(ViPipe, 0x33fa, 0x00);
    sc4210_write_register(ViPipe, 0x33fb, 0x06);
    sc4210_write_register(ViPipe, 0x33fc, 0x10);
    sc4210_write_register(ViPipe, 0x33fd, 0x00);
    sc4210_write_register(ViPipe, 0x33fe, 0x04);
    sc4210_write_register(ViPipe, 0x33ff, 0x10);
    sc4210_write_register(ViPipe, 0x5784, 0x0b);
    sc4210_write_register(ViPipe, 0x5787, 0x07);
    sc4210_write_register(ViPipe, 0x5788, 0x06);
    sc4210_write_register(ViPipe, 0x5789, 0x02);
    sc4210_write_register(ViPipe, 0x578a, 0x08);
    sc4210_write_register(ViPipe, 0x578b, 0x04);
    sc4210_write_register(ViPipe, 0x578c, 0x02);
    sc4210_write_register(ViPipe, 0x57c4, 0x0b);
    sc4210_write_register(ViPipe, 0x57c7, 0x07);
    sc4210_write_register(ViPipe, 0x57c8, 0x06);
    sc4210_write_register(ViPipe, 0x57c9, 0x02);
    sc4210_write_register(ViPipe, 0x57ca, 0x08);
    sc4210_write_register(ViPipe, 0x57cb, 0x04);
    sc4210_write_register(ViPipe, 0x57cc, 0x02);
    sc4210_write_register(ViPipe, 0x5000, 0x06);

    sc4210_write_register(ViPipe, 0x3e26, 0x40);

    sc4210_write_register(ViPipe, 0x4418, 0x0b);
    sc4210_write_register(ViPipe, 0x3306, 0x74);

    sc4210_write_register(ViPipe, 0x5784, 0x10);
    sc4210_write_register(ViPipe, 0x5785, 0x08);
    sc4210_write_register(ViPipe, 0x5787, 0x06);
    sc4210_write_register(ViPipe, 0x5788, 0x06);
    sc4210_write_register(ViPipe, 0x5789, 0x00);
    sc4210_write_register(ViPipe, 0x578a, 0x06);
    sc4210_write_register(ViPipe, 0x578b, 0x06);
    sc4210_write_register(ViPipe, 0x578c, 0x00);
    sc4210_write_register(ViPipe, 0x5790, 0x10);
    sc4210_write_register(ViPipe, 0x5791, 0x10);
    sc4210_write_register(ViPipe, 0x5792, 0x00);
    sc4210_write_register(ViPipe, 0x5793, 0x10);
    sc4210_write_register(ViPipe, 0x5794, 0x10);
    sc4210_write_register(ViPipe, 0x5795, 0x00);
    sc4210_write_register(ViPipe, 0x57c4, 0x10);
    sc4210_write_register(ViPipe, 0x57c5, 0x08);
    sc4210_write_register(ViPipe, 0x57c7, 0x06);
    sc4210_write_register(ViPipe, 0x57c8, 0x06);
    sc4210_write_register(ViPipe, 0x57c9, 0x00);
    sc4210_write_register(ViPipe, 0x57ca, 0x06);
    sc4210_write_register(ViPipe, 0x57cb, 0x06);
    sc4210_write_register(ViPipe, 0x57cc, 0x00);
    sc4210_write_register(ViPipe, 0x57d0, 0x10);
    sc4210_write_register(ViPipe, 0x57d1, 0x10);
    sc4210_write_register(ViPipe, 0x57d2, 0x00);
    sc4210_write_register(ViPipe, 0x57d3, 0x10);
    sc4210_write_register(ViPipe, 0x57d4, 0x10);
    sc4210_write_register(ViPipe, 0x57d5, 0x00);

    sc4210_write_register(ViPipe, 0x33e0, 0xa0);
    sc4210_write_register(ViPipe, 0x33e1, 0x08);
    sc4210_write_register(ViPipe, 0x33e2, 0x00);
    sc4210_write_register(ViPipe, 0x33e3, 0x10);
    sc4210_write_register(ViPipe, 0x33e4, 0x10);
    sc4210_write_register(ViPipe, 0x33e5, 0x00);
    sc4210_write_register(ViPipe, 0x33e6, 0x10);
    sc4210_write_register(ViPipe, 0x33e7, 0x10);
    sc4210_write_register(ViPipe, 0x33e8, 0x00);
    sc4210_write_register(ViPipe, 0x33e9, 0x10);
    sc4210_write_register(ViPipe, 0x33ea, 0x16);
    sc4210_write_register(ViPipe, 0x33eb, 0x00);
    sc4210_write_register(ViPipe, 0x33ec, 0x10);
    sc4210_write_register(ViPipe, 0x33ed, 0x18);
    sc4210_write_register(ViPipe, 0x33ee, 0xa0);
    sc4210_write_register(ViPipe, 0x33ef, 0x08);
    sc4210_write_register(ViPipe, 0x33f4, 0x00);
    sc4210_write_register(ViPipe, 0x33f5, 0x10);
    sc4210_write_register(ViPipe, 0x33f6, 0x10);
    sc4210_write_register(ViPipe, 0x33f7, 0x00);
    sc4210_write_register(ViPipe, 0x33f8, 0x10);
    sc4210_write_register(ViPipe, 0x33f9, 0x10);
    sc4210_write_register(ViPipe, 0x33fa, 0x00);
    sc4210_write_register(ViPipe, 0x33fb, 0x10);
    sc4210_write_register(ViPipe, 0x33fc, 0x16);
    sc4210_write_register(ViPipe, 0x33fd, 0x00);
    sc4210_write_register(ViPipe, 0x33fe, 0x10);
    sc4210_write_register(ViPipe, 0x33ff, 0x18);

    sc4210_write_register(ViPipe, 0x3638, 0x28);
    sc4210_write_register(ViPipe, 0x36ed, 0x0c);
    sc4210_write_register(ViPipe, 0x36fd, 0x2c);
    sc4210_write_register(ViPipe, 0x363b, 0x03);
    sc4210_write_register(ViPipe, 0x3635, 0x20);

    sc4210_write_register(ViPipe, 0x335d, 0x20);
    sc4210_write_register(ViPipe, 0x330e, 0x18);
    sc4210_write_register(ViPipe, 0x3367, 0x08);
    sc4210_write_register(ViPipe, 0x3368, 0x05);  /* vts */
    sc4210_write_register(ViPipe, 0x3369, 0xdc);
    sc4210_write_register(ViPipe, 0x336a, 0x0b);  /* vts*2 */
    sc4210_write_register(ViPipe, 0x336b, 0xb8);

    sc4210_write_register(ViPipe, 0x550f, 0x20);
    sc4210_write_register(ViPipe, 0x4407, 0xb0);
    sc4210_write_register(ViPipe, 0x3632, 0x88);

    sc4210_write_register(ViPipe, 0x3e00, 0x00);
    sc4210_write_register(ViPipe, 0x3e01, 0xbb);
    sc4210_write_register(ViPipe, 0x3e02, 0x40);
    sc4210_write_register(ViPipe, 0x3e03, 0x0b);
    sc4210_write_register(ViPipe, 0x3e06, 0x00);
    sc4210_write_register(ViPipe, 0x3e07, 0x80);
    sc4210_write_register(ViPipe, 0x3e08, 0x03);
    sc4210_write_register(ViPipe, 0x3e09, 0x40);

    sc4210_write_register(ViPipe, 0x36e9, 0x27);
    sc4210_write_register(ViPipe, 0x36f9, 0x20);

    sc4210_default_reg_init(ViPipe);
    sc4210_write_register(ViPipe, 0x0100, 0x01);

    printf("===SC4210 1440P 30fps 12bit LINE Init OK!===\n");
    return;
}


/* 1440P30 and 1440P25 */
void sc4210_wdr_1440p30_init(VI_PIPE ViPipe)
{
    sc4210_write_register (ViPipe, 0x0103, 0x01);
    sc4210_write_register (ViPipe, 0x0100, 0x00);

    sc4210_write_register (ViPipe, 0x36e9, 0xa4); // pll1 bypass
    sc4210_write_register (ViPipe, 0x36f9, 0xa0); // pll2 bypass

    // timing
    sc4210_write_register (ViPipe, 0x335d, 0x00); // [6]sp rst all1  1008
    sc4210_write_register (ViPipe, 0x3312, 0x02); // [7]sp dcg all1
    sc4210_write_register (ViPipe, 0x336c, 0xc2); // hcg rst all1 c2
    sc4210_write_register (ViPipe, 0x337e, 0x40);
    sc4210_write_register (ViPipe, 0x3338, 0x10); // [4] sp addr to rp addr disable

    sc4210_write_register (ViPipe, 0x335d, 0x40);
    sc4210_write_register (ViPipe, 0x3312, 0x82);

    sc4210_write_register (ViPipe, 0x3309, 0x68);
    sc4210_write_register (ViPipe, 0x331f, 0x59);
    sc4210_write_register (ViPipe, 0x3306, 0x58);
    sc4210_write_register (ViPipe, 0x3302, 0x18);
    sc4210_write_register (ViPipe, 0x3301, 0x30);
    sc4210_write_register (ViPipe, 0x3320, 0x01);
    sc4210_write_register (ViPipe, 0x3633, 0x33);
    sc4210_write_register (ViPipe, 0x3634, 0x64);

    sc4210_write_register (ViPipe, 0x3624, 0x07); // cnt_clk delay

    // fullwell
    sc4210_write_register (ViPipe, 0x4418, 0x3a); // 11.9K fullwell

    // blksun
    sc4210_write_register (ViPipe, 0x3622, 0xff);
    sc4210_write_register (ViPipe, 0x3630, 0xc4);
    sc4210_write_register (ViPipe, 0x3631, 0x80);

    // pump
    sc4210_write_register (ViPipe, 0x3635, 0x42);
    sc4210_write_register (ViPipe, 0x363b, 0x08);
    sc4210_write_register (ViPipe, 0x363c, 0x06);
    sc4210_write_register (ViPipe, 0x363d, 0x06);
    sc4210_write_register (ViPipe, 0x3038, 0x22);

    // power save mode
    sc4210_write_register (ViPipe, 0x3253, 0x08); // quit power save mode in ahead of 8 half row of vts
    sc4210_write_register (ViPipe, 0x325e, 0x00); // exp threshold
    sc4210_write_register (ViPipe, 0x325f, 0x00);
    sc4210_write_register (ViPipe, 0x3251, 0x08); // [6]: 0: vts  1:exp

    // 12bit
    sc4210_write_register (ViPipe, 0x4418, 0x0e);
    sc4210_write_register (ViPipe, 0x330a, 0x02);
    sc4210_write_register (ViPipe, 0x330b, 0xf8);
    sc4210_write_register (ViPipe, 0x3305, 0x01);
    sc4210_write_register (ViPipe, 0x3306, 0x60);
    sc4210_write_register (ViPipe, 0x320c, 0x0a);
    sc4210_write_register (ViPipe, 0x320d, 0x8c);
    sc4210_write_register (ViPipe, 0x4501, 0xc4);

    /* digital updates start */
    sc4210_write_register (ViPipe, 0x33e0, 0xa0);
    sc4210_write_register (ViPipe, 0x33e1, 0x08);
    sc4210_write_register (ViPipe, 0x33e2, 0x02);
    sc4210_write_register (ViPipe, 0x33e3, 0x10);
    sc4210_write_register (ViPipe, 0x33e4, 0x20);
    sc4210_write_register (ViPipe, 0x33e5, 0x00);
    sc4210_write_register (ViPipe, 0x33e6, 0x06);
    sc4210_write_register (ViPipe, 0x33e7, 0x10);
    sc4210_write_register (ViPipe, 0x33e8, 0x00);
    sc4210_write_register (ViPipe, 0x33e9, 0x02);
    sc4210_write_register (ViPipe, 0x33ea, 0x20);
    sc4210_write_register (ViPipe, 0x33eb, 0x00);
    sc4210_write_register (ViPipe, 0x33ec, 0x04);
    sc4210_write_register (ViPipe, 0x33ed, 0x10);

    sc4210_write_register (ViPipe, 0x3225, 0x01);
    sc4210_write_register (ViPipe, 0x3227, 0x03);

    // blc
    sc4210_write_register (ViPipe, 0x391b, 0x80);
    sc4210_write_register (ViPipe, 0x391c, 0x0f);
    sc4210_write_register (ViPipe, 0x3935, 0x80);
    sc4210_write_register (ViPipe, 0x3936, 0x1f);
    sc4210_write_register (ViPipe, 0x3908, 0x11);

    // dummy addr & blc addr
    sc4210_write_register (ViPipe, 0x3273, 0x01);
    sc4210_write_register (ViPipe, 0x3241, 0x02);
    sc4210_write_register (ViPipe, 0x3243, 0x03);
    sc4210_write_register (ViPipe, 0x3249, 0x17);
    sc4210_write_register (ViPipe, 0x3229, 0x08);
    sc4210_write_register (ViPipe, 0x3905, 0xd8); // one channel blc


    /* digital updates end */
    sc4210_write_register (ViPipe, 0x300f, 0xff);

    // DVP mode : close mipi
    sc4210_write_register (ViPipe, 0x3018, 0x73); // [7:5] lane_num-1 [4] digital mipi pad en 1:mipi 0:DVP
    sc4210_write_register (ViPipe, 0x3031, 0x0a); // [3:0] bitmode
    sc4210_write_register (ViPipe, 0x3000, 0x00); // DVP oen [3:0]->DATA[11:8]
    sc4210_write_register (ViPipe, 0x3001, 0x07); // DVP oen [7:0]->DATA[7:0]  not reused :[2:0] dat[2:0] oen  0705
    sc4210_write_register (ViPipe, 0x3002, 0xc0); // [6] oen href [5] oen pclk  not reused href  0705
    sc4210_write_register (ViPipe, 0x300a, 0x2c); // [6] oen efsync [2] oen fsync

    sc4210_write_register (ViPipe, 0x4603, 0x00); // [0] data_fifo mipi mode
    sc4210_write_register (ViPipe, 0x4837, 0x1a); // [7:0] pclk period * 2  1009

    sc4210_write_register (ViPipe, 0x36ea, 0x3a); // 729Mbps->405Mbps

    // mipi end
    sc4210_write_register (ViPipe, 0x300f, 0x00);

    // exp&gain
    sc4210_write_register (ViPipe, 0x3e01, 0xba);
    sc4210_write_register (ViPipe, 0x3e02, 0xe0);
    sc4210_write_register (ViPipe, 0x3e08, 0x03);
    sc4210_write_register (ViPipe, 0x3e09, 0x20);
    sc4210_write_register (ViPipe, 0x3e06, 0x00);

    sc4210_write_register (ViPipe, 0x4418, 0x0d);
    sc4210_write_register (ViPipe, 0x3638, 0x27);

    sc4210_write_register (ViPipe, 0x3635, 0x40);
    sc4210_write_register (ViPipe, 0x363b, 0x06);

    // logic
    sc4210_write_register (ViPipe, 0x3622, 0xff);
    sc4210_write_register (ViPipe, 0x3633, 0x22);

    // 621Mbps
    sc4210_write_register (ViPipe, 0x36e9, 0xa4);
    sc4210_write_register (ViPipe, 0x36ea, 0x29);
    sc4210_write_register (ViPipe, 0x4837, 0x1a);

    // high temp
    sc4210_write_register (ViPipe, 0x39a0, 0x14); // 0x08,
    sc4210_write_register (ViPipe, 0x39a1, 0x28); // 0x10,
    sc4210_write_register (ViPipe, 0x39a2, 0x48); // 0x20,
    sc4210_write_register (ViPipe, 0x39a3, 0x70); // 0x40,
    sc4210_write_register (ViPipe, 0x39a4, 0x10); // 0x20,
    sc4210_write_register (ViPipe, 0x39a5, 0x10);
    sc4210_write_register (ViPipe, 0x39a6, 0x08);
    sc4210_write_register (ViPipe, 0x39a7, 0x04);
    sc4210_write_register (ViPipe, 0x39a8, 0x10); // 0x18,
    sc4210_write_register (ViPipe, 0x39a9, 0x28); // 0x30,
    sc4210_write_register (ViPipe, 0x39aa, 0x40);
    sc4210_write_register (ViPipe, 0x39ab, 0x68); // 0x60,
    sc4210_write_register (ViPipe, 0x39ac, 0x30); // 0x38,
    sc4210_write_register (ViPipe, 0x39ad, 0x20);
    sc4210_write_register (ViPipe, 0x39ae, 0x10);
    sc4210_write_register (ViPipe, 0x39af, 0x08);
    sc4210_write_register (ViPipe, 0x3980, 0x00);
    sc4210_write_register (ViPipe, 0x3981, 0x30);
    sc4210_write_register (ViPipe, 0x3982, 0x00);
    sc4210_write_register (ViPipe, 0x3983, 0x18);
    sc4210_write_register (ViPipe, 0x3984, 0x00);
    sc4210_write_register (ViPipe, 0x3985, 0x14);
    sc4210_write_register (ViPipe, 0x3986, 0x00);
    sc4210_write_register (ViPipe, 0x3987, 0x08);
    sc4210_write_register (ViPipe, 0x3988, 0x00);
    sc4210_write_register (ViPipe, 0x3989, 0x18);
    sc4210_write_register (ViPipe, 0x398a, 0x00);
    sc4210_write_register (ViPipe, 0x398b, 0x28);
    sc4210_write_register (ViPipe, 0x398c, 0x00);
    sc4210_write_register (ViPipe, 0x398d, 0x38);
    sc4210_write_register (ViPipe, 0x398e, 0x00);
    sc4210_write_register (ViPipe, 0x398f, 0x48);
    sc4210_write_register (ViPipe, 0x3990, 0x00);
    sc4210_write_register (ViPipe, 0x3991, 0x50);
    sc4210_write_register (ViPipe, 0x3992, 0x00);
    sc4210_write_register (ViPipe, 0x3993, 0x28);
    sc4210_write_register (ViPipe, 0x3994, 0x00);
    sc4210_write_register (ViPipe, 0x3995, 0x18);
    sc4210_write_register (ViPipe, 0x3996, 0x00);
    sc4210_write_register (ViPipe, 0x3997, 0x0c);
    sc4210_write_register (ViPipe, 0x3998, 0x00);
    sc4210_write_register (ViPipe, 0x3999, 0x18);
    sc4210_write_register (ViPipe, 0x399a, 0x00);
    sc4210_write_register (ViPipe, 0x399b, 0x30);
    sc4210_write_register (ViPipe, 0x399c, 0x00);
    sc4210_write_register (ViPipe, 0x399d, 0x50);
    sc4210_write_register (ViPipe, 0x399e, 0x00);
    sc4210_write_register (ViPipe, 0x399f, 0x80);
    sc4210_write_register (ViPipe, 0x39b9, 0x00);
    sc4210_write_register (ViPipe, 0x39ba, 0x66);
    sc4210_write_register (ViPipe, 0x39bb, 0x00);
    sc4210_write_register (ViPipe, 0x39bc, 0x00);
    sc4210_write_register (ViPipe, 0x39bd, 0x4b);
    sc4210_write_register (ViPipe, 0x39be, 0xc0);
    sc4210_write_register (ViPipe, 0x39bf, 0x00);
    sc4210_write_register (ViPipe, 0x39c0, 0x00);
    sc4210_write_register (ViPipe, 0x3933, 0x28);
    sc4210_write_register (ViPipe, 0x3934, 0xc0);
    sc4210_write_register (ViPipe, 0x3940, 0x60);
    sc4210_write_register (ViPipe, 0x3942, 0x08);
    sc4210_write_register (ViPipe, 0x3943, 0xd0);
    sc4210_write_register (ViPipe, 0x39c5, 0x41);

    sc4210_write_register (ViPipe, 0x3817, 0x20); // grp hold delay
    sc4210_write_register (ViPipe, 0x3314, 0x14);

    // 121.5M sysclk  607.5M cntclk  621Mbps x 4
    sc4210_write_register (ViPipe, 0x36e9, 0xa4);
    sc4210_write_register (ViPipe, 0x36ea, 0x29);
    sc4210_write_register (ViPipe, 0x36eb, 0x06);
    sc4210_write_register (ViPipe, 0x36ec, 0x03);
    sc4210_write_register (ViPipe, 0x36ed, 0x04);

    sc4210_write_register (ViPipe, 0x36f9, 0xa0);
    sc4210_write_register (ViPipe, 0x36fa, 0x37);
    sc4210_write_register (ViPipe, 0x36fb, 0x04);
    sc4210_write_register (ViPipe, 0x36fc, 0x00);
    sc4210_write_register (ViPipe, 0x36fd, 0x24);

    sc4210_write_register (ViPipe, 0x320c, 0x05);
    sc4210_write_register (ViPipe, 0x320d, 0x46);
    sc4210_write_register (ViPipe, 0x4501, 0xa4);
    sc4210_write_register (ViPipe, 0x4418, 0x1a);
    sc4210_write_register (ViPipe, 0x3638, 0x2a);
    sc4210_write_register (ViPipe, 0x3304, 0x20);
    sc4210_write_register (ViPipe, 0x331e, 0x19);
    sc4210_write_register (ViPipe, 0x3309, 0x50);
    sc4210_write_register (ViPipe, 0x331f, 0x49);
    sc4210_write_register (ViPipe, 0x3305, 0x00);
    sc4210_write_register (ViPipe, 0x3306, 0x78); // [68,--]
    sc4210_write_register (ViPipe, 0x330a, 0x00);
    sc4210_write_register (ViPipe, 0x330b, 0xd8); // [c8,d8]

    sc4210_write_register (ViPipe, 0x3301, 0x26); // [18,--]
    sc4210_write_register (ViPipe, 0x3320, 0x05);

    sc4210_write_register (ViPipe, 0x337a, 0x08); // count_keep/up
    sc4210_write_register (ViPipe, 0x337b, 0x10);
    sc4210_write_register (ViPipe, 0x33a3, 0x0c);

    sc4210_write_register (ViPipe, 0x3308, 0x10);
    sc4210_write_register (ViPipe, 0x3366, 0x92);
    sc4210_write_register (ViPipe, 0x4509, 0x10);

    // preprecharge
    sc4210_write_register (ViPipe, 0x3314, 0x84);
    sc4210_write_register (ViPipe, 0x330e, 0x20); // [f,2e]
    sc4210_write_register (ViPipe, 0x334c, 0x10);

    // 1008
    sc4210_write_register (ViPipe, 0x3636, 0x28);
    sc4210_write_register (ViPipe, 0x4418, 0x1a);

    // timing
    sc4210_write_register (ViPipe, 0x335d, 0x00); // [6]sp rst all1  1008
    sc4210_write_register (ViPipe, 0x3312, 0x02); // [7]sp dcg all1
    sc4210_write_register (ViPipe, 0x336c, 0xc2); // hcg rst all1 c2
    sc4210_write_register (ViPipe, 0x337e, 0x40);
    sc4210_write_register (ViPipe, 0x3338, 0x10); // [4] sp addr to rp addr disable

    sc4210_write_register (ViPipe, 0x4418, 0x18);
    sc4210_write_register (ViPipe, 0x3905, 0xd8); // one channel blc

    // logic
    sc4210_write_register (ViPipe, 0x3622, 0xff);
    sc4210_write_register (ViPipe, 0x3633, 0x42);

    // VC
    sc4210_write_register (ViPipe, 0x3220, 0x53);
    sc4210_write_register (ViPipe, 0x320e, 0x0b);
    sc4210_write_register (ViPipe, 0x320f, 0xb8);
    sc4210_write_register (ViPipe, 0x3250, 0x3f);
    sc4210_write_register (ViPipe, 0x4816, 0x11);

    sc4210_write_register (ViPipe, 0x3231, 0x01);
    sc4210_write_register (ViPipe, 0x3220, 0x50);
    sc4210_write_register (ViPipe, 0x3e0e, 0x6a);

    // modify fine gain map to 40~7f
    sc4210_write_register (ViPipe, 0x3625, 0x02);
    sc4210_write_register (ViPipe, 0x3636, 0x20);
    sc4210_write_register (ViPipe, 0x4418, 0x18);

    // modify mipi clk = 626.4M
    sc4210_write_register (ViPipe, 0x36e9, 0xb4);
    sc4210_write_register (ViPipe, 0x36ea, 0x23);
    sc4210_write_register (ViPipe, 0x4819, 0x40);
    sc4210_write_register (ViPipe, 0x4829, 0x01);
    sc4210_write_register (ViPipe, 0x4837, 0x1a);

    // internal logic
    sc4210_write_register (ViPipe, 0x366e, 0x04);
    sc4210_write_register (ViPipe, 0x360f, 0x05); // 0x360f[0]: 0x3622 logic enable
    sc4210_write_register (ViPipe, 0x367a, 0x40); // <1*dcg gain
    sc4210_write_register (ViPipe, 0x367b, 0x48); // >=2*dcg gain
    sc4210_write_register (ViPipe, 0x3671, 0xff); // 0x3622 logic value
    sc4210_write_register (ViPipe, 0x3672, 0x9f);
    sc4210_write_register (ViPipe, 0x3673, 0x9f); // 0x3680 read 0x3622 value

    sc4210_write_register (ViPipe, 0x3670, 0x48); // 0x3670[3]:0x3633 logic enable ---- 0x3670[6]:0x363a logic enable
    sc4210_write_register (ViPipe, 0x369c, 0x40); // <1*dcg gain
    sc4210_write_register (ViPipe, 0x369d, 0x48); // >=2*dcg gain
    sc4210_write_register (ViPipe, 0x3690, 0x43); // 0x3633 logic value
    sc4210_write_register (ViPipe, 0x3691, 0x54);
    sc4210_write_register (ViPipe, 0x3692, 0x66); // 0x3683 read 0x3633 value

    sc4210_write_register (ViPipe, 0x36a2, 0x40); // <1*dcg gain 0x363a logic, current value read out from 0x3686
    sc4210_write_register (ViPipe, 0x36a3, 0x48); // >=2*dcg gain
    sc4210_write_register (ViPipe, 0x3699, 0x8c);
    sc4210_write_register (ViPipe, 0x369a, 0x96);
    sc4210_write_register (ViPipe, 0x369b, 0x9f);

    sc4210_write_register (ViPipe, 0x36d0, 0x20); // 0x36d0[5]=1: 0x363e pwce auto en,  0x36cf auto pwce(FPN, horizontal lines/fullwell)
    sc4210_write_register (ViPipe, 0x36d1, 0x40); // gain 0
    sc4210_write_register (ViPipe, 0x36d2, 0x40); // gain 1
    sc4210_write_register (ViPipe, 0x36cc, 0x2c); // sel 0
    sc4210_write_register (ViPipe, 0x36cd, 0x30); // sel 1
    sc4210_write_register (ViPipe, 0x36ce, 0x30); // sel 2

    sc4210_write_register (ViPipe, 0x3364, 0x1e); // bit[4] comprst auto enable  bit[3] 0~gain 10~1f 1~ gain 20~3f  //0x33f2 readout
    sc4210_write_register (ViPipe, 0x3301, 0x24); // comprst sel0 when dcg off
    sc4210_write_register (ViPipe, 0x3393, 0x24); // comprst sel1 when dcg off
    sc4210_write_register (ViPipe, 0x3394, 0x24); // comprst sel2 when dcg off
    sc4210_write_register (ViPipe, 0x3395, 0x24); // comprst sel3 when dcg off
    sc4210_write_register (ViPipe, 0x3390, 0x08); // comprst gain0 when dcg off
    sc4210_write_register (ViPipe, 0x3391, 0x08); // comprst gain1 when dcg off
    sc4210_write_register (ViPipe, 0x3392, 0x08); // comprst gain2 when dcg off
    sc4210_write_register (ViPipe, 0x3399, 0x1c); // comprst sel0 when dcg on
    sc4210_write_register (ViPipe, 0x339a, 0x26); // comprst sel1 when dcg on
    sc4210_write_register (ViPipe, 0x339b, 0x1d); // comprst sel2 when dcg on
    sc4210_write_register (ViPipe, 0x339c, 0x26); // comprst sel3 when dcg on
    sc4210_write_register (ViPipe, 0x3396, 0x08); // comprst gain0 when dcg on
    sc4210_write_register (ViPipe, 0x3397, 0x38); // comprst gain1 when dcg on
    sc4210_write_register (ViPipe, 0x3398, 0x3c); // comprst gain2 when dcg on[40,4f]=1d,[50,5f]=26,[60,6f]=1d,[70,7f]=26

    // dpc
    sc4210_write_register (ViPipe, 0x33e0, 0xa0);
    sc4210_write_register (ViPipe, 0x33e1, 0x08);
    sc4210_write_register (ViPipe, 0x33e2, 0x02);
    sc4210_write_register (ViPipe, 0x33e3, 0x06);
    sc4210_write_register (ViPipe, 0x33e4, 0x07);
    sc4210_write_register (ViPipe, 0x33e5, 0x02);
    sc4210_write_register (ViPipe, 0x33e6, 0x04);
    sc4210_write_register (ViPipe, 0x33e7, 0x08);
    sc4210_write_register (ViPipe, 0x33e8, 0x00);
    sc4210_write_register (ViPipe, 0x33e9, 0x06);
    sc4210_write_register (ViPipe, 0x33ea, 0x10);
    sc4210_write_register (ViPipe, 0x33eb, 0x00);
    sc4210_write_register (ViPipe, 0x33ec, 0x04);
    sc4210_write_register (ViPipe, 0x33ed, 0x10);
    sc4210_write_register (ViPipe, 0x33ee, 0xa0);
    sc4210_write_register (ViPipe, 0x33ef, 0x08);
    sc4210_write_register (ViPipe, 0x33f4, 0x02);
    sc4210_write_register (ViPipe, 0x33f5, 0x06);
    sc4210_write_register (ViPipe, 0x33f6, 0x07);
    sc4210_write_register (ViPipe, 0x33f7, 0x02);
    sc4210_write_register (ViPipe, 0x33f8, 0x04);
    sc4210_write_register (ViPipe, 0x33f9, 0x08);
    sc4210_write_register (ViPipe, 0x33fa, 0x00);
    sc4210_write_register (ViPipe, 0x33fb, 0x06);
    sc4210_write_register (ViPipe, 0x33fc, 0x10);
    sc4210_write_register (ViPipe, 0x33fd, 0x00);
    sc4210_write_register (ViPipe, 0x33fe, 0x04);
    sc4210_write_register (ViPipe, 0x33ff, 0x10);
    sc4210_write_register (ViPipe, 0x5784, 0x0b);
    sc4210_write_register (ViPipe, 0x5787, 0x07);
    sc4210_write_register (ViPipe, 0x5788, 0x06);
    sc4210_write_register (ViPipe, 0x5789, 0x02);
    sc4210_write_register (ViPipe, 0x578a, 0x08);
    sc4210_write_register (ViPipe, 0x578b, 0x04);
    sc4210_write_register (ViPipe, 0x578c, 0x02);
    sc4210_write_register (ViPipe, 0x57c4, 0x0b);
    sc4210_write_register (ViPipe, 0x57c7, 0x07);
    sc4210_write_register (ViPipe, 0x57c8, 0x06);
    sc4210_write_register (ViPipe, 0x57c9, 0x02);
    sc4210_write_register (ViPipe, 0x57ca, 0x08);
    sc4210_write_register (ViPipe, 0x57cb, 0x04);
    sc4210_write_register (ViPipe, 0x57cc, 0x02);
    sc4210_write_register (ViPipe, 0x5000, 0x06);
    sc4210_write_register (ViPipe, 0x3e26, 0x40); // time point of blc dithering

    sc4210_write_register (ViPipe, 0x4418, 0x16); // fullwell
    sc4210_write_register (ViPipe, 0x3638, 0x28); // black sun, tempN
    sc4210_write_register (ViPipe, 0x3306, 0x78); // [hl,bs][1,6a,7a]     [2,68,7a]   [3,6a,7a]   0x330b=0xe0

    sc4210_write_register (ViPipe, 0x330b, 0xe0); // [bs,to][1,d6,ea]     [2,d6,ea]   [3,d6,ea]   0x3306=0x78

    // vco=27/2*5*9=607.5
    // sysclk=vco/5=121.5
    // countclk=vco/1=607.5
    // countclk:sysclk=5:1
    sc4210_write_register (ViPipe, 0x5784, 0x10);
    sc4210_write_register (ViPipe, 0x5785, 0x08);
    sc4210_write_register (ViPipe, 0x5787, 0x06);
    sc4210_write_register (ViPipe, 0x5788, 0x06);
    sc4210_write_register (ViPipe, 0x5789, 0x00);
    sc4210_write_register (ViPipe, 0x578a, 0x06);
    sc4210_write_register (ViPipe, 0x578b, 0x06);
    sc4210_write_register (ViPipe, 0x578c, 0x00);
    sc4210_write_register (ViPipe, 0x5790, 0x10);
    sc4210_write_register (ViPipe, 0x5791, 0x10);
    sc4210_write_register (ViPipe, 0x5792, 0x00);
    sc4210_write_register (ViPipe, 0x5793, 0x10);
    sc4210_write_register (ViPipe, 0x5794, 0x10);
    sc4210_write_register (ViPipe, 0x5795, 0x00);
    sc4210_write_register (ViPipe, 0x57c4, 0x10);
    sc4210_write_register (ViPipe, 0x57c5, 0x08);
    sc4210_write_register (ViPipe, 0x57c7, 0x06);
    sc4210_write_register (ViPipe, 0x57c8, 0x06);
    sc4210_write_register (ViPipe, 0x57c9, 0x00);
    sc4210_write_register (ViPipe, 0x57ca, 0x06);
    sc4210_write_register (ViPipe, 0x57cb, 0x06);
    sc4210_write_register (ViPipe, 0x57cc, 0x00);
    sc4210_write_register (ViPipe, 0x57d0, 0x10);
    sc4210_write_register (ViPipe, 0x57d1, 0x10);
    sc4210_write_register (ViPipe, 0x57d2, 0x00);
    sc4210_write_register (ViPipe, 0x57d3, 0x10);
    sc4210_write_register (ViPipe, 0x57d4, 0x10);
    sc4210_write_register (ViPipe, 0x57d5, 0x00);
    sc4210_write_register (ViPipe, 0x33e0, 0xa0);
    sc4210_write_register (ViPipe, 0x33e1, 0x08);
    sc4210_write_register (ViPipe, 0x33e2, 0x00);
    sc4210_write_register (ViPipe, 0x33e3, 0x10);
    sc4210_write_register (ViPipe, 0x33e4, 0x10);
    sc4210_write_register (ViPipe, 0x33e5, 0x00);
    sc4210_write_register (ViPipe, 0x33e6, 0x10);
    sc4210_write_register (ViPipe, 0x33e7, 0x10);
    sc4210_write_register (ViPipe, 0x33e8, 0x00);
    sc4210_write_register (ViPipe, 0x33e9, 0x10);
    sc4210_write_register (ViPipe, 0x33ea, 0x16);
    sc4210_write_register (ViPipe, 0x33eb, 0x00);
    sc4210_write_register (ViPipe, 0x33ec, 0x10);
    sc4210_write_register (ViPipe, 0x33ed, 0x18);
    sc4210_write_register (ViPipe, 0x33ee, 0xa0);
    sc4210_write_register (ViPipe, 0x33ef, 0x08);
    sc4210_write_register (ViPipe, 0x33f4, 0x00);
    sc4210_write_register (ViPipe, 0x33f5, 0x10);
    sc4210_write_register (ViPipe, 0x33f6, 0x10);
    sc4210_write_register (ViPipe, 0x33f7, 0x00);
    sc4210_write_register (ViPipe, 0x33f8, 0x10);
    sc4210_write_register (ViPipe, 0x33f9, 0x10);
    sc4210_write_register (ViPipe, 0x33fa, 0x00);
    sc4210_write_register (ViPipe, 0x33fb, 0x10);
    sc4210_write_register (ViPipe, 0x33fc, 0x16);
    sc4210_write_register (ViPipe, 0x33fd, 0x00);
    sc4210_write_register (ViPipe, 0x33fe, 0x10);
    sc4210_write_register (ViPipe, 0x33ff, 0x18);

    sc4210_write_register (ViPipe, 0x3905, 0x98); // channel blc
    sc4210_write_register (ViPipe, 0x4509, 0x08); // init code
    sc4210_write_register (ViPipe, 0x36ed, 0x0c); // local bias en, pwdn current
    sc4210_write_register (ViPipe, 0x36fd, 0x2c);

    // high temperature
    sc4210_write_register (ViPipe, 0x3933, 0x1f); // m
    sc4210_write_register (ViPipe, 0x3934, 0xff);
    sc4210_write_register (ViPipe, 0x3942, 0x04);
    sc4210_write_register (ViPipe, 0x393e, 0x01);
    sc4210_write_register (ViPipe, 0x39bc, 0x00); // LeftK
    sc4210_write_register (ViPipe, 0x39bd, 0x58);
    sc4210_write_register (ViPipe, 0x39be, 0xc0);
    sc4210_write_register (ViPipe, 0x39a0, 0x14);
    sc4210_write_register (ViPipe, 0x39a1, 0x28);
    sc4210_write_register (ViPipe, 0x39a2, 0x48);
    sc4210_write_register (ViPipe, 0x39a3, 0x70);
    sc4210_write_register (ViPipe, 0x39a4, 0x18); // posH
    sc4210_write_register (ViPipe, 0x39a5, 0x10);
    sc4210_write_register (ViPipe, 0x39a5, 0x08);
    sc4210_write_register (ViPipe, 0x39a5, 0x04);
    sc4210_write_register (ViPipe, 0x3980, 0x00); // KH
    sc4210_write_register (ViPipe, 0x3981, 0x30);
    sc4210_write_register (ViPipe, 0x3982, 0x00);
    sc4210_write_register (ViPipe, 0x3983, 0x2c);
    sc4210_write_register (ViPipe, 0x3984, 0x00);
    sc4210_write_register (ViPipe, 0x3985, 0x15);
    sc4210_write_register (ViPipe, 0x3986, 0x00);
    sc4210_write_register (ViPipe, 0x3987, 0x10);
    sc4210_write_register (ViPipe, 0x3988, 0x00);
    sc4210_write_register (ViPipe, 0x3989, 0x30);
    sc4210_write_register (ViPipe, 0x398a, 0x00);
    sc4210_write_register (ViPipe, 0x398b, 0x28);
    sc4210_write_register (ViPipe, 0x398c, 0x00);
    sc4210_write_register (ViPipe, 0x398d, 0x30);
    sc4210_write_register (ViPipe, 0x398e, 0x00);
    sc4210_write_register (ViPipe, 0x398f, 0x70);
    sc4210_write_register (ViPipe, 0x39b9, 0x00); // TopV
    sc4210_write_register (ViPipe, 0x39ba, 0x00);
    sc4210_write_register (ViPipe, 0x39bb, 0x00);
    sc4210_write_register (ViPipe, 0x39a8, 0x01); // POSV
    sc4210_write_register (ViPipe, 0x39a9, 0x14);
    sc4210_write_register (ViPipe, 0x39aa, 0x30);
    sc4210_write_register (ViPipe, 0x39ab, 0x50);
    sc4210_write_register (ViPipe, 0x39ac, 0x40);
    sc4210_write_register (ViPipe, 0x39ad, 0x20);
    sc4210_write_register (ViPipe, 0x39ae, 0x10);
    sc4210_write_register (ViPipe, 0x39af, 0x08);
    sc4210_write_register (ViPipe, 0x3990, 0x05); // KV
    sc4210_write_register (ViPipe, 0x3991, 0x00);
    sc4210_write_register (ViPipe, 0x3992, 0x00);
    sc4210_write_register (ViPipe, 0x3993, 0x60);
    sc4210_write_register (ViPipe, 0x3994, 0x00);
    sc4210_write_register (ViPipe, 0x3995, 0x30);
    sc4210_write_register (ViPipe, 0x3996, 0x00);
    sc4210_write_register (ViPipe, 0x3997, 0x10);
    sc4210_write_register (ViPipe, 0x3998, 0x00);
    sc4210_write_register (ViPipe, 0x3999, 0x1c);
    sc4210_write_register (ViPipe, 0x399a, 0x00);
    sc4210_write_register (ViPipe, 0x399b, 0x48);
    sc4210_write_register (ViPipe, 0x399c, 0x00);
    sc4210_write_register (ViPipe, 0x399d, 0x90);
    sc4210_write_register (ViPipe, 0x399e, 0x00);
    sc4210_write_register (ViPipe, 0x399f, 0xc0);
    sc4210_write_register (ViPipe, 0x3990, 0x0a); // KV
    sc4210_write_register (ViPipe, 0x39aa, 0x28); // PosV
    sc4210_write_register (ViPipe, 0x39ac, 0x30); // PosV

    sc4210_write_register (ViPipe, 0x363b, 0x03); // vrefh 3.0v
    sc4210_write_register (ViPipe, 0x3635, 0x20); // txvdd 2.8v

    // auto precharge
    sc4210_write_register (ViPipe, 0x335d, 0x20);
    sc4210_write_register (ViPipe, 0x330e, 0x20); // [f,2e]
    sc4210_write_register (ViPipe, 0x3367, 0x08);
    sc4210_write_register (ViPipe, 0x3368, 0x0b); // vts-{0x3e23,0x3e24}
    sc4210_write_register (ViPipe, 0x3369, 0x04);
    sc4210_write_register (ViPipe, 0x336a, 0x16); // (vts-{0x3e23,0x3e24})*2
    sc4210_write_register (ViPipe, 0x336b, 0x08);
    sc4210_write_register (ViPipe, 0x3362, 0x72); // short exp tx en
    sc4210_write_register (ViPipe, 0x3360, 0x20); // short exp tx width
    sc4210_write_register (ViPipe, 0x4819, 0x40); // hs_zero  hdr Compression timing
    sc4210_write_register (ViPipe, 0x4829, 0x01); // hs_exi

    sc4210_write_register (ViPipe, 0x4837, 0x1b); // [7:0] pclk period*2

    sc4210_write_register (ViPipe, 0x550f, 0x20); // opt
    sc4210_write_register (ViPipe, 0x4407, 0xb0);

    sc4210_write_register (ViPipe, 0x3253, 0x10); // ramp local buffer margin

    // init
    sc4210_write_register (ViPipe, 0x3e00, 0x01); // max long expsoure=0x15fe, max short expsoure=160 --> <16:1
    sc4210_write_register (ViPipe, 0x3e01, 0x5f);
    sc4210_write_register (ViPipe, 0x3e02, 0xe0);
    sc4210_write_register (ViPipe, 0x3e03, 0x0b);
    sc4210_write_register (ViPipe, 0x3e04, 0x16);
    sc4210_write_register (ViPipe, 0x3e05, 0x00);
    sc4210_write_register (ViPipe, 0x3e23, 0x00);
    sc4210_write_register (ViPipe, 0x3e24, 0xb4);
    sc4210_write_register (ViPipe, 0x3e06, 0x00);
    sc4210_write_register (ViPipe, 0x3e07, 0x80);
    sc4210_write_register (ViPipe, 0x3e08, 0x03);
    sc4210_write_register (ViPipe, 0x3e09, 0x40);
    sc4210_write_register (ViPipe, 0x3e10, 0x00);
    sc4210_write_register (ViPipe, 0x3e11, 0x80);
    sc4210_write_register (ViPipe, 0x3e12, 0x03);
    sc4210_write_register (ViPipe, 0x3e13, 0x40);

    sc4210_write_register (ViPipe, 0x36e9, 0x34); // pll1 enable
    sc4210_write_register (ViPipe, 0x36f9, 0x20); // pll2 enable
    sc4210_default_reg_init(ViPipe);
    sc4210_write_register (ViPipe, 0x0100, 0x01);

    printf("===SC4210 1440P 30fps 10bit WDR Init OK!===\n");

    return;
}
