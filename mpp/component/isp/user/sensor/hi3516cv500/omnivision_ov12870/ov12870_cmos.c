/*
* Copyright (C) Hisilicon Technologies Co., Ltd. 2012-2019. All rights reserved.
* Description:
* Author: Hisilicon multimedia software group
* Create: 2011/06/28
*/

#if !defined(__OV12870_CMOS_H_)
#define __OV12870_CMOS_H_

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "hi_comm_sns.h"
#include "hi_comm_video.h"
#include "hi_sns_ctrl.h"
#include "mpi_isp.h"
#include "mpi_ae.h"
#include "mpi_awb.h"

#include "ov12870_cmos_ex.h"
#include "ov12870_cmos_priv.h"
#include "isp_math_utils.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */


#define OV12870_ID 12870

#define HIGH_8BITS(x) (((x) & 0xff00) >> 8)
#define LOW_8BITS(x)  ((x) & 0x00ff)

ISP_SNS_STATE_S *g_pastOv12870[ISP_MAX_PIPE_NUM] = {HI_NULL};

#define OV12870_SENSOR_GET_CTX(dev, pstCtx)   (pstCtx = g_pastOv12870[dev])
#define OV12870_SENSOR_SET_CTX(dev, pstCtx)   (g_pastOv12870[dev] = pstCtx)
#define OV12870_SENSOR_RESET_CTX(dev)         (g_pastOv12870[dev] = HI_NULL)

static HI_U32 g_au32InitExposure[ISP_MAX_PIPE_NUM]  = {0};
static HI_U32 g_au32LinesPer500ms[ISP_MAX_PIPE_NUM] = {0};
static HI_U16 g_au16InitWBGain[ISP_MAX_PIPE_NUM][3] = {{0}};
static HI_U16 g_au16SampleRgain[ISP_MAX_PIPE_NUM] = {0};
static HI_U16 g_au16SampleBgain[ISP_MAX_PIPE_NUM] = {0};
static HI_U32 g_au32Again[ISP_MAX_PIPE_NUM] = {1024};

ISP_SNS_COMMBUS_U g_aunOv12870BusInfo[ISP_MAX_PIPE_NUM] = {
    [0] = { .s8I2cDev = 0},
    [1 ... ISP_MAX_PIPE_NUM - 1] = { .s8I2cDev = -1}
};

/****************************************************************************
 * extern                                                                   *
 ****************************************************************************/
extern unsigned char ov12870_i2c_addr;
extern unsigned int  ov12870_addr_byte;
extern unsigned int  ov12870_data_byte;

const OV12870_VIDEO_MODE_TBL_S g_astOv12870ModeTbl[OV12870_MODE_BUTT] = {
    {0x0340, 0, 8, 36, 240, 0, "720P_10BIT_240FPS"  },              /* */
    {0x0682, 0, 8, 36, 120, 0, "2K1K_10BIT_120FPS"  },               /*  */
    {0x0d06, 0, 8, 36, 30, 0,  "4K2K_10BIT_30FPS"  },               /*  */
    {0x0d06, 0, 8, 36, 30,  0, "4K3K_10BIT_30FPS"  },               /*  */
};


extern void ov12870_init(VI_PIPE ViPipe);
extern void ov12870_exit(VI_PIPE ViPipe);
extern void ov12870_standby(VI_PIPE ViPipe);
extern void ov12870_restart(VI_PIPE ViPipe);
extern void ov12870_mirror_flip(VI_PIPE ViPipe, ISP_SNS_MIRRORFLIP_TYPE_E eSnsMirrorFlip);
extern int  ov12870_write_register(VI_PIPE ViPipe, int addr, int data);
extern int  ov12870_read_register(VI_PIPE ViPipe, int addr);

/****************************************************************************
 * local variables                                                            *
 ****************************************************************************/
#define OV12870_FULL_LINES_MAX  (0xFFFF)

#define OV12870_ERR_MODE_PRINT(pstSensorImageMode)\
    do{\
        ISP_ERR_TRACE("Not support! Width:%d, Height:%d, Fps:%f, SnsMode:%d\n",\
                  pstSensorImageMode->u16Width,  \
                  pstSensorImageMode->u16Height, \
                  pstSensorImageMode->f32Fps,    \
                  pstSensorImageMode->u8SnsMode);\
    }while(0)

static HI_S32 cmos_get_ae_default(VI_PIPE ViPipe, AE_SENSOR_DEFAULT_S *pstAeSnsDft)
{
    HI_U32 u32Fll = 0;
    HI_FLOAT f32MaxFps = 30;

    ISP_SNS_STATE_S *pstSnsState = HI_NULL;

    CMOS_CHECK_POINTER(pstAeSnsDft);
    OV12870_SENSOR_GET_CTX(ViPipe, pstSnsState);
    CMOS_CHECK_POINTER(pstSnsState);

    u32Fll = g_astOv12870ModeTbl[pstSnsState->u8ImgMode].u32VMax;
    f32MaxFps = g_astOv12870ModeTbl[pstSnsState->u8ImgMode].f32MaxFps;
    pstAeSnsDft->stIntTimeAccu.f32Offset = g_astOv12870ModeTbl[pstSnsState->u8ImgMode].u32Offset;
    pstSnsState->u32FLStd = u32Fll;
    pstAeSnsDft->u32MaxIntTime = pstSnsState->u32FLStd - g_astOv12870ModeTbl[pstSnsState->u8ImgMode].u32ExpLineLimit;

    pstAeSnsDft->u32FullLinesStd = pstSnsState->u32FLStd;
    pstAeSnsDft->u32FlickerFreq = 0;
    pstAeSnsDft->u32HmaxTimes = (1000000 * 1000) / DIV_0_TO_1_FLOAT(u32Fll * f32MaxFps);

    pstAeSnsDft->stIntTimeAccu.enAccuType = AE_ACCURACY_LINEAR;
    pstAeSnsDft->stIntTimeAccu.f32Accuracy = 1;

    pstAeSnsDft->stAgainAccu.enAccuType = AE_ACCURACY_TABLE;
    pstAeSnsDft->stAgainAccu.f32Accuracy = 0.3;

    pstAeSnsDft->stDgainAccu.enAccuType = AE_ACCURACY_TABLE;
    pstAeSnsDft->stDgainAccu.f32Accuracy = 0.3;

    pstAeSnsDft->u32ISPDgainShift = 8;
    pstAeSnsDft->u32MinISPDgainTarget = 1 << pstAeSnsDft->u32ISPDgainShift;
    pstAeSnsDft->u32MaxISPDgainTarget = 4 << pstAeSnsDft->u32ISPDgainShift;

    memcpy(&pstAeSnsDft->stPirisAttr, &gstPirisAttr, sizeof(ISP_PIRIS_ATTR_S));
    pstAeSnsDft->enMaxIrisFNO = ISP_IRIS_F_NO_1_4;
    pstAeSnsDft->enMinIrisFNO = ISP_IRIS_F_NO_5_6;

    pstAeSnsDft->bAERouteExValid = HI_FALSE;
    pstAeSnsDft->stAERouteAttr.u32TotalNum = 0;
    pstAeSnsDft->stAERouteAttrEx.u32TotalNum = 0;

    if (g_au32InitExposure[ViPipe] == 0) {
        pstAeSnsDft->u32InitExposure = 1000000;
    } else {
        pstAeSnsDft->u32InitExposure = g_au32InitExposure[ViPipe];;
    }

    if (g_au32LinesPer500ms[ViPipe] == 0) {
        pstAeSnsDft->u32LinesPer500ms = ((HI_U64)(u32Fll * f32MaxFps)) >> 1;
    } else {
        pstAeSnsDft->u32LinesPer500ms = g_au32LinesPer500ms[ViPipe];
    }

    switch (pstSnsState->enWDRMode) {
        default:
        case WDR_MODE_NONE: {
            pstAeSnsDft->au8HistThresh[0] = 0xd;
            pstAeSnsDft->au8HistThresh[1] = 0x28;
            pstAeSnsDft->au8HistThresh[2] = 0x60;
            pstAeSnsDft->au8HistThresh[3] = 0x80;

            pstAeSnsDft->u8AeCompensation = 0x2D;
            pstAeSnsDft->enAeExpMode = AE_EXP_HIGHLIGHT_PRIOR;

            pstAeSnsDft->u32MinIntTime = g_astOv12870ModeTbl[pstSnsState->u8ImgMode].u32ExpLineMin;
            pstAeSnsDft->u32MaxIntTimeTarget = 65535;
            pstAeSnsDft->u32MinIntTimeTarget = pstAeSnsDft->u32MinIntTime;

            pstAeSnsDft->u32MaxAgain = 15872 * 2;/*Again15.5x * Dgain2x*/
            pstAeSnsDft->u32MinAgain = 1024;
            pstAeSnsDft->u32MaxAgainTarget = pstAeSnsDft->u32MaxAgain;
            pstAeSnsDft->u32MinAgainTarget = pstAeSnsDft->u32MinAgain;

            pstAeSnsDft->u32MaxDgain = 1024;  /* if Dgain enable,please set ispdgain bigger than 1*/
            pstAeSnsDft->u32MinDgain = 1024;
            pstAeSnsDft->u32MaxDgainTarget = pstAeSnsDft->u32MaxDgain;
            pstAeSnsDft->u32MinDgainTarget = pstAeSnsDft->u32MinDgain;

            break;
        }
    }

    return HI_SUCCESS;
}


/* the function of sensor set fps */
static HI_VOID cmos_fps_set(VI_PIPE ViPipe, HI_FLOAT f32Fps, AE_SENSOR_DEFAULT_S *pstAeSnsDft)
{
    HI_FLOAT f32MaxFps;
    HI_U32 u32Lines;
    ISP_SNS_STATE_S *pstSnsState = HI_NULL;

    CMOS_CHECK_POINTER_VOID(pstAeSnsDft);
    OV12870_SENSOR_GET_CTX(ViPipe, pstSnsState);
    CMOS_CHECK_POINTER_VOID(pstSnsState);

    f32MaxFps = g_astOv12870ModeTbl[pstSnsState->u8ImgMode].f32MaxFps;
    u32Lines = g_astOv12870ModeTbl[pstSnsState->u8ImgMode].u32VMax * (f32MaxFps / DIV_0_TO_1_FLOAT(f32Fps));
    pstSnsState->u32FLStd = u32Lines;
    pstAeSnsDft->u32MaxIntTime = pstSnsState->u32FLStd - g_astOv12870ModeTbl[pstSnsState->u8ImgMode].u32ExpLineLimit;

    /* SHR 16bit, So limit full_lines as 0xFFFF */
    if (f32Fps > f32MaxFps || u32Lines > OV12870_FULL_LINES_MAX) {
        ISP_ERR_TRACE("Not support Fps: %f\n", f32Fps);
        return;
    }

    pstAeSnsDft->f32Fps = f32Fps;
    //pstAeSnsDft->u32LinesPer500ms = pstSnsState->u32FLStd * f32Fps / 2;
    pstAeSnsDft->u32LinesPer500ms = g_astOv12870ModeTbl[pstSnsState->u8ImgMode].u32VMax*f32MaxFps/2;
    pstAeSnsDft->u32FullLinesStd = pstSnsState->u32FLStd;

    pstSnsState->au32FL[0] = pstSnsState->u32FLStd;
    pstAeSnsDft->u32FullLines = pstSnsState->au32FL[0];
    pstAeSnsDft->u32HmaxTimes = (1000000 * 1000) / DIV_0_TO_1_FLOAT(pstSnsState->u32FLStd * f32Fps);

    pstSnsState->astRegsInfo[0].astI2cData[7].u32Data = LOW_8BITS(pstSnsState->au32FL[0]);
    pstSnsState->astRegsInfo[0].astI2cData[8].u32Data = HIGH_8BITS(pstSnsState->au32FL[0]);

    return;
}

#define FSCLK (0xb4)   //get from 'ov12870_timing_v4.xlsx'
#define EXPTIMEMAX (600)  //calculated maxtime is 6080s, 600s is enough
static HI_VOID cmos_slow_framerate_set(VI_PIPE ViPipe, HI_U32 u32FullLines,
                                       AE_SENSOR_DEFAULT_S *pstAeSnsDft)
{
    ISP_SNS_STATE_S *pstSnsState = HI_NULL;
    HI_FLOAT fExpoTime;
    HI_U32 u32RegExpo;
    HI_U8 u8R3412, u8R3413, u8R3414, u8R3415;

    CMOS_CHECK_POINTER_VOID(pstAeSnsDft);
    OV12870_SENSOR_GET_CTX(ViPipe, pstSnsState);
    CMOS_CHECK_POINTER_VOID(pstSnsState);

    if (u32FullLines > OV12870_FULL_LINES_MAX) {
        fExpoTime = ((HI_FLOAT)pstAeSnsDft->u32HmaxTimes / (1000000*1000)) * u32FullLines;
        if (fExpoTime > EXPTIMEMAX) {
            ISP_ERR_TRACE("Not support long exposure time %f !\n", fExpoTime);
            return;
        }
    }
    pstSnsState->au32FL[0] = u32FullLines;
    pstAeSnsDft->u32FullLines = pstSnsState->au32FL[0];
    pstAeSnsDft->u32MaxIntTime = pstSnsState->au32FL[0] - g_astOv12870ModeTbl[pstSnsState->u8ImgMode].u32ExpLineLimit;


    if (u32FullLines > OV12870_FULL_LINES_MAX) {
        u32RegExpo = (HI_U32)(fExpoTime * FSCLK * 0xf424 / 0x10);
        u8R3412 = u32RegExpo / 0x1000000;
        u8R3413 = (u32RegExpo - u8R3412 * 0x1000000) / 0x10000;
        u8R3414 = (u32RegExpo - u8R3412 * 0x1000000 - u8R3413 * 0x10000) / 0x100;
        u8R3415 = u32RegExpo - u8R3412 * 0x1000000 - u8R3413 * 0x10000 - u8R3414 * 0x100;
        pstSnsState->astRegsInfo[0].astI2cData[9].u32Data = 0x0; //write 0x4202 0x1
        pstSnsState->astRegsInfo[0].astI2cData[10].u32Data = 0x1; //write 0x3410 0x1
        pstSnsState->astRegsInfo[0].astI2cData[11].u32Data = u8R3412; //write 0x3412 u8R3412
        pstSnsState->astRegsInfo[0].astI2cData[12].u32Data = u8R3413; //write 0x3413 u8R3413
        pstSnsState->astRegsInfo[0].astI2cData[13].u32Data = u8R3414; //write 0x3414 u8R3414
        pstSnsState->astRegsInfo[0].astI2cData[14].u32Data = u8R3415; //write 0x3415 u8R3415

        return;
    } else {
        pstSnsState->astRegsInfo[0].astI2cData[9].u32Data = 0x0; //write 0x4202 0x0
        pstSnsState->astRegsInfo[0].astI2cData[10].u32Data = 0x0; //write 0x3410 0x0
        pstSnsState->astRegsInfo[0].astI2cData[11].u32Data = 0; //write 0x3412 u8R3412
        pstSnsState->astRegsInfo[0].astI2cData[12].u32Data = 0; //write 0x3413 u8R3413
        pstSnsState->astRegsInfo[0].astI2cData[13].u32Data = 0; //write 0x3414 u8R3414
        pstSnsState->astRegsInfo[0].astI2cData[14].u32Data = 0; //write 0x3415 u8R3415
    }
    pstSnsState->astRegsInfo[0].astI2cData[7].u32Data = LOW_8BITS(u32FullLines);
    pstSnsState->astRegsInfo[0].astI2cData[8].u32Data = HIGH_8BITS(u32FullLines);

    return;
}

/* while isp notify ae to update sensor regs, ae call these funcs. */
static HI_VOID cmos_inttime_update(VI_PIPE ViPipe, HI_U32 u32IntTime)
{
    ISP_SNS_STATE_S *pstSnsState = HI_NULL;

    OV12870_SENSOR_GET_CTX(ViPipe, pstSnsState);
    CMOS_CHECK_POINTER_VOID(pstSnsState);

    pstSnsState->astRegsInfo[0].astI2cData[0].u32Data = LOW_8BITS(u32IntTime);
    pstSnsState->astRegsInfo[0].astI2cData[1].u32Data = HIGH_8BITS(u32IntTime);
    pstSnsState->astRegsInfo[0].astI2cData[2].u32Data = HIGHER_8BITS(u32IntTime);

    return;
}

static HI_U32 gain_table[64] = {
    1024, 1088, 1152, 1216, 1280, 1344, 1408, 1472, 1536, 1600, 1664, 1728, 1792, 1856, 1920, 1984, 2048, 2176, 2304, 2432,
    2560, 2688, 2816, 2944, 3072, 3200, 3328, 3456, 3584, 3712, 3840, 3968, 4096, 4352, 4608, 4864, 5120, 5376, 5632, 5888,
    6144, 6400, 6656, 6912, 7168, 7424, 7680, 7936, 8192, 8704, 9216, 9728, 10240, 10752, 11264, 11776, 12288, 12800, 13312,
    13824, 14336, 14848, 15360, 15872
};


static HI_VOID cmos_again_calc_table(VI_PIPE ViPipe, HI_U32 *pu32AgainLin, HI_U32 *pu32AgainDb)
{
    int i;

    CMOS_CHECK_POINTER_VOID(pu32AgainLin);
    CMOS_CHECK_POINTER_VOID(pu32AgainDb);

    if (*pu32AgainLin >= gain_table[63]) {
#if 0
        *pu32AgainLin = gain_table[63];
        g_au32Again[ViPipe] = *pu32AgainLin;
        *pu32AgainDb = (gain_table[63] >> 3);
#else
        *pu32AgainDb = (*pu32AgainLin >> 3);
        *pu32AgainLin = ((*pu32AgainLin >> 3) << 3);
        g_au32Again[ViPipe] = *pu32AgainLin;
#endif
        return;
    }

    for (i = 1; i < 64; i++) {
        if (*pu32AgainLin < gain_table[i]) {
            *pu32AgainLin = gain_table[i - 1];
            g_au32Again[ViPipe] = *pu32AgainLin;
            *pu32AgainDb = (gain_table[i - 1] >> 3);
            break;
        }
    }
    return;
}

static HI_VOID cmos_dgain_calc_table(VI_PIPE ViPipe, HI_U32 *pu32DgainLin, HI_U32 *pu32DgainDb)
{


    return;
}

static HI_VOID cmos_gains_update(VI_PIPE ViPipe, HI_U32 u32Again, HI_U32 u32Dgain)
{
    ISP_SNS_STATE_S *pstSnsState = HI_NULL;

    OV12870_SENSOR_GET_CTX(ViPipe, pstSnsState);
    CMOS_CHECK_POINTER_VOID(pstSnsState);

    pstSnsState->astRegsInfo[0].astI2cData[3].u32Data = (u32Again & 0xFF);
    pstSnsState->astRegsInfo[0].astI2cData[4].u32Data = ((u32Again >> 8) & 0xf);
    pstSnsState->astRegsInfo[0].astI2cData[5].u32Data = (u32Again & 0xFF);
    pstSnsState->astRegsInfo[0].astI2cData[6].u32Data = ((u32Again >> 8) & 0xf);

    return;
}



static HI_S32 cmos_init_ae_exp_function(AE_SENSOR_EXP_FUNC_S *pstExpFuncs)
{
    CMOS_CHECK_POINTER(pstExpFuncs);

    memset(pstExpFuncs, 0, sizeof(AE_SENSOR_EXP_FUNC_S));

    pstExpFuncs->pfn_cmos_get_ae_default    = cmos_get_ae_default;
    pstExpFuncs->pfn_cmos_fps_set           = cmos_fps_set;
    pstExpFuncs->pfn_cmos_slow_framerate_set = cmos_slow_framerate_set;
    pstExpFuncs->pfn_cmos_inttime_update    = cmos_inttime_update;
    pstExpFuncs->pfn_cmos_gains_update      = cmos_gains_update;
    pstExpFuncs->pfn_cmos_again_calc_table  = cmos_again_calc_table;
    pstExpFuncs->pfn_cmos_dgain_calc_table  = cmos_dgain_calc_table;

    return HI_SUCCESS;
}


/* AWB default parameter and function */
#define CALIBRATE_STATIC_WB_R_GAIN 518
#define CALIBRATE_STATIC_WB_GR_GAIN 256
#define CALIBRATE_STATIC_WB_GB_GAIN 256
#define CALIBRATE_STATIC_WB_B_GAIN 409

/* Calibration results for Auto WB Planck */
#define CALIBRATE_AWB_P1 30
#define CALIBRATE_AWB_P2 188
#define CALIBRATE_AWB_Q1 -38
#define CALIBRATE_AWB_A1 153679
#define CALIBRATE_AWB_B1 128
#define CALIBRATE_AWB_C1 -101709

/* Rgain and Bgain of the golden sample */
#define GOLDEN_RGAIN 0
#define GOLDEN_BGAIN 0
static HI_S32 cmos_get_awb_default(VI_PIPE ViPipe, AWB_SENSOR_DEFAULT_S *pstAwbSnsDft)
{
    ISP_SNS_STATE_S *pstSnsState = HI_NULL;

    CMOS_CHECK_POINTER(pstAwbSnsDft);
    OV12870_SENSOR_GET_CTX(ViPipe, pstSnsState);
    CMOS_CHECK_POINTER(pstSnsState);

    memset(pstAwbSnsDft, 0, sizeof(AWB_SENSOR_DEFAULT_S));

    pstAwbSnsDft->u16WbRefTemp = 4983;

    pstAwbSnsDft->au16GainOffset[0] = CALIBRATE_STATIC_WB_R_GAIN;
    pstAwbSnsDft->au16GainOffset[1] = CALIBRATE_STATIC_WB_GR_GAIN;
    pstAwbSnsDft->au16GainOffset[2] = CALIBRATE_STATIC_WB_GB_GAIN;
    pstAwbSnsDft->au16GainOffset[3] = CALIBRATE_STATIC_WB_B_GAIN;

    pstAwbSnsDft->as32WbPara[0] = CALIBRATE_AWB_P1;
    pstAwbSnsDft->as32WbPara[1] = CALIBRATE_AWB_P2;
    pstAwbSnsDft->as32WbPara[2] = CALIBRATE_AWB_Q1;
    pstAwbSnsDft->as32WbPara[3] = CALIBRATE_AWB_A1;
    pstAwbSnsDft->as32WbPara[4] = CALIBRATE_AWB_B1;
    pstAwbSnsDft->as32WbPara[5] = CALIBRATE_AWB_C1;
    pstAwbSnsDft->u16GoldenRgain = GOLDEN_RGAIN;
    pstAwbSnsDft->u16GoldenBgain = GOLDEN_BGAIN;


    switch (pstSnsState->enWDRMode) {
        default:
        case WDR_MODE_NONE:
            memcpy(&pstAwbSnsDft->stCcm, &g_stAwbCcm, sizeof(AWB_CCM_S));
            memcpy(&pstAwbSnsDft->stAgcTbl, &g_stAwbAgcTable, sizeof(AWB_AGC_TABLE_S));
            break;

    }
    pstAwbSnsDft->u16InitRgain = g_au16InitWBGain[ViPipe][0];
    pstAwbSnsDft->u16InitGgain = g_au16InitWBGain[ViPipe][1];
    pstAwbSnsDft->u16InitBgain = g_au16InitWBGain[ViPipe][2];
    pstAwbSnsDft->u16SampleRgain = g_au16SampleRgain[ViPipe];
    pstAwbSnsDft->u16SampleBgain = g_au16SampleBgain[ViPipe];

    return HI_SUCCESS;
}

static HI_S32 cmos_get_awb_spec_default(VI_PIPE ViPipe, AWB_SPEC_SENSOR_DEFAULT_S *pstAwbSpecSnsDft)
{
    CMOS_CHECK_POINTER(pstAwbSpecSnsDft);

    memset(pstAwbSpecSnsDft, 0, sizeof(AWB_SPEC_SENSOR_DEFAULT_S));

    memcpy(&pstAwbSpecSnsDft->stSpecAwbAttrs, &g_SpecAWBFactTbl, sizeof(ISP_SPECAWB_ATTR_S));
    memcpy(&pstAwbSpecSnsDft->stCaaControl, &g_SpecKCAWBCaaTblControl, sizeof(ISP_SPECAWB_CAA_CONTROl_S));

    return HI_SUCCESS;
}

static HI_S32 cmos_init_awb_exp_function(AWB_SENSOR_EXP_FUNC_S *pstExpFuncs)
{
    CMOS_CHECK_POINTER(pstExpFuncs);

    memset(pstExpFuncs, 0, sizeof(AWB_SENSOR_EXP_FUNC_S));

    pstExpFuncs->pfn_cmos_get_awb_default = cmos_get_awb_default;
    pstExpFuncs->pfn_cmos_get_awb_spec_default = cmos_get_awb_spec_default;

    return HI_SUCCESS;
}


static ISP_CMOS_DNG_COLORPARAM_S g_stDngColorParam = {
    {378, 256, 430},
    {439, 256, 439}
};

static HI_S32 cmos_get_isp_default(VI_PIPE ViPipe, ISP_CMOS_DEFAULT_S *pstDef)
{
    ISP_SNS_STATE_S *pstSnsState = HI_NULL;

    CMOS_CHECK_POINTER(pstDef);
    OV12870_SENSOR_GET_CTX(ViPipe, pstSnsState);
    CMOS_CHECK_POINTER(pstSnsState);

    memset(pstDef, 0, sizeof(ISP_CMOS_DEFAULT_S));
#ifdef CONFIG_HI_ISP_CA_SUPPORT
    pstDef->unKey.bit1Ca       = 1;
    pstDef->pstCa              = &g_stIspCA;
#endif
    pstDef->unKey.bit1Clut     = 1;
    pstDef->pstClut            = &g_stIspCLUT;

    pstDef->unKey.bit1Dpc      = 1;
    pstDef->pstDpc             = &g_stCmosDpc;

    pstDef->unKey.bit1Wdr      = 1;
    pstDef->pstWdr             = &g_stIspWDR;
#ifdef CONFIG_HI_ISP_HLC_SUPPORT
    pstDef->unKey.bit1Hlc      = 0;
    pstDef->pstHlc             = &g_stIspHlc;
#endif
    if (pstSnsState->u8ImgMode == OV12870_8M_30FPS_10BIT_LINEAR_MODE) {
        pstDef->unKey.bit1Lsc      = 1;
        pstDef->pstLsc             = &g_stCmosLsc_8M;
    } else if (pstSnsState->u8ImgMode == OV12870_12M_30FPS_10BIT_LINEAR_MODE) {
        pstDef->unKey.bit1Lsc      = 1;
        pstDef->pstLsc             = &g_stCmosLsc_12M;
    } else if (pstSnsState->u8ImgMode == OV12870_2M_120FPS_10BIT_LINEAR_MODE) {
        pstDef->unKey.bit1Lsc      = 1;
        pstDef->pstLsc             = &g_stCmosLsc_2M;
    }
#ifdef CONFIG_HI_ISP_EDGEMARK_SUPPORT
    pstDef->unKey.bit1EdgeMark = 0;
    pstDef->pstEdgeMark        = &g_stIspEdgeMark;
#endif
#ifdef CONFIG_HI_ISP_CR_SUPPORT
    pstDef->unKey.bit1Ge       = 1;
    pstDef->pstGe              = &g_stIspGe;
#endif
    pstDef->unKey.bit1Detail   = 1;
    pstDef->pstDetail          = &g_stIspDetail;

    switch (pstSnsState->enWDRMode) {
        default:
        case WDR_MODE_NONE:
            pstDef->unKey.bit1Demosaic       = 1;
            pstDef->pstDemosaic              = &g_stIspDemosaic;
            pstDef->unKey.bit1Sharpen        = 1;
            pstDef->pstSharpen               = &g_stIspYuvSharpen;
            pstDef->unKey.bit1Drc            = 1;
            pstDef->pstDrc                   = &g_stIspDRC;
            pstDef->unKey.bit1Gamma          = 1;
            pstDef->pstGamma                 = &g_stIspGamma;
            pstDef->unKey.bit1BayerNr        = 1;
            pstDef->pstBayerNr               = &g_stIspBayerNr;
            pstDef->unKey.bit1AntiFalseColor = 1;
            pstDef->pstAntiFalseColor        = &g_stIspAntiFalseColor;
            pstDef->unKey.bit1Ldci           = 1;
            pstDef->pstLdci                  = &g_stIspLdci;
            pstDef->unKey.bit1Dehaze         = 1;
            pstDef->pstDehaze                = &g_stIspDehaze;
            memcpy(&pstDef->stNoiseCalibration, &g_stIspNoiseCalibration, sizeof(ISP_CMOS_NOISE_CALIBRATION_S));
            break;
    }

    pstDef->stSensorMode.u32SensorID = OV12870_ID;
    pstDef->stSensorMode.u8SensorMode = pstSnsState->u8ImgMode;

    memcpy(&pstDef->stDngColorParam, &g_stDngColorParam, sizeof(ISP_CMOS_DNG_COLORPARAM_S));

    switch (pstSnsState->u8ImgMode) {
        default:
            pstDef->stSensorMode.stDngRawFormat.u8BitsPerSample = 10;
            pstDef->stSensorMode.stDngRawFormat.u32WhiteLevel = 1023;
            break;
    }

    pstDef->stSensorMode.stDngRawFormat.stDefaultScale.stDefaultScaleH.u32Denominator = 1;
    pstDef->stSensorMode.stDngRawFormat.stDefaultScale.stDefaultScaleH.u32Numerator = 1;
    pstDef->stSensorMode.stDngRawFormat.stDefaultScale.stDefaultScaleV.u32Denominator = 1;
    pstDef->stSensorMode.stDngRawFormat.stDefaultScale.stDefaultScaleV.u32Numerator = 1;
    pstDef->stSensorMode.stDngRawFormat.stCfaRepeatPatternDim.u16RepeatPatternDimRows = 2;
    pstDef->stSensorMode.stDngRawFormat.stCfaRepeatPatternDim.u16RepeatPatternDimCols = 2;
    pstDef->stSensorMode.stDngRawFormat.stBlcRepeatDim.u16BlcRepeatRows = 2;
    pstDef->stSensorMode.stDngRawFormat.stBlcRepeatDim.u16BlcRepeatCols = 2;
    pstDef->stSensorMode.stDngRawFormat.enCfaLayout = CFALAYOUT_TYPE_RECTANGULAR;
    pstDef->stSensorMode.stDngRawFormat.au8CfaPlaneColor[0] = 0;
    pstDef->stSensorMode.stDngRawFormat.au8CfaPlaneColor[1] = 1;
    pstDef->stSensorMode.stDngRawFormat.au8CfaPlaneColor[2] = 2;
    pstDef->stSensorMode.stDngRawFormat.au8CfaPattern[0] = 0;
    pstDef->stSensorMode.stDngRawFormat.au8CfaPattern[1] = 1;
    pstDef->stSensorMode.stDngRawFormat.au8CfaPattern[2] = 1;
    pstDef->stSensorMode.stDngRawFormat.au8CfaPattern[3] = 2;
    pstDef->stSensorMode.bValidDngRawFormat = HI_TRUE;

    return HI_SUCCESS;
}


static HI_S32 cmos_get_isp_black_level(VI_PIPE ViPipe, ISP_CMOS_BLACK_LEVEL_S *pstBlackLevel)
{
    CMOS_CHECK_POINTER(pstBlackLevel);

    /* It need to update black level when iso change */
    pstBlackLevel->bUpdate = HI_TRUE;

    if (g_au32Again[ViPipe] < 4096) {
        pstBlackLevel->au16BlackLevel[0] = 255;
        pstBlackLevel->au16BlackLevel[1] = 255;
        pstBlackLevel->au16BlackLevel[2] = 255;
        pstBlackLevel->au16BlackLevel[3] = 255;
    } else if (g_au32Again[ViPipe] < 8192) {
        pstBlackLevel->au16BlackLevel[0] = 252;
        pstBlackLevel->au16BlackLevel[1] = 252;
        pstBlackLevel->au16BlackLevel[2] = 252;
        pstBlackLevel->au16BlackLevel[3] = 252;
    } else if (g_au32Again[ViPipe] < 15872) {
        pstBlackLevel->au16BlackLevel[0] = 250;
        pstBlackLevel->au16BlackLevel[1] = 250;
        pstBlackLevel->au16BlackLevel[2] = 250;
        pstBlackLevel->au16BlackLevel[3] = 250;
    } else {
        pstBlackLevel->au16BlackLevel[0] = 255;
        pstBlackLevel->au16BlackLevel[1] = 255;
        pstBlackLevel->au16BlackLevel[2] = 255;
        pstBlackLevel->au16BlackLevel[3] = 255;
    }

    return HI_SUCCESS;

}
static HI_VOID cmos_set_pixel_detect(VI_PIPE ViPipe, HI_BOOL bEnable)
{
    ISP_SNS_STATE_S *pstSnsState = HI_NULL;
    HI_U32 u32Again;
    HI_U32 u32CoarseTime;
    HI_U32 u32Vmax;

    OV12870_SENSOR_GET_CTX(ViPipe, pstSnsState);
    CMOS_CHECK_POINTER_VOID(pstSnsState);

    if (bEnable) { /* setup for ISP pixel calibration mode */
        u32Again = (1024 >> 3);
        u32Vmax = g_astOv12870ModeTbl[pstSnsState->u8ImgMode].u32VMax * g_astOv12870ModeTbl[pstSnsState->u8ImgMode].f32MaxFps / 5;
        u32Vmax = MIN(u32Vmax, OV12870_FULL_LINES_MAX);
        u32CoarseTime = u32Vmax - g_astOv12870ModeTbl[pstSnsState->u8ImgMode].u32ExpLineLimit;
        ov12870_write_register(ViPipe, OV12870_COARSE_INTEG_TIME_L, LOW_8BITS(u32CoarseTime));
        ov12870_write_register(ViPipe, OV12870_COARSE_INTEG_TIME_M, HIGH_8BITS(u32CoarseTime));
        ov12870_write_register(ViPipe, OV12870_COARSE_INTEG_TIME_H, HIGHER_8BITS(u32CoarseTime));
        ov12870_write_register(ViPipe, OV12870_ANA_GAIN_GLOBAL_L_0, (u32Again & 0xFF));
        ov12870_write_register(ViPipe, OV12870_ANA_GAIN_GLOBAL_H_0, ((u32Again >> 8) & 0xf));
        ov12870_write_register(ViPipe, OV12870_ANA_GAIN_GLOBAL_L_1, (u32Again & 0xFF));
        ov12870_write_register(ViPipe, OV12870_ANA_GAIN_GLOBAL_H_1, ((u32Again >> 8) & 0xf));
        ov12870_write_register(ViPipe, OV12870_VMAX_L, LOW_8BITS(u32Vmax));
        ov12870_write_register(ViPipe, OV12870_VMAX_H, HIGH_8BITS(u32Vmax));
    } else { /* setup for ISP 'normal mode' */
        u32Vmax = pstSnsState->u32FLStd;
        ov12870_write_register(ViPipe, OV12870_VMAX_L, LOW_8BITS(u32Vmax));
        ov12870_write_register(ViPipe, OV12870_VMAX_L, HIGH_8BITS(u32Vmax));
        pstSnsState->bSyncInit  = HI_FALSE;
    }

    return;
}

static HI_S32 cmos_set_wdr_mode(VI_PIPE ViPipe, HI_U8 u8Mode)
{
    ISP_SNS_STATE_S *pstSnsState = HI_NULL;

    OV12870_SENSOR_GET_CTX(ViPipe, pstSnsState);
    CMOS_CHECK_POINTER(pstSnsState);

    pstSnsState->bSyncInit = HI_FALSE;

    switch (u8Mode & 0x3F) {
        case WDR_MODE_NONE:
            pstSnsState->enWDRMode = WDR_MODE_NONE;
            printf("linear mode\n");
            break;

        default:
            ISP_ERR_TRACE("NOT support this mode!\n");
            return HI_FAILURE;
    }

    memset(pstSnsState->au32WDRIntTime, 0, sizeof(pstSnsState->au32WDRIntTime));

    return HI_SUCCESS;
}

static HI_S32 cmos_get_sns_regs_info(VI_PIPE ViPipe, ISP_SNS_REGS_INFO_S *pstSnsRegsInfo)
{
    HI_S32 i;
    ISP_SNS_STATE_S *pstSnsState = HI_NULL;

    CMOS_CHECK_POINTER(pstSnsRegsInfo);
    OV12870_SENSOR_GET_CTX(ViPipe, pstSnsState);
    CMOS_CHECK_POINTER(pstSnsState);

    if ((pstSnsState->bSyncInit == HI_FALSE) || (pstSnsRegsInfo->bConfig == HI_FALSE)) {
        pstSnsState->astRegsInfo[0].enSnsType = ISP_SNS_I2C_TYPE;
        pstSnsState->astRegsInfo[0].unComBus.s8I2cDev = g_aunOv12870BusInfo[ViPipe].s8I2cDev;
        pstSnsState->astRegsInfo[0].u8Cfg2ValidDelayMax = 2;
        pstSnsState->astRegsInfo[0].u32RegNum = 15;

        for (i = 0; i < pstSnsState->astRegsInfo[0].u32RegNum; i++) {
            pstSnsState->astRegsInfo[0].astI2cData[i].bUpdate = HI_TRUE;
            pstSnsState->astRegsInfo[0].astI2cData[i].u8DevAddr = ov12870_i2c_addr;
            pstSnsState->astRegsInfo[0].astI2cData[i].u32AddrByteNum = ov12870_addr_byte;
            pstSnsState->astRegsInfo[0].astI2cData[i].u32DataByteNum = ov12870_data_byte;
        }

        //shutter related
        pstSnsState->astRegsInfo[0].astI2cData[0].u8DelayFrmNum = 0;
        pstSnsState->astRegsInfo[0].astI2cData[0].u32RegAddr = OV12870_COARSE_INTEG_TIME_L;
        pstSnsState->astRegsInfo[0].astI2cData[1].u8DelayFrmNum = 0;
        pstSnsState->astRegsInfo[0].astI2cData[1].u32RegAddr = OV12870_COARSE_INTEG_TIME_M;
        pstSnsState->astRegsInfo[0].astI2cData[2].u8DelayFrmNum = 0;
        pstSnsState->astRegsInfo[0].astI2cData[2].u32RegAddr = OV12870_COARSE_INTEG_TIME_H;

        // gain related
        pstSnsState->astRegsInfo[0].astI2cData[3].u32RegAddr = OV12870_ANA_GAIN_GLOBAL_L_0;
        pstSnsState->astRegsInfo[0].astI2cData[3].u8DelayFrmNum = 0;
        pstSnsState->astRegsInfo[0].astI2cData[4].u32RegAddr = OV12870_ANA_GAIN_GLOBAL_H_0;
        pstSnsState->astRegsInfo[0].astI2cData[4].u8DelayFrmNum = 0;

        // Dgain_gr
        pstSnsState->astRegsInfo[0].astI2cData[5].u8DelayFrmNum = 0;
        pstSnsState->astRegsInfo[0].astI2cData[5].u32RegAddr = OV12870_ANA_GAIN_GLOBAL_L_1;
        pstSnsState->astRegsInfo[0].astI2cData[6].u8DelayFrmNum = 0;
        pstSnsState->astRegsInfo[0].astI2cData[6].u32RegAddr = OV12870_ANA_GAIN_GLOBAL_H_1;

        //Vmax
        pstSnsState->astRegsInfo[0].astI2cData[7].u8DelayFrmNum = 0;
        pstSnsState->astRegsInfo[0].astI2cData[7].u32RegAddr = OV12870_VMAX_L;
        pstSnsState->astRegsInfo[0].astI2cData[8].u8DelayFrmNum = 0;
        pstSnsState->astRegsInfo[0].astI2cData[8].u32RegAddr = OV12870_VMAX_H;

        //long exposure
        pstSnsState->astRegsInfo[0].astI2cData[9].u8DelayFrmNum = 1;
        pstSnsState->astRegsInfo[0].astI2cData[9].u32RegAddr = OV12870_LONG_EXPOSURE_0;
        pstSnsState->astRegsInfo[0].astI2cData[10].u8DelayFrmNum = 1;
        pstSnsState->astRegsInfo[0].astI2cData[10].u32RegAddr = OV12870_LONG_EXPOSURE_1;
        pstSnsState->astRegsInfo[0].astI2cData[11].u8DelayFrmNum = 1;
        pstSnsState->astRegsInfo[0].astI2cData[11].u32RegAddr = OV12870_LONG_EXPOSURE_2;
        pstSnsState->astRegsInfo[0].astI2cData[12].u8DelayFrmNum = 1;
        pstSnsState->astRegsInfo[0].astI2cData[12].u32RegAddr = OV12870_LONG_EXPOSURE_3;
        pstSnsState->astRegsInfo[0].astI2cData[13].u8DelayFrmNum = 1;
        pstSnsState->astRegsInfo[0].astI2cData[13].u32RegAddr = OV12870_LONG_EXPOSURE_4;
        pstSnsState->astRegsInfo[0].astI2cData[14].u8DelayFrmNum = 1;
        pstSnsState->astRegsInfo[0].astI2cData[14].u32RegAddr = OV12870_LONG_EXPOSURE_5;

        pstSnsState->bSyncInit = HI_TRUE;
    } else {
        for (i = 0; i < pstSnsState->astRegsInfo[0].u32RegNum; i++) {
            if (pstSnsState->astRegsInfo[0].astI2cData[i].u32Data == pstSnsState->astRegsInfo[1].astI2cData[i].u32Data) {
                pstSnsState->astRegsInfo[0].astI2cData[i].bUpdate = HI_FALSE;
            } else {

                pstSnsState->astRegsInfo[0].astI2cData[i].bUpdate = HI_TRUE;
            }
        }
    }

    memcpy(pstSnsRegsInfo, &pstSnsState->astRegsInfo[0], sizeof(ISP_SNS_REGS_INFO_S));
    memcpy(&pstSnsState->astRegsInfo[1], &pstSnsState->astRegsInfo[0], sizeof(ISP_SNS_REGS_INFO_S));

    pstSnsState->au32FL[1] = pstSnsState->au32FL[0];

    return HI_SUCCESS;
}

static HI_S32 cmos_set_image_mode(VI_PIPE ViPipe, ISP_CMOS_SENSOR_IMAGE_MODE_S *pstSensorImageMode)
{
    HI_U8 u8SensorImageMode;
    ISP_SNS_STATE_S *pstSnsState = HI_NULL;
    HI_U8 u8SnsMode;
    HI_U32 u32W, u32H;

    CMOS_CHECK_POINTER(pstSensorImageMode);
    OV12870_SENSOR_GET_CTX(ViPipe, pstSnsState);
    CMOS_CHECK_POINTER(pstSnsState);

    pstSnsState->bSyncInit = HI_FALSE;

    u32H = pstSensorImageMode->u16Height;
    u32W = pstSensorImageMode->u16Width;
    u8SnsMode = pstSensorImageMode->u8SnsMode;

    if (OV12870_RES_IS_1M(u32W, u32H)) {
        if (u8SnsMode == 0) {
            u8SensorImageMode = OV12870_1M_240FPS_10BIT_LINEAR_MODE;
        } else {
            OV12870_ERR_MODE_PRINT(pstSensorImageMode);
            return HI_FAILURE;
        }
    } else if (OV12870_RES_IS_2M(u32W, u32H)) {
        if (u8SnsMode == 0) {
            u8SensorImageMode = OV12870_2M_120FPS_10BIT_LINEAR_MODE;
        } else {
            OV12870_ERR_MODE_PRINT(pstSensorImageMode);
            return HI_FAILURE;
        }
    } else if (OV12870_RES_IS_8M(u32W, u32H)) {

        if (u8SnsMode == 0) {
            u8SensorImageMode = OV12870_8M_30FPS_10BIT_LINEAR_MODE;
        } else {
            OV12870_ERR_MODE_PRINT(pstSensorImageMode);
            return HI_FAILURE;
        }
    } else if (OV12870_RES_IS_12M(u32W, u32H)) {
        if (u8SnsMode == 0) {
            u8SensorImageMode = OV12870_12M_30FPS_10BIT_LINEAR_MODE;
        } else {
            OV12870_ERR_MODE_PRINT(pstSensorImageMode);
            return HI_FAILURE;
        }
    } else {
        OV12870_ERR_MODE_PRINT(pstSensorImageMode);
        return HI_FAILURE;
    }

    /* Switch SensorImageMode */
    if ((pstSnsState->bInit == HI_TRUE) && (u8SensorImageMode == pstSnsState->u8ImgMode)) {
        /* Don't need to switch SensorImageMode */
        return ISP_DO_NOT_NEED_SWITCH_IMAGEMODE;
    }

    pstSnsState->u8ImgMode = u8SensorImageMode;
    pstSnsState->u32FLStd  = g_astOv12870ModeTbl[pstSnsState->u8ImgMode].u32VMax;
    pstSnsState->au32FL[0] = pstSnsState->u32FLStd;
    pstSnsState->au32FL[1] = pstSnsState->au32FL[0];

    return HI_SUCCESS;
}

static HI_VOID sensor_global_init(VI_PIPE ViPipe)
{
    ISP_SNS_STATE_S *pstSnsState = HI_NULL;

    OV12870_SENSOR_GET_CTX(ViPipe, pstSnsState);
    CMOS_CHECK_POINTER_VOID(pstSnsState);

    pstSnsState->bInit = HI_FALSE;
    pstSnsState->bSyncInit = HI_FALSE;
    pstSnsState->u8ImgMode = OV12870_2M_120FPS_10BIT_LINEAR_MODE;
    pstSnsState->enWDRMode = WDR_MODE_NONE;
    pstSnsState->u32FLStd = g_astOv12870ModeTbl[pstSnsState->u8ImgMode].u32VMax;
    pstSnsState->au32FL[0] = g_astOv12870ModeTbl[pstSnsState->u8ImgMode].u32VMax;
    pstSnsState->au32FL[1] = g_astOv12870ModeTbl[pstSnsState->u8ImgMode].u32VMax;

    memset(&pstSnsState->astRegsInfo[0], 0, sizeof(ISP_SNS_REGS_INFO_S));
    memset(&pstSnsState->astRegsInfo[1], 0, sizeof(ISP_SNS_REGS_INFO_S));
}

static HI_S32 cmos_init_sensor_exp_function(ISP_SENSOR_EXP_FUNC_S *pstSensorExpFunc)
{
    CMOS_CHECK_POINTER(pstSensorExpFunc);

    memset(pstSensorExpFunc, 0, sizeof(ISP_SENSOR_EXP_FUNC_S));

    pstSensorExpFunc->pfn_cmos_sensor_init = ov12870_init;
    pstSensorExpFunc->pfn_cmos_sensor_exit = ov12870_exit;
    pstSensorExpFunc->pfn_cmos_sensor_global_init = sensor_global_init;
    pstSensorExpFunc->pfn_cmos_set_image_mode = cmos_set_image_mode;
    pstSensorExpFunc->pfn_cmos_set_wdr_mode = cmos_set_wdr_mode;
    pstSensorExpFunc->pfn_cmos_get_isp_default = cmos_get_isp_default;
    pstSensorExpFunc->pfn_cmos_get_isp_black_level = cmos_get_isp_black_level;
    pstSensorExpFunc->pfn_cmos_set_pixel_detect = cmos_set_pixel_detect;
    pstSensorExpFunc->pfn_cmos_get_sns_reg_info = cmos_get_sns_regs_info;

    return HI_SUCCESS;
}

/****************************************************************************
 * callback structure                                                       *
 ****************************************************************************/

static HI_S32 ov12870_set_bus_info(VI_PIPE ViPipe, ISP_SNS_COMMBUS_U unSNSBusInfo)
{
    g_aunOv12870BusInfo[ViPipe].s8I2cDev = unSNSBusInfo.s8I2cDev;

    return HI_SUCCESS;
}

static HI_S32 sensor_ctx_init(VI_PIPE ViPipe)
{
    ISP_SNS_STATE_S *pastSnsStateCtx = HI_NULL;

    OV12870_SENSOR_GET_CTX(ViPipe, pastSnsStateCtx);

    if (pastSnsStateCtx == HI_NULL) {
        pastSnsStateCtx = (ISP_SNS_STATE_S *)malloc(sizeof(ISP_SNS_STATE_S));
        if (pastSnsStateCtx == HI_NULL) {
            ISP_ERR_TRACE("Isp[%d] SnsCtx malloc memory failed!\n", ViPipe);
            return HI_ERR_ISP_NOMEM;
        }
    }

    memset(pastSnsStateCtx, 0, sizeof(ISP_SNS_STATE_S));

    OV12870_SENSOR_SET_CTX(ViPipe, pastSnsStateCtx);

    return HI_SUCCESS;
}

static HI_VOID sensor_ctx_exit(VI_PIPE ViPipe)
{
    ISP_SNS_STATE_S *pastSnsStateCtx = HI_NULL;

    OV12870_SENSOR_GET_CTX(ViPipe, pastSnsStateCtx);
    SENSOR_FREE(pastSnsStateCtx);
    OV12870_SENSOR_RESET_CTX(ViPipe);
}

static HI_S32 sensor_register_callback(VI_PIPE ViPipe, ALG_LIB_S *pstAeLib, ALG_LIB_S *pstAwbLib)
{
    HI_S32 s32Ret;
    ISP_SENSOR_REGISTER_S stIspRegister;
    AE_SENSOR_REGISTER_S  stAeRegister;
    AWB_SENSOR_REGISTER_S stAwbRegister;
    ISP_SNS_ATTR_INFO_S   stSnsAttrInfo;

    CMOS_CHECK_POINTER(pstAeLib);
    CMOS_CHECK_POINTER(pstAwbLib);

    s32Ret = sensor_ctx_init(ViPipe);

    if (s32Ret != HI_SUCCESS) {
        return HI_FAILURE;
    }

    stSnsAttrInfo.eSensorId = OV12870_ID;

    s32Ret  = cmos_init_sensor_exp_function(&stIspRegister.stSnsExp);
    s32Ret |= HI_MPI_ISP_SensorRegCallBack(ViPipe, &stSnsAttrInfo, &stIspRegister);

    if (s32Ret != HI_SUCCESS) {
        ISP_ERR_TRACE("sensor register callback function failed!\n");
        return s32Ret;
    }

    s32Ret  = cmos_init_ae_exp_function(&stAeRegister.stSnsExp);
    s32Ret |= HI_MPI_AE_SensorRegCallBack(ViPipe, pstAeLib, &stSnsAttrInfo, &stAeRegister);

    if (s32Ret != HI_SUCCESS) {
        ISP_ERR_TRACE("sensor register callback function to ae lib failed!\n");
        return s32Ret;
    }

    s32Ret  = cmos_init_awb_exp_function(&stAwbRegister.stSnsExp);
    s32Ret |= HI_MPI_AWB_SensorRegCallBack(ViPipe, pstAwbLib, &stSnsAttrInfo, &stAwbRegister);

    if (s32Ret != HI_SUCCESS) {
        ISP_ERR_TRACE("sensor register callback function to awb lib failed!\n");
        return s32Ret;
    }

    return HI_SUCCESS;
}

static HI_S32 sensor_unregister_callback(VI_PIPE ViPipe, ALG_LIB_S *pstAeLib, ALG_LIB_S *pstAwbLib)
{
    HI_S32 s32Ret;

    CMOS_CHECK_POINTER(pstAeLib);
    CMOS_CHECK_POINTER(pstAwbLib);

    s32Ret = HI_MPI_ISP_SensorUnRegCallBack(ViPipe, OV12870_ID);

    if (s32Ret != HI_SUCCESS) {
        ISP_ERR_TRACE("sensor unregister callback function failed!\n");
        return s32Ret;
    }

    s32Ret = HI_MPI_AE_SensorUnRegCallBack(ViPipe, pstAeLib, OV12870_ID);

    if (s32Ret != HI_SUCCESS) {
        ISP_ERR_TRACE("sensor unregister callback function to ae lib failed!\n");
        return s32Ret;
    }

    s32Ret = HI_MPI_AWB_SensorUnRegCallBack(ViPipe, pstAwbLib, OV12870_ID);

    if (s32Ret != HI_SUCCESS) {
        ISP_ERR_TRACE("sensor unregister callback function to awb lib failed!\n");
        return s32Ret;
    }

    sensor_ctx_exit(ViPipe);

    return HI_SUCCESS;
}

static HI_S32 sensor_set_init(VI_PIPE ViPipe, ISP_INIT_ATTR_S *pstInitAttr)
{
    CMOS_CHECK_POINTER(pstInitAttr);

    g_au32InitExposure[ViPipe] = pstInitAttr->u32Exposure;
    g_au32LinesPer500ms[ViPipe] = pstInitAttr->u32LinesPer500ms;
    g_au16InitWBGain[ViPipe][0] = pstInitAttr->u16WBRgain;
    g_au16InitWBGain[ViPipe][1] = pstInitAttr->u16WBGgain;
    g_au16InitWBGain[ViPipe][2] = pstInitAttr->u16WBBgain;
    g_au16SampleRgain[ViPipe] = pstInitAttr->u16SampleRgain;
    g_au16SampleBgain[ViPipe] = pstInitAttr->u16SampleBgain;

    return HI_SUCCESS;
}

ISP_SNS_OBJ_S stSnsOv12870Obj = {
    .pfnRegisterCallback    = sensor_register_callback,
    .pfnUnRegisterCallback  = sensor_unregister_callback,
    .pfnStandby             = ov12870_standby,
    .pfnRestart             = ov12870_restart,
    .pfnMirrorFlip          = ov12870_mirror_flip,
    .pfnWriteReg            = ov12870_write_register,
    .pfnReadReg             = ov12870_read_register,
    .pfnSetBusInfo          = ov12870_set_bus_info,
    .pfnSetInit             = sensor_set_init
};


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */

#endif /* __OV12870_CMOS_H_ */
