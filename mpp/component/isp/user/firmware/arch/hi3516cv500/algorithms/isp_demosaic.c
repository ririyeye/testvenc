/*
* Copyright (C) Hisilicon Technologies Co., Ltd. 2012-2019. All rights reserved.
* Description:
* Author: Hisilicon multimedia software group
* Create: 2011/06/28
*/


#include "isp_alg.h"
#include "isp_sensor.h"
#include "isp_config.h"
#include "isp_ext_config.h"
#include "isp_proc.h"
#include "isp_math_utils.h"
#include <math.h>

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

#define HI_DEMOSAIC_BITDEPTH    (12)

static const  HI_S32 g_as32DemosaicFilterCoef[7][3] = {{0, 0, 32}, { -1, 4, 26}, { -1, 5, 24}, { -1, 6, 22}, { -1, 7, 20}, { -1, 8, 18}, {0, 8, 16}};

static const  HI_U8  g_au8NonDirStr[ISP_AUTO_ISO_STRENGTH_NUM]             = {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64};
static const  HI_U8  g_au8NonDirMFDetailEhcStr[ISP_AUTO_ISO_STRENGTH_NUM]  = {32, 24, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};
static const  HI_U8  g_au8NonDirHFDetailEhcStr[ISP_AUTO_ISO_STRENGTH_NUM]  = {0, 0, 3, 5, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7};
static const  HI_U8  g_au8DetailSmoothRange[ISP_AUTO_ISO_STRENGTH_NUM]     = {2, 2, 2, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 7};
static const  HI_U8  g_au8ColorNoiseThdF[ISP_AUTO_ISO_STRENGTH_NUM]        = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static const  HI_U8  g_au8ColorNoiseStrF[ISP_AUTO_ISO_STRENGTH_NUM]        = {8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8};
static const  HI_U8  g_au8ColorNoiseThdY[ISP_AUTO_ISO_STRENGTH_NUM]        = {30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30};
static const  HI_U8  g_au8ColorNoiseStrY[ISP_AUTO_ISO_STRENGTH_NUM]        = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

typedef struct hiISP_DEMOSAIC_S {
    // Processing Depth
    HI_BOOL  bEnable;           // u1.0
    HI_BOOL  bInit;
    HI_BOOL  bDemAttrUpdate;

    HI_U8    u8BitDepthPrc;    // u5.0
    HI_U8    u8WdrMode;
    HI_U32   au32sigma[HI_ISP_DEMOSAIC_LUT_LENGTH];

    HI_U8    au8LutAwbGFGainLow [ISP_AUTO_ISO_STRENGTH_NUM]; // u3.4,
    HI_U8    au8LutAwbGFGainHig [ISP_AUTO_ISO_STRENGTH_NUM]; // u3.4,
    HI_U8    au8LutAwbGFGainMax [ISP_AUTO_ISO_STRENGTH_NUM]; // u4.0,
    HI_U8    au8LutCcHFMaxRatio [ISP_AUTO_ISO_STRENGTH_NUM]; // u5.0, 0~16
    HI_U8    au8LutCcHFMinRatio [ISP_AUTO_ISO_STRENGTH_NUM]; // u5.0, 0~16
    HI_S8    as8LutFcrGFGain    [ISP_AUTO_ISO_STRENGTH_NUM]; // s3.2, fcr control
    HI_U16   au16LutDeSatLow    [ISP_AUTO_ISO_STRENGTH_NUM]; // u9.0, 0~256
    HI_U16   au16LutDeSatProtTh [ISP_AUTO_ISO_STRENGTH_NUM]; // u10.0, 0~1023
    HI_U16   au16LutFcrDetLow   [ISP_AUTO_ISO_STRENGTH_NUM]; // u12.0,  fcr det thresh
    HI_U16   au16HfIntpBlurThLow[ISP_AUTO_ISO_STRENGTH_NUM]; // u10.0
    HI_U16   au16HfIntpBlurThHig[ISP_AUTO_ISO_STRENGTH_NUM]; // u10.0,

    ISP_OP_TYPE_E enOpType;
    ISP_DEMOSAIC_AUTO_ATTR_S   stAuto;
    ISP_DEMOSAIC_MANUAL_ATTR_S stManual;
} ISP_DEMOSAIC_S;

ISP_DEMOSAIC_S *g_pastDemosaicCtx[ISP_MAX_PIPE_NUM] = {HI_NULL};

#define DEMOSAIC_GET_CTX(dev, pstCtx)   (pstCtx = g_pastDemosaicCtx[dev])
#define DEMOSAIC_SET_CTX(dev, pstCtx)   (g_pastDemosaicCtx[dev] = pstCtx)
#define DEMOSAIC_RESET_CTX(dev)         (g_pastDemosaicCtx[dev] = HI_NULL)

HI_S32 DemosaicCtxInit(VI_PIPE ViPipe)
{
    ISP_DEMOSAIC_S *pastDemosaicCtx = HI_NULL;

    DEMOSAIC_GET_CTX(ViPipe, pastDemosaicCtx);

    if (pastDemosaicCtx == HI_NULL) {
        pastDemosaicCtx = (ISP_DEMOSAIC_S *)ISP_MALLOC(sizeof(ISP_DEMOSAIC_S));
        if (pastDemosaicCtx == HI_NULL) {
            ISP_ERR_TRACE("Isp[%d] DemosaicCtx malloc memory failed!\n", ViPipe);
            return HI_ERR_ISP_NOMEM;
        }
    }

    memset(pastDemosaicCtx, 0, sizeof(ISP_DEMOSAIC_S));

    DEMOSAIC_SET_CTX(ViPipe, pastDemosaicCtx);

    return HI_SUCCESS;
}

HI_VOID DemosaicCtxExit(VI_PIPE ViPipe)
{
    ISP_DEMOSAIC_S *pastDemosaicCtx = HI_NULL;

    DEMOSAIC_GET_CTX(ViPipe, pastDemosaicCtx);
    ISP_FREE(pastDemosaicCtx);
    DEMOSAIC_RESET_CTX(ViPipe);
}

static HI_VOID  DemosaicInitFwLinear(VI_PIPE ViPipe)
{
    ISP_DEMOSAIC_S *pstDemosaic = HI_NULL;

    HI_U8   au8LutCcHFMaxRatio[ISP_AUTO_ISO_STRENGTH_NUM]  = {3, 4, 5, 6, 6, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8};
    HI_U8   au8LutCcHFMinRatio[ISP_AUTO_ISO_STRENGTH_NUM]  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    HI_S8   as8LutFcrGFGain[ISP_AUTO_ISO_STRENGTH_NUM]     = {12, 12, 12, 10, 8, 6, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    HI_U8   au8LutAwbGFGainLow[ISP_AUTO_ISO_STRENGTH_NUM]  = {32, 32, 30, 26, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20 };
    HI_U8   au8LutAwbGFGainHig[ISP_AUTO_ISO_STRENGTH_NUM]  = {64, 64, 60, 56, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48, 48 };
    HI_U8   au8LutAwbGFGainMax[ISP_AUTO_ISO_STRENGTH_NUM]  = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    HI_U16  au16LutFcrDetLow[ISP_AUTO_ISO_STRENGTH_NUM]    = {150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150};
    HI_U16  au16LutDeSatLow[ISP_AUTO_ISO_STRENGTH_NUM]     = {166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166};
    HI_U16  au16LutDeSatProtTh[ISP_AUTO_ISO_STRENGTH_NUM]  = {128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128};
    HI_U16  au16HfIntpBlurThLow[ISP_AUTO_ISO_STRENGTH_NUM] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
    HI_U16  au16HfIntpBlurThHig[ISP_AUTO_ISO_STRENGTH_NUM] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};

    DEMOSAIC_GET_CTX(ViPipe, pstDemosaic);

    memcpy(pstDemosaic->au8LutCcHFMaxRatio,  au8LutCcHFMaxRatio,  sizeof(HI_U8) * ISP_AUTO_ISO_STRENGTH_NUM);
    memcpy(pstDemosaic->au8LutCcHFMinRatio,  au8LutCcHFMinRatio,  sizeof(HI_U8) * ISP_AUTO_ISO_STRENGTH_NUM);
    memcpy(pstDemosaic->as8LutFcrGFGain,     as8LutFcrGFGain,     sizeof(HI_S8) * ISP_AUTO_ISO_STRENGTH_NUM);
    memcpy(pstDemosaic->au8LutAwbGFGainLow,  au8LutAwbGFGainLow,  sizeof(HI_U8) * ISP_AUTO_ISO_STRENGTH_NUM);
    memcpy(pstDemosaic->au8LutAwbGFGainHig,  au8LutAwbGFGainHig,  sizeof(HI_U8) * ISP_AUTO_ISO_STRENGTH_NUM);
    memcpy(pstDemosaic->au8LutAwbGFGainMax,  au8LutAwbGFGainMax,  sizeof(HI_U8) * ISP_AUTO_ISO_STRENGTH_NUM);
    memcpy(pstDemosaic->au16LutFcrDetLow,    au16LutFcrDetLow,    sizeof(HI_U16) * ISP_AUTO_ISO_STRENGTH_NUM);
    memcpy(pstDemosaic->au16LutDeSatLow,     au16LutDeSatLow,     sizeof(HI_U16) * ISP_AUTO_ISO_STRENGTH_NUM);
    memcpy(pstDemosaic->au16LutDeSatProtTh,  au16LutDeSatProtTh,  sizeof(HI_U16) * ISP_AUTO_ISO_STRENGTH_NUM);
    memcpy(pstDemosaic->au16HfIntpBlurThLow, au16HfIntpBlurThLow, sizeof(HI_U16) * ISP_AUTO_ISO_STRENGTH_NUM);
    memcpy(pstDemosaic->au16HfIntpBlurThHig, au16HfIntpBlurThHig, sizeof(HI_U16) * ISP_AUTO_ISO_STRENGTH_NUM);
}

static HI_VOID  DemosaicInitFwWdr(VI_PIPE ViPipe)
{
    ISP_DEMOSAIC_S *pstDemosaic = HI_NULL;

    HI_U8   au8LutCcHFMaxRatio[ISP_AUTO_ISO_STRENGTH_NUM]  = {8, 8, 8, 8, 10, 10, 10, 12, 12, 14, 14, 16, 16, 16, 16, 16};
    HI_U8   au8LutCcHFMinRatio[ISP_AUTO_ISO_STRENGTH_NUM]  = {0, 0, 0, 0, 0, 0, 2, 2, 4, 4, 4, 8, 10, 10, 12, 14};
    HI_S8   as8LutFcrGFGain[ISP_AUTO_ISO_STRENGTH_NUM]     = { -1, -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    HI_U8   au8LutAwbGFGainLow[ISP_AUTO_ISO_STRENGTH_NUM]  = {32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32 };
    HI_U8   au8LutAwbGFGainHig[ISP_AUTO_ISO_STRENGTH_NUM]  = {64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64, 64 };
    HI_U8   au8LutAwbGFGainMax[ISP_AUTO_ISO_STRENGTH_NUM]  = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
    HI_U16  au16LutFcrDetLow[ISP_AUTO_ISO_STRENGTH_NUM]    = {150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150};
    HI_U16  au16LutDeSatLow[ISP_AUTO_ISO_STRENGTH_NUM]     = {166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166, 166};
    HI_U16  au16LutDeSatProtTh[ISP_AUTO_ISO_STRENGTH_NUM]  = {128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128};
    HI_U16  au16HfIntpBlurThLow[ISP_AUTO_ISO_STRENGTH_NUM] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};
    HI_U16  au16HfIntpBlurThHig[ISP_AUTO_ISO_STRENGTH_NUM] = {1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023};

    DEMOSAIC_GET_CTX(ViPipe, pstDemosaic);

    memcpy(pstDemosaic->au8LutCcHFMaxRatio,  au8LutCcHFMaxRatio,  sizeof(HI_U8) * ISP_AUTO_ISO_STRENGTH_NUM);
    memcpy(pstDemosaic->au8LutCcHFMinRatio,  au8LutCcHFMinRatio,  sizeof(HI_U8) * ISP_AUTO_ISO_STRENGTH_NUM);
    memcpy(pstDemosaic->as8LutFcrGFGain,     as8LutFcrGFGain,     sizeof(HI_S8) * ISP_AUTO_ISO_STRENGTH_NUM);
    memcpy(pstDemosaic->au8LutAwbGFGainLow,  au8LutAwbGFGainLow,  sizeof(HI_U8) * ISP_AUTO_ISO_STRENGTH_NUM);
    memcpy(pstDemosaic->au8LutAwbGFGainHig,  au8LutAwbGFGainHig,  sizeof(HI_U8) * ISP_AUTO_ISO_STRENGTH_NUM);
    memcpy(pstDemosaic->au8LutAwbGFGainMax,  au8LutAwbGFGainMax,  sizeof(HI_U8) * ISP_AUTO_ISO_STRENGTH_NUM);
    memcpy(pstDemosaic->au16LutFcrDetLow,    au16LutFcrDetLow,    sizeof(HI_U16) * ISP_AUTO_ISO_STRENGTH_NUM);
    memcpy(pstDemosaic->au16LutDeSatLow,     au16LutDeSatLow,     sizeof(HI_U16) * ISP_AUTO_ISO_STRENGTH_NUM);
    memcpy(pstDemosaic->au16LutDeSatProtTh,  au16LutDeSatProtTh,  sizeof(HI_U16) * ISP_AUTO_ISO_STRENGTH_NUM);
    memcpy(pstDemosaic->au16HfIntpBlurThLow, au16HfIntpBlurThLow, sizeof(HI_U16) * ISP_AUTO_ISO_STRENGTH_NUM);
    memcpy(pstDemosaic->au16HfIntpBlurThHig, au16HfIntpBlurThHig, sizeof(HI_U16) * ISP_AUTO_ISO_STRENGTH_NUM);
}

static HI_VOID DemosaicStaticRegsInitialize(VI_PIPE ViPipe, ISP_DEMOSAIC_STATIC_CFG_S *pstDemosaicStaticRegCfg, HI_U32 i)
{
    pstDemosaicStaticRegCfg->bDeSatEnable     = HI_ISP_DEMOSAIC_DESAT_ENABLE_DEFAULT;
    pstDemosaicStaticRegCfg->bGIntpCtrlEnable = HI_ISP_DEMOSAIC_G_INTP_CONTROL_DEFAULT;
    pstDemosaicStaticRegCfg->bAHDEnable       = HI_ISP_DEMOSAIC_AND_EN_DEFAULT;
    pstDemosaicStaticRegCfg->bDeFakeEnable    = HI_ISP_DEMOSAIC_DE_FAKE_EN_DEFAULT;
    pstDemosaicStaticRegCfg->u8Lpff3          = HI_ISP_DEMOSAIC_LPF_F3_DEFAULT;
    pstDemosaicStaticRegCfg->u8hvBlendLimit1  = HI_ISP_DEMOSAIC_BLENDLIMIT1_DEFAULT;
    pstDemosaicStaticRegCfg->u8hvBlendLimit2  = HI_ISP_DEMOSAIC_BLENDLIMIT2_DEFAULT;
    pstDemosaicStaticRegCfg->u8hvColorRatio   = HI_ISP_DEMOSAIC_HV_RATIO_DEFAULT;
    pstDemosaicStaticRegCfg->u8hvSelection    = HI_ISP_DEMOSAIC_HV_SEL_DEFAULT;
    pstDemosaicStaticRegCfg->u8GClipBitSft    = HI_ISP_DEMOSAIC_G_CLIP_SFT_BIT_DEFAULT;
    pstDemosaicStaticRegCfg->u8DitherMask     = HI_ISP_DITHER_MASK_DEFAULT;
    pstDemosaicStaticRegCfg->u8DitherRatio    = HI_ISP_DITHER_RATIO_DEFAULT;
    pstDemosaicStaticRegCfg->u8CxVarMaxRate   = HI_ISP_DEMOSAIC_CX_VAR_MAX_RATE_DEFAULT;
    pstDemosaicStaticRegCfg->u8CxVarMinRate   = HI_ISP_DEMOSAIC_CX_VAR_MIN_RATE_DEFAULT;
    pstDemosaicStaticRegCfg->u16DeSatThresh1  = HI_ISP_DEMOSAIC_DESAT_THRESH1_DEFAULT;
    pstDemosaicStaticRegCfg->u16DeSatThresh2  = HI_ISP_DEMOSAIC_DESAT_THRESH2_DEFAULT;
    pstDemosaicStaticRegCfg->u16DeSatHig      = HI_ISP_DEMOSAIC_DESAT_HIG_DEFAULT;
    pstDemosaicStaticRegCfg->u16DeSatProtSl   = HI_ISP_DEMOSAIC_DESAT_PROTECT_SL_DEFAULT;
    pstDemosaicStaticRegCfg->u16AhdPart1      = HI_ISP_DEMOSAIC_AHDPART1_DEFAULT;
    pstDemosaicStaticRegCfg->u16AhdPart2      = HI_ISP_DEMOSAIC_AHDPART2_DEFAULT;
    pstDemosaicStaticRegCfg->u16GFThLow       = HI_ISP_NDDM_GF_TH_LOW_DEFAULT;
    pstDemosaicStaticRegCfg->u16GFThHig       = HI_ISP_NDDM_GF_TH_HIGH_DEFAULT;
    pstDemosaicStaticRegCfg->u16CbCrAvgThr    = HI_ISP_DEMOSAIC_CBCR_AVG_THLD_DEFAULT;

    pstDemosaicStaticRegCfg->bResh = HI_TRUE;
}

static HI_VOID DemosaicDynaRegsInitialize(ISP_DEMOSAIC_DYNA_CFG_S *pstDemosaicDynaRegCfg)
{
    HI_U32 n;

    pstDemosaicDynaRegCfg->u8Lpff0              = HI_ISP_DEMOSAIC_LPF_F0_DEFAULT;
    pstDemosaicDynaRegCfg->u8Lpff1              = HI_ISP_DEMOSAIC_LPF_F1_DEFAULT;
    pstDemosaicDynaRegCfg->u8Lpff2              = HI_ISP_DEMOSAIC_LPF_F2_DEFAULT;
    pstDemosaicDynaRegCfg->u8CcHFMaxRatio       = HI_ISP_DEMOSAIC_CC_HF_MAX_RATIO_DEFAULT;
    pstDemosaicDynaRegCfg->u8CcHFMinRatio       = HI_ISP_DEMOSAIC_CC_HF_MIN_RATIO_DEFAULT;
    pstDemosaicDynaRegCfg->u8DitherMax          = HI_ISP_DITH_MAX_DEFAULT;
    pstDemosaicDynaRegCfg->u8FcrGFGain          = HI_ISP_NDDM_FCR_GF_GAIN_DEFAULT;
    pstDemosaicDynaRegCfg->u8AwbGFGainLow       = HI_ISP_NDDM_AWB_GF_GN_LOW_DEFAULT;
    pstDemosaicDynaRegCfg->u8AwbGFGainHig       = HI_ISP_NDDM_AWB_GF_GN_HIGH_DEFAULT;
    pstDemosaicDynaRegCfg->u8AwbGFGainMax       = HI_ISP_NDDM_AWB_GF_GN_MAX_DEFAULT;
    pstDemosaicDynaRegCfg->u8EhcGray            = HI_ISP_NDDM_EHC_GRAY_DEFAULT;
    pstDemosaicDynaRegCfg->u16DeSatLow          = HI_ISP_DEMOSAIC_DESAT_LOW_DEFAULT;
    pstDemosaicDynaRegCfg->u16DeSatProtTh       = HI_ISP_DEMOSAIC_DESAT_PROTECT_TH_DEFAULT;
    pstDemosaicDynaRegCfg->u16HfIntpBlurThLow   = HI_ISP_DEMOSAIC_HF_INPT_BLUR_TH_LOW_DEFAULT;
    pstDemosaicDynaRegCfg->u16HfIntpBlurThHig   = HI_ISP_DEMOSAIC_HF_INPT_BLUR_TH_HIGH_DEFAULT;
    pstDemosaicDynaRegCfg->u16FcrDetLow         = HI_ISP_NDDM_FCR_DET_LOW_DEFAULT;
    pstDemosaicDynaRegCfg->u16FakeCrVarThrLow   = HI_ISP_DEMOSAIC_FAKECR_VAR_THR_LOW_DEFAULT;
    pstDemosaicDynaRegCfg->u16FakeCrVarThrHigh  = HI_ISP_DEMOSAIC_FAKECR_VAR_THR_HIGH_DEFAULT;
    pstDemosaicDynaRegCfg->u8DeFColorCb         = HI_ISP_DEMOSAIC_COLORNOISE_STR_MAX - HI_ISP_DEMOSAIC_DEFCOLOR_CB_DEFAULT;
    pstDemosaicDynaRegCfg->u8DeFColorCr         = HI_ISP_DEMOSAIC_COLORNOISE_STR_MAX - HI_ISP_DEMOSAIC_DEFCOLOR_CR_DEFAULT;    

    for (n = 0; n < HI_ISP_DEMOSAIC_LUT_LENGTH; n++) {
        pstDemosaicDynaRegCfg->au16GFBlurLut[n] = 0;
    }

    pstDemosaicDynaRegCfg->u8ColorNoiseThdY                = HI_EXT_SYSTEM_DEMOSAIC_MANUAL_DESAT_DARK_RANGE_DEFAULT;
    pstDemosaicDynaRegCfg->u8ColorNoiseStrY                = HI_EXT_SYSTEM_DEMOSAIC_MANUAL_DESAT_DARK_STRENGTH_DEFAULT;
    pstDemosaicDynaRegCfg->stProDarkRegion.bCcmProtEn      = HI_ISP_CCM_PROT_EN_DEFAULT;
    pstDemosaicDynaRegCfg->stProDarkRegion.u32DataCCThd0   = HI_ISP_CCM_CC_THD0_DEFAULT;
    pstDemosaicDynaRegCfg->stProDarkRegion.u32DataCCThd1   = HI_ISP_CCM_CC_THD1_DEFAULT;
    pstDemosaicDynaRegCfg->stProDarkRegion.u16CCProtRatio  = HI_ISP_CCM_CC_PROT_RATIO_DEFAULT;
    pstDemosaicDynaRegCfg->stProDarkRegion.u32DataRRThd0   = HI_ISP_CCM_RR_THD0_DEFAULT;
    pstDemosaicDynaRegCfg->stProDarkRegion.u32DataRRThd1   = HI_ISP_CCM_RR_THD1_DEFAULT;
    pstDemosaicDynaRegCfg->stProDarkRegion.u32DataGGThd0   = HI_ISP_CCM_GG_THD0_DEFAULT;
    pstDemosaicDynaRegCfg->stProDarkRegion.u32DataGGThd1   = HI_ISP_CCM_GG_THD1_DEFAULT;
    pstDemosaicDynaRegCfg->stProDarkRegion.u32DataBBThd0   = HI_ISP_CCM_BB_THD0_DEFAULT;
    pstDemosaicDynaRegCfg->stProDarkRegion.u32DataBBThd1   = HI_ISP_CCM_BB_THD1_DEFAULT;
    pstDemosaicDynaRegCfg->stProDarkRegion.u32MaxRGBThd    = HI_ISP_CCM_MAX_RGB_DEFAULT;
    pstDemosaicDynaRegCfg->stProDarkRegion.u16RGBProtRatio = HI_ISP_CCM_RGB_PROT_RATIO_DEFAULT;

    pstDemosaicDynaRegCfg->bUpdateGF = HI_TRUE;
    pstDemosaicDynaRegCfg->bResh     = HI_TRUE;
}

static HI_VOID DemosaicRegsInitialize(VI_PIPE ViPipe, isp_reg_cfg  *pstRegCfg)
{
    HI_U32 i;
    ISP_DEMOSAIC_STATIC_CFG_S *pstDemosaicStaticRegCfg = HI_NULL;
    ISP_DEMOSAIC_DYNA_CFG_S   *pstDemosaicDynaRegCfg   = HI_NULL;

    for (i = 0; i < pstRegCfg->cfg_num; i++) {
        pstDemosaicStaticRegCfg = &pstRegCfg->alg_reg_cfg[i].stDemRegCfg.stStaticRegCfg;
        pstDemosaicDynaRegCfg   = &pstRegCfg->alg_reg_cfg[i].stDemRegCfg.stDynaRegCfg;
        pstRegCfg->alg_reg_cfg[i].stDemRegCfg.bVhdmEnable = HI_TRUE;
        pstRegCfg->alg_reg_cfg[i].stDemRegCfg.bNddmEnable = HI_TRUE;
        DemosaicStaticRegsInitialize(ViPipe, pstDemosaicStaticRegCfg, i);
        DemosaicDynaRegsInitialize(pstDemosaicDynaRegCfg);
    }

    pstRegCfg->cfg_key.bit1DemCfg = 1;
}

static HI_S32 DemosaicCheckCmosParam(VI_PIPE ViPipe, const hi_isp_cmos_demosaic *cmos_demosaic)
{
    HI_U8 i;

    ISP_CHECK_BOOL(cmos_demosaic->enable);

    for (i = 0; i < ISP_AUTO_ISO_STRENGTH_NUM; i++) {
        if (cmos_demosaic->non_dir_mf_detail_ehc_str[i] > HI_ISP_DEMOSAIC_NONDIR_MFDETALEHC_STR_MAX) {
            ISP_ERR_TRACE("Invalid au8NonDirMFDetailEhcStr[%d]!\n", i);
            return HI_ERR_ISP_ILLEGAL_PARAM;
        }
        if (cmos_demosaic->non_dir_hf_detail_ehc_str[i] > 0x10) {
            ISP_ERR_TRACE("Invalid au8NonDirHFDetailEhcStr[%d]!\n", i);
            return HI_ERR_ISP_ILLEGAL_PARAM;
        }
        if ((cmos_demosaic->detail_smooth_range[i] > HI_ISP_DEMOSAIC_DETAIL_SMOOTH_RANGE_MAX) || (cmos_demosaic->detail_smooth_range[i] < HI_ISP_DEMOSAIC_DETAIL_SMOOTH_RANGE_MIN)) {
            ISP_ERR_TRACE("Invalid au8DetailSmoothRange[%d]!\n", i);
            return HI_ERR_ISP_ILLEGAL_PARAM;
        }
    }

    return HI_SUCCESS;
}

static HI_S32 DemosaicExtRegsInitialize(VI_PIPE ViPipe)
{
    HI_U8  i;
    HI_S32 s32Ret;
    HI_FLOAT n_cur, n_pre, n_fct;
    HI_FLOAT drc_compensate = 0.6f;
    HI_FLOAT afsigma[HI_ISP_DEMOSAIC_LUT_LENGTH];
    ISP_DEMOSAIC_S     *pstDemosaic = HI_NULL;
    hi_isp_cmos_default *sns_dft   = HI_NULL;
    isp_usr_ctx          *pstIspCtx   = HI_NULL;

    ISP_GET_CTX(ViPipe, pstIspCtx);
    isp_sensor_get_default(ViPipe, &sns_dft);
    DEMOSAIC_GET_CTX(ViPipe, pstDemosaic);

    pstDemosaic->u8WdrMode = pstIspCtx->sns_wdr_mode;
    pstDemosaic->u8BitDepthPrc = HI_DEMOSAIC_BITDEPTH;

    if (pstDemosaic->u8WdrMode != 0) {
        DemosaicInitFwWdr(ViPipe);

        for (i = 0; i < HI_ISP_DEMOSAIC_LUT_LENGTH; i++) {
            n_cur = (HI_FLOAT)(i * 16);
            n_pre = (HI_FLOAT)(256.0 * pow(n_cur / 256.0, 1.0f / DIV_0_TO_1_FLOAT(drc_compensate))) + 0.5f;
            n_fct = (HI_FLOAT)(ISP_SQR(n_cur / DIV_0_TO_1_FLOAT(n_pre)));

            afsigma[i]  =  n_cur * n_fct;
            pstDemosaic->au32sigma[i] = (HI_U32)(afsigma[i] * ISP_BITFIX(10));
        }
    } else {
        DemosaicInitFwLinear(ViPipe);

        for (i = 0; i < HI_ISP_DEMOSAIC_LUT_LENGTH; i++) {
            pstDemosaic->au32sigma[i]  = (HI_U32)(i * 16 * ISP_BITFIX(10));
        }
    }

    hi_ext_system_demosaic_manual_mode_write(ViPipe, HI_EXT_SYSTEM_DEMOSAIC_MANUAL_MODE_DEFAULT);

    if (IS_WDR_MODE(pstIspCtx->sns_wdr_mode)) {    // Manual:WDR mode
        pstDemosaic->stManual.u8NonDirStr            = HI_EXT_SYSTEM_DEMOSAIC_MANUAL_NONDIRECTION_STRENTH_WDR_DEFAULT;
        pstDemosaic->stManual.u8NonDirMFDetailEhcStr = HI_EXT_SYSTEM_DEMOSAIC_MANUAL_NONDIRECTION_MIDFREQ_DETAILENHANCE_STRENGTH_WDR_DEFAULT;
        pstDemosaic->stManual.u8NonDirHFDetailEhcStr = HI_EXT_SYSTEM_DEMOSAIC_MANUAL_NONDIRECTION_HIGFREQ_DETAILENHANCE_STRENGTH_WDR_DEFAULT;
        pstDemosaic->stManual.u8DetailSmoothRange    = HI_EXT_SYSTEM_DEMOSAIC_MANUAL_DETAIL_SMOOTH_RANGE_WDR_DEFAULT;
    } else {   // Manual:Linear Mode
        pstDemosaic->stManual.u8NonDirStr            = HI_EXT_SYSTEM_DEMOSAIC_MANUAL_NONDIRECTION_STRENTH_LINEAR_DEFAULT;
        pstDemosaic->stManual.u8NonDirMFDetailEhcStr = HI_EXT_SYSTEM_DEMOSAIC_MANUAL_NONDIRECTION_MIDFREQ_DETAILENHANCE_STRENGTH_LINEAR_DEFAULT;
        pstDemosaic->stManual.u8NonDirHFDetailEhcStr = HI_EXT_SYSTEM_DEMOSAIC_MANUAL_NONDIRECTION_HIGFREQ_DETAILENHANCE_STRENGTH_LINEAR_DEFAULT;
        pstDemosaic->stManual.u8DetailSmoothRange    = HI_EXT_SYSTEM_DEMOSAIC_MANUAL_DETAIL_SMOOTH_RANGE_LINEAR_DEFAULT;
    }
    pstDemosaic->stManual.u8ColorNoiseThdF = HI_EXT_SYSTEM_DEMOSAIC_COLORNOISE_CTRL_THDF_DEFAULT;
    pstDemosaic->stManual.u8ColorNoiseStrF = HI_EXT_SYSTEM_DEMOSAIC_COLORNOISE_CTRL_STRF_DEFAULT;
    pstDemosaic->stManual.u8ColorNoiseThdY = HI_EXT_SYSTEM_DEMOSAIC_MANUAL_DESAT_DARK_RANGE_DEFAULT;
    pstDemosaic->stManual.u8ColorNoiseStrY = HI_EXT_SYSTEM_DEMOSAIC_MANUAL_DESAT_DARK_STRENGTH_DEFAULT;

    hi_ext_system_demosaic_manual_nondirection_midfreq_detailenhance_strength_write(ViPipe, pstDemosaic->stManual.u8NonDirMFDetailEhcStr);
    hi_ext_system_demosaic_manual_nondirection_higfreq_detailenhance_strength_write(ViPipe, pstDemosaic->stManual.u8NonDirHFDetailEhcStr);
    hi_ext_system_demosaic_manual_nondirection_strength_write(ViPipe, pstDemosaic->stManual.u8NonDirStr);
    hi_ext_system_demosaic_manual_detail_smooth_range_write(ViPipe, pstDemosaic->stManual.u8DetailSmoothRange);
    hi_ext_system_demosaic_manual_colornoise_thdf_write(ViPipe, pstDemosaic->stManual.u8ColorNoiseThdF);
    hi_ext_system_demosaic_manual_colornoise_strf_write(ViPipe, pstDemosaic->stManual.u8ColorNoiseStrF);
    hi_ext_system_demosaic_manual_desat_dark_range_write(ViPipe, pstDemosaic->stManual.u8ColorNoiseThdY);
    hi_ext_system_demosaic_manual_desat_dark_strength_write(ViPipe, pstDemosaic->stManual.u8ColorNoiseStrY);

    if (sns_dft->key.bit1_demosaic) {
        ISP_CHECK_POINTER(sns_dft->demosaic);

        s32Ret = DemosaicCheckCmosParam(ViPipe, sns_dft->demosaic);

        if (s32Ret != HI_SUCCESS) {
            return s32Ret;
        }

        hi_ext_system_demosaic_enable_write(ViPipe, sns_dft->demosaic->enable);

        memcpy(pstDemosaic->stAuto.au8NonDirStr,            sns_dft->demosaic->non_dir_str,            ISP_AUTO_ISO_STRENGTH_NUM * sizeof(HI_U8));
        memcpy(pstDemosaic->stAuto.au8NonDirMFDetailEhcStr, sns_dft->demosaic->non_dir_mf_detail_ehc_str, ISP_AUTO_ISO_STRENGTH_NUM * sizeof(HI_U8));
        memcpy(pstDemosaic->stAuto.au8NonDirHFDetailEhcStr, sns_dft->demosaic->non_dir_hf_detail_ehc_str, ISP_AUTO_ISO_STRENGTH_NUM * sizeof(HI_U8));
        memcpy(pstDemosaic->stAuto.au8DetailSmoothRange,    sns_dft->demosaic->detail_smooth_range,    ISP_AUTO_ISO_STRENGTH_NUM * sizeof(HI_U8));
    } else {
        hi_ext_system_demosaic_enable_write(ViPipe, HI_TRUE);

        memcpy(pstDemosaic->stAuto.au8NonDirStr,            g_au8NonDirStr,            ISP_AUTO_ISO_STRENGTH_NUM * sizeof(HI_U8));
        memcpy(pstDemosaic->stAuto.au8NonDirMFDetailEhcStr, g_au8NonDirMFDetailEhcStr, ISP_AUTO_ISO_STRENGTH_NUM * sizeof(HI_U8));
        memcpy(pstDemosaic->stAuto.au8NonDirHFDetailEhcStr, g_au8NonDirHFDetailEhcStr, ISP_AUTO_ISO_STRENGTH_NUM * sizeof(HI_U8));
        memcpy(pstDemosaic->stAuto.au8DetailSmoothRange,    g_au8DetailSmoothRange,    ISP_AUTO_ISO_STRENGTH_NUM * sizeof(HI_U8));
    }
    memcpy(pstDemosaic->stAuto.au8ColorNoiseThdF, g_au8ColorNoiseThdF, ISP_AUTO_ISO_STRENGTH_NUM * sizeof(HI_U8));
    memcpy(pstDemosaic->stAuto.au8ColorNoiseStrF, g_au8ColorNoiseStrF, ISP_AUTO_ISO_STRENGTH_NUM * sizeof(HI_U8));
    memcpy(pstDemosaic->stAuto.au8ColorNoiseThdY, g_au8ColorNoiseThdY, ISP_AUTO_ISO_STRENGTH_NUM * sizeof(HI_U8));
    memcpy(pstDemosaic->stAuto.au8ColorNoiseStrY, g_au8ColorNoiseStrY, ISP_AUTO_ISO_STRENGTH_NUM * sizeof(HI_U8));

    for (i = 0; i < ISP_AUTO_ISO_STRENGTH_NUM; i++) { // Auto
        hi_ext_system_demosaic_auto_nondirection_strength_write(ViPipe, i, pstDemosaic->stAuto.au8NonDirStr[i]);
        hi_ext_system_demosaic_auto_nondirection_midfreq_detailenhance_strength_write(ViPipe, i, pstDemosaic->stAuto.au8NonDirMFDetailEhcStr[i]);
        hi_ext_system_demosaic_auto_nondirection_higfreq_detailenhance_strength_write(ViPipe, i, pstDemosaic->stAuto.au8NonDirHFDetailEhcStr[i]);
        hi_ext_system_demosaic_auto_detail_smooth_range_write(ViPipe, i, pstDemosaic->stAuto.au8DetailSmoothRange[i]);
        hi_ext_system_demosaic_auto_colornoise_thdf_write(ViPipe, i, pstDemosaic->stAuto.au8ColorNoiseThdF[i]);
        hi_ext_system_demosaic_auto_colornoise_strf_write(ViPipe, i, pstDemosaic->stAuto.au8ColorNoiseStrF[i]);
        hi_ext_system_demosaic_auto_desat_dark_range_write(ViPipe, i, pstDemosaic->stAuto.au8ColorNoiseThdY[i]);
        hi_ext_system_demosaic_auto_desat_dark_strength_write(ViPipe, i, pstDemosaic->stAuto.au8ColorNoiseStrY[i]);
    }

    return HI_SUCCESS;
}

static HI_S32 Demosaic_SetLongFrameMode(VI_PIPE ViPipe)
{
    HI_U8  i;
    HI_FLOAT n_cur, n_pre, n_fct;
    HI_FLOAT drc_compensate = 0.6f;
    HI_FLOAT afsigma[HI_ISP_DEMOSAIC_LUT_LENGTH];
    ISP_DEMOSAIC_S     *pstDemosaic = HI_NULL;
    isp_usr_ctx          *pstIspCtx   = HI_NULL;

    ISP_GET_CTX(ViPipe, pstIspCtx);
    DEMOSAIC_GET_CTX(ViPipe, pstDemosaic);

    if (IS_LINEAR_MODE(pstIspCtx->sns_wdr_mode) ||
        (pstIspCtx->linkage.fswdr_mode == ISP_FSWDR_LONG_FRAME_MODE) ||
        (pstIspCtx->linkage.fswdr_mode == ISP_FSWDR_AUTO_LONG_FRAME_MODE)) {
        DemosaicInitFwLinear(ViPipe);

        for (i = 0; i < HI_ISP_DEMOSAIC_LUT_LENGTH; i++) {
            pstDemosaic->au32sigma[i]  = (HI_U32)(i * 16 * ISP_BITFIX(10));
        }
    } else {
        DemosaicInitFwWdr(ViPipe);

        for (i = 0; i < HI_ISP_DEMOSAIC_LUT_LENGTH; i++) {
            n_cur = (HI_FLOAT)(i * 16);
            n_pre = (HI_FLOAT)(256.0 * pow(n_cur / 256.0, 1.0f / DIV_0_TO_1_FLOAT(drc_compensate))) + 0.5f;
            n_fct = (HI_FLOAT)(ISP_SQR(n_cur / DIV_0_TO_1_FLOAT(n_pre)));
            afsigma[i]  =  n_cur * n_fct;
            pstDemosaic->au32sigma[i] = (HI_U32)(afsigma[i] * ISP_BITFIX(10));
        }
    }

    return HI_SUCCESS;
}

static HI_S32 DemosaicReadExtregs(VI_PIPE ViPipe)
{
    ISP_DEMOSAIC_S *pstDemosaic = HI_NULL;
    HI_U8 i;

    DEMOSAIC_GET_CTX(ViPipe, pstDemosaic);

    pstDemosaic->bDemAttrUpdate = hi_ext_system_demosaic_attr_update_en_read(ViPipe);
    hi_ext_system_demosaic_attr_update_en_write(ViPipe, HI_FALSE);

    if (pstDemosaic->bDemAttrUpdate) {
        pstDemosaic->enOpType = hi_ext_system_demosaic_manual_mode_read(ViPipe);

        if (pstDemosaic->enOpType == OP_TYPE_AUTO) {
            for (i = 0; i < ISP_AUTO_ISO_STRENGTH_NUM; i++) {
                pstDemosaic->stAuto.au8NonDirStr[i]            = hi_ext_system_demosaic_auto_nondirection_strength_read(ViPipe, i);
                pstDemosaic->stAuto.au8NonDirMFDetailEhcStr[i] = hi_ext_system_demosaic_auto_nondirection_midfreq_detailenhance_strength_read(ViPipe, i);
                pstDemosaic->stAuto.au8NonDirHFDetailEhcStr[i] = hi_ext_system_demosaic_auto_nondirection_higfreq_detailenhance_strength_read(ViPipe, i);
                pstDemosaic->stAuto.au8DetailSmoothRange[i]    = hi_ext_system_demosaic_auto_detail_smooth_range_read(ViPipe, i);
                pstDemosaic->stAuto.au8ColorNoiseThdF[i]       = hi_ext_system_demosaic_auto_colornoise_thdf_read(ViPipe, i);
                pstDemosaic->stAuto.au8ColorNoiseStrF[i]       = hi_ext_system_demosaic_auto_colornoise_strf_read(ViPipe, i);
                pstDemosaic->stAuto.au8ColorNoiseThdY[i]       = hi_ext_system_demosaic_auto_desat_dark_range_read(ViPipe, i);
                pstDemosaic->stAuto.au8ColorNoiseStrY[i]       = hi_ext_system_demosaic_auto_desat_dark_strength_read(ViPipe, i);
            }
        } else if (pstDemosaic->enOpType == OP_TYPE_MANUAL) {
            pstDemosaic->stManual.u8NonDirStr            = hi_ext_system_demosaic_manual_nondirection_strength_read(ViPipe);
            pstDemosaic->stManual.u8NonDirMFDetailEhcStr = hi_ext_system_demosaic_manual_nondirection_midfreq_detailenhance_strength_read(ViPipe);
            pstDemosaic->stManual.u8NonDirHFDetailEhcStr = hi_ext_system_demosaic_manual_nondirection_higfreq_detailenhance_strength_read(ViPipe);
            pstDemosaic->stManual.u8DetailSmoothRange    = hi_ext_system_demosaic_manual_detail_smooth_range_read(ViPipe);
            pstDemosaic->stManual.u8ColorNoiseThdF       = hi_ext_system_demosaic_manual_colornoise_thdf_read(ViPipe);
            pstDemosaic->stManual.u8ColorNoiseStrF       = hi_ext_system_demosaic_manual_colornoise_strf_read(ViPipe);
            pstDemosaic->stManual.u8ColorNoiseThdY       = hi_ext_system_demosaic_manual_desat_dark_range_read(ViPipe);
            pstDemosaic->stManual.u8ColorNoiseStrY       = hi_ext_system_demosaic_manual_desat_dark_strength_read(ViPipe);
        }
    }
    return HI_SUCCESS;
}

static HI_S32 DemosaicProcWrite(VI_PIPE ViPipe, hi_isp_ctrl_proc_write *pstProc)
{
    hi_isp_ctrl_proc_write stProcTmp;
    ISP_DEMOSAIC_S *pstDemosaic = HI_NULL;

    DEMOSAIC_GET_CTX(ViPipe, pstDemosaic);
    ISP_CHECK_POINTER(pstDemosaic);

    if ((pstProc->proc_buff == HI_NULL)
        || (pstProc->buff_len == 0)) {
        return HI_FAILURE;
    }

    stProcTmp.proc_buff = pstProc->proc_buff;
    stProcTmp.buff_len = pstProc->buff_len;

    ISP_PROC_PRINTF(&stProcTmp, pstProc->write_len,
                    "-----DEMOSAIC INFO-------------------------------------------------------------\n");

    ISP_PROC_PRINTF(&stProcTmp, pstProc->write_len,
                    "%16s\n",
                    "Enable");

    ISP_PROC_PRINTF(&stProcTmp, pstProc->write_len,
                    "%16u\n",
                    pstDemosaic->bEnable);

    pstProc->write_len += 1;

    return HI_SUCCESS;
}


HI_S32 ISP_DemosaicInit(VI_PIPE ViPipe, HI_VOID *pRegCfg)
{
    HI_S32 s32Ret = HI_SUCCESS;
    ISP_DEMOSAIC_S *pstDemosaic = HI_NULL;
    isp_reg_cfg  *pstRegCfg   = (isp_reg_cfg *)pRegCfg;

    s32Ret = DemosaicCtxInit(ViPipe);
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    DEMOSAIC_GET_CTX(ViPipe, pstDemosaic);
    ISP_CHECK_POINTER(pstDemosaic);

    DemosaicRegsInitialize(ViPipe, pstRegCfg);
    s32Ret = DemosaicExtRegsInitialize(ViPipe);
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    pstDemosaic->bInit = HI_TRUE;
    return HI_SUCCESS;
}

#define  EPS (0.000001f)
#define  COL_ISO      0
#define  COL_K        1
#define  COL_B        2
static HI_FLOAT Demosaic_getAlphafromNoiseLut(HI_FLOAT (*pRecord)[3], HI_U16 recordNum, HI_S32 iso)
{
    HI_S32 i = 0;
    HI_FLOAT  y_diff = 0, x_diff = 0;
    HI_FLOAT k = 0.0f;

    if (recordNum > BAYER_CALIBTAION_MAX_NUM) {
        k = pRecord[BAYER_CALIBTAION_MAX_NUM - 1][COL_K];
        return k;
    }

    // record: iso - k
    if (iso <= pRecord[0][COL_ISO]) {
        k = pRecord[0][COL_K];
    }

    if (iso >= pRecord[recordNum - 1][COL_ISO]) {
        k = pRecord[recordNum - 1][COL_K];
    }

    for (i = 0; i < recordNum - 1; i++) {
        if (iso >= pRecord[i][COL_ISO] && iso <= pRecord[i + 1][COL_ISO]) {
            x_diff = pRecord[i + 1][COL_ISO] - pRecord[i][COL_ISO];  // iso diff
            y_diff = pRecord[i + 1][COL_K]  - pRecord[i][COL_K];     // k diff
            k = pRecord[i][COL_K] + y_diff * (iso - pRecord[i][COL_ISO]) / DIV_0_TO_1_FLOAT(x_diff + EPS);
        }
    }

    return k;
}

static HI_S32 Demosaic_GFBlurLut(ISP_DEMOSAIC_S *pstDemosaic, VI_PIPE ViPipe, ISP_DEMOSAIC_DYNA_CFG_S *pstDmCfg, HI_U8 u8NonDirStr, HI_U32 u32Iso)
{
    HI_U8  n = 0;
    HI_U32 alpha, sigma;
    HI_U64 u64sigma;
    hi_isp_cmos_default *sns_dft = HI_NULL;
    isp_sensor_get_default(ViPipe, &sns_dft);
    HI_S32   bitScale   =  ISP_BITFIX(pstDemosaic->u8BitDepthPrc - 8);
    HI_FLOAT fCalibrationCoef = 0.0f;

    fCalibrationCoef = Demosaic_getAlphafromNoiseLut(sns_dft->noise_calibration.calibration_coef, sns_dft->noise_calibration.calibration_lut_num, u32Iso);
    alpha = (HI_U32)(fCalibrationCoef * ISP_BITFIX(10));

    for (n = 0; n < HI_ISP_DEMOSAIC_LUT_LENGTH; n++) {
        u64sigma  = (HI_U64)pstDemosaic->au32sigma[n] * alpha;
        u64sigma *= u8NonDirStr;
        sigma  = MIN2((u64sigma >> 14), 0xffffffff);
        sigma  = (HI_U32)Sqrt32(sigma);
        sigma = (sigma * bitScale) >> 5;
        pstDmCfg->au16GFBlurLut[n] = MIN2(sigma, ISP_BITMASK(pstDemosaic->u8BitDepthPrc));
    }

    pstDmCfg->au16GFBlurLut[0] = pstDmCfg->au16GFBlurLut[1] / 2;
    pstDmCfg->bUpdateGF       = HI_TRUE;

    return HI_SUCCESS;
}

static HI_VOID DemosaicDarkProCfg(ISP_DEMOSAIC_DYNA_CFG_S* pstDmCfg)
{
    HI_U32 u32DataRBThd0;
    HI_U32 u32DataGThd0;
    HI_U32 u32DataRBThd1;
    HI_U32 u32DataGThd1;

    if (pstDmCfg->u8ColorNoiseStrY == 0) {
        pstDmCfg->stProDarkRegion.bCcmProtEn = 0;
    } else {
        pstDmCfg->stProDarkRegion.bCcmProtEn = 1;
    }

    pstDmCfg->stProDarkRegion.u16CCProtRatio     = 257 - pstDmCfg->u8ColorNoiseStrY;
    pstDmCfg->stProDarkRegion.u16RGBProtRatio    = 257 - pstDmCfg->u8ColorNoiseStrY;

    u32DataRBThd0 = pstDmCfg->u8ColorNoiseThdY;

    if (pstDmCfg->u8ColorNoiseThdY > 48) {
        u32DataGThd0 = pstDmCfg->u8ColorNoiseThdY - 24;
    } else {
        u32DataGThd0 = pstDmCfg->u8ColorNoiseThdY >> 1;
    }

    u32DataRBThd0 = MAX2(u32DataRBThd0, 2);
    u32DataGThd0 = MAX2(u32DataGThd0, 2);
    u32DataRBThd0 = MIN2(u32DataRBThd0, 255);
    u32DataGThd0 = MIN2(u32DataGThd0, 255);
    u32DataRBThd1 = u32DataRBThd0 + 70;
    u32DataGThd1 = u32DataGThd0 + 54;
    u32DataRBThd1 = MIN2(u32DataRBThd1, 511);
    u32DataGThd1 = MIN2(u32DataGThd1, 511);

    pstDmCfg->stProDarkRegion.u32DataCCThd0      = 0;
    pstDmCfg->stProDarkRegion.u32DataCCThd1      = u32DataRBThd0 << 3;
    pstDmCfg->stProDarkRegion.u32DataRRThd0      = u32DataRBThd0 << 3;
    pstDmCfg->stProDarkRegion.u32DataRRThd1      = u32DataRBThd1 << 3;
    pstDmCfg->stProDarkRegion.u32DataGGThd0      = u32DataGThd0 << 3;
    pstDmCfg->stProDarkRegion.u32DataGGThd1      = u32DataGThd1 << 3;
    pstDmCfg->stProDarkRegion.u32DataBBThd0      = u32DataRBThd0 << 3;
    pstDmCfg->stProDarkRegion.u32DataBBThd1      = u32DataRBThd1 << 3;
    pstDmCfg->stProDarkRegion.u32MaxRGBThd       = 0xFFF;
}

static HI_VOID DemosaicCfg(ISP_DEMOSAIC_DYNA_CFG_S *pstDmCfg, ISP_DEMOSAIC_S *pstDemosaic, HI_U8 u8IsoIndexUpper, HI_U8 u8IsoIndexLower, HI_U32 u32ISO2, HI_U32 u32ISO1, HI_U32 u32Iso)
{
    pstDmCfg->u8CcHFMaxRatio     = (HI_U8) LinearInter(u32Iso, u32ISO1, pstDemosaic->au8LutCcHFMaxRatio[u8IsoIndexLower], \
                                                               u32ISO2, pstDemosaic->au8LutCcHFMaxRatio[u8IsoIndexUpper]);
    pstDmCfg->u8CcHFMinRatio     = (HI_U16)LinearInter(u32Iso, u32ISO1, pstDemosaic->au8LutCcHFMinRatio[u8IsoIndexLower], \
                                                               u32ISO2, pstDemosaic->au8LutCcHFMinRatio[u8IsoIndexUpper]);
    pstDmCfg->u8FcrGFGain        = (HI_U8) LinearInter(u32Iso, u32ISO1, pstDemosaic->as8LutFcrGFGain[u8IsoIndexLower], \
                                                               u32ISO2, pstDemosaic->as8LutFcrGFGain[u8IsoIndexUpper]);
    pstDmCfg->u8AwbGFGainLow     = (HI_U8) LinearInter(u32Iso, u32ISO1, pstDemosaic->au8LutAwbGFGainLow[u8IsoIndexLower], \
                                                               u32ISO2, pstDemosaic->au8LutAwbGFGainLow[u8IsoIndexUpper]);
    pstDmCfg->u8AwbGFGainHig     = (HI_U8) LinearInter(u32Iso, u32ISO1, pstDemosaic->au8LutAwbGFGainHig[u8IsoIndexLower], \
                                                               u32ISO2, pstDemosaic->au8LutAwbGFGainHig[u8IsoIndexUpper]);
    pstDmCfg->u8AwbGFGainMax     = (HI_U8) LinearInter(u32Iso, u32ISO1, pstDemosaic->au8LutAwbGFGainMax[u8IsoIndexLower], \
                                                               u32ISO2, pstDemosaic->au8LutAwbGFGainMax[u8IsoIndexUpper]);
    pstDmCfg->u16FcrDetLow       = (HI_U16)LinearInter(u32Iso, u32ISO1, pstDemosaic->au16LutFcrDetLow[u8IsoIndexLower], \
                                                               u32ISO2, pstDemosaic->au16LutFcrDetLow[u8IsoIndexUpper]);
    pstDmCfg->u16DeSatLow        = (HI_U16)LinearInter(u32Iso, u32ISO1, pstDemosaic->au16LutDeSatLow[u8IsoIndexLower], \
                                                               u32ISO2, pstDemosaic->au16LutDeSatLow[u8IsoIndexUpper]);
    pstDmCfg->u16DeSatProtTh     = (HI_U16)LinearInter(u32Iso, u32ISO1, pstDemosaic->au16LutDeSatProtTh[u8IsoIndexLower], \
                                                               u32ISO2, pstDemosaic->au16LutDeSatProtTh[u8IsoIndexUpper]);
    pstDmCfg->u16HfIntpBlurThLow = (HI_U16)LinearInter(u32Iso, u32ISO1, pstDemosaic->au16HfIntpBlurThLow[u8IsoIndexLower], \
                                                               u32ISO2, pstDemosaic->au16HfIntpBlurThLow[u8IsoIndexUpper]);
    pstDmCfg->u16HfIntpBlurThHig = (HI_U16)LinearInter(u32Iso, u32ISO1, pstDemosaic->au16HfIntpBlurThHig[u8IsoIndexLower], \
                                                               u32ISO2, pstDemosaic->au16HfIntpBlurThHig[u8IsoIndexUpper]);
}

static HI_S32 ISP_DemosaicFw(HI_U32 u32Iso, VI_PIPE ViPipe, HI_U8 u8CurBlk, isp_reg_cfg *pstReg)
{
    HI_U8  u8IsoIndexUpper, u8IsoIndexLower;
    HI_U8  u8FilterCoefIndex = 0;
    HI_U8  u8NonDirStr = 0;
    HI_U8  u8TempDeF;
    HI_U32 u32ISO1 = 0;
    HI_U32 u32ISO2 = 0;
    ISP_DEMOSAIC_DYNA_CFG_S *pstDmCfg = &pstReg->alg_reg_cfg[u8CurBlk].stDemRegCfg.stDynaRegCfg;
    ISP_DEMOSAIC_S *pstDemosaic = HI_NULL;

    DEMOSAIC_GET_CTX(ViPipe, pstDemosaic);

    u8IsoIndexUpper = GetIsoIndex(u32Iso);
    u8IsoIndexLower = MAX2((HI_S8)u8IsoIndexUpper - 1, 0);

    u32ISO1 = g_au32IsoLut[u8IsoIndexLower];
    u32ISO2 = g_au32IsoLut[u8IsoIndexUpper];

    if (pstDemosaic->enOpType == OP_TYPE_AUTO) {
        u8NonDirStr           = (HI_U8)LinearInter(u32Iso, u32ISO1, pstDemosaic->stAuto.au8NonDirStr[u8IsoIndexLower], \
                                                           u32ISO2, pstDemosaic->stAuto.au8NonDirStr[u8IsoIndexUpper]);
        pstDmCfg->u8EhcGray   = (HI_U8)LinearInter(u32Iso, u32ISO1, pstDemosaic->stAuto.au8NonDirMFDetailEhcStr[u8IsoIndexLower], \
                                                           u32ISO2, pstDemosaic->stAuto.au8NonDirMFDetailEhcStr[u8IsoIndexUpper]);
        pstDmCfg->u8DitherMax = (HI_U8)LinearInter(u32Iso, u32ISO1, pstDemosaic->stAuto.au8NonDirHFDetailEhcStr[u8IsoIndexLower], \
                                                           u32ISO2, pstDemosaic->stAuto.au8NonDirHFDetailEhcStr[u8IsoIndexUpper]);
        u8FilterCoefIndex     = (HI_U8)LinearInter(u32Iso, u32ISO1, pstDemosaic->stAuto.au8DetailSmoothRange[u8IsoIndexLower], \
                                                           u32ISO2, pstDemosaic->stAuto.au8DetailSmoothRange[u8IsoIndexUpper]);
        pstDmCfg->u16FakeCrVarThrLow = (HI_U16)LinearInter(u32Iso, u32ISO1, pstDemosaic->stAuto.au8ColorNoiseThdF[u8IsoIndexLower], \
                                                           u32ISO2, pstDemosaic->stAuto.au8ColorNoiseThdF[u8IsoIndexUpper]);
        u8TempDeF             = (HI_U8)LinearInter(u32Iso, u32ISO1, pstDemosaic->stAuto.au8ColorNoiseStrF[u8IsoIndexLower], \
                                                           u32ISO2, pstDemosaic->stAuto.au8ColorNoiseStrF[u8IsoIndexUpper]);
        pstDmCfg->u8DeFColorCr = HI_ISP_DEMOSAIC_COLORNOISE_STR_MAX - u8TempDeF;
        pstDmCfg->u8DeFColorCb = HI_ISP_DEMOSAIC_COLORNOISE_STR_MAX - u8TempDeF;
        pstDmCfg->u8ColorNoiseThdY   = (HI_U8)LinearInter(u32Iso, u32ISO1, pstDemosaic->stAuto.au8ColorNoiseThdY[u8IsoIndexLower], \
                                                                   u32ISO2, pstDemosaic->stAuto.au8ColorNoiseThdY[u8IsoIndexUpper]);
        pstDmCfg->u8ColorNoiseStrY   = (HI_U8)LinearInter(u32Iso, u32ISO1, pstDemosaic->stAuto.au8ColorNoiseStrY[u8IsoIndexLower], \
                                                                  u32ISO2, pstDemosaic->stAuto.au8ColorNoiseStrY[u8IsoIndexUpper]);
    } else if (pstDemosaic->enOpType == OP_TYPE_MANUAL) {
        u8NonDirStr               = pstDemosaic->stManual.u8NonDirStr;
        pstDmCfg->u8EhcGray       = pstDemosaic->stManual.u8NonDirMFDetailEhcStr;
        pstDmCfg->u8DitherMax     = pstDemosaic->stManual.u8NonDirHFDetailEhcStr;
        u8FilterCoefIndex         = pstDemosaic->stManual.u8DetailSmoothRange;
        pstDmCfg->u16FakeCrVarThrLow = (HI_U16)pstDemosaic->stManual.u8ColorNoiseThdF;
        pstDmCfg->u8DeFColorCb    = HI_ISP_DEMOSAIC_COLORNOISE_STR_MAX - pstDemosaic->stManual.u8ColorNoiseStrF;
        pstDmCfg->u8DeFColorCr    = HI_ISP_DEMOSAIC_COLORNOISE_STR_MAX - pstDemosaic->stManual.u8ColorNoiseStrF;
        pstDmCfg->u8ColorNoiseThdY= pstDemosaic->stManual.u8ColorNoiseThdY;
        pstDmCfg->u8ColorNoiseStrY= pstDemosaic->stManual.u8ColorNoiseStrY;
    }
    DemosaicDarkProCfg(pstDmCfg);

    DemosaicCfg(pstDmCfg, pstDemosaic, u8IsoIndexUpper, u8IsoIndexLower, u32ISO2, u32ISO1, u32Iso);
    Demosaic_GFBlurLut(pstDemosaic, ViPipe, pstDmCfg, u8NonDirStr, u32Iso);

    if (u8FilterCoefIndex < 1) {
        u8FilterCoefIndex = 1;
    }
    if (u8FilterCoefIndex > 7) {
        u8FilterCoefIndex = 7;
    }

    pstDmCfg->u8Lpff0 = g_as32DemosaicFilterCoef[u8FilterCoefIndex - 1][0];
    pstDmCfg->u8Lpff1 = g_as32DemosaicFilterCoef[u8FilterCoefIndex - 1][1];
    pstDmCfg->u8Lpff2 = g_as32DemosaicFilterCoef[u8FilterCoefIndex - 1][2];

    pstDmCfg->bResh = HI_TRUE;

    return  HI_SUCCESS;
}

static HI_BOOL __inline CheckDemosaicOpen(ISP_DEMOSAIC_S *pstDemosaic)
{
    return (pstDemosaic->bEnable == HI_TRUE);
}

HI_S32 ISP_DemosaicRun(VI_PIPE ViPipe, const HI_VOID *pStatInfo, HI_VOID *pRegCfg, HI_S32 s32Rsv)
{
    HI_U8  i;
    isp_reg_cfg *pstReg = (isp_reg_cfg *)pRegCfg;
    isp_usr_ctx *pstIspCtx = HI_NULL;
    ISP_DEMOSAIC_S *pstDemosaic = HI_NULL;

    ISP_GET_CTX(ViPipe, pstIspCtx);
    DEMOSAIC_GET_CTX(ViPipe, pstDemosaic);
    ISP_CHECK_POINTER(pstDemosaic);

    /* calculate every two interrupts */
    if (!pstDemosaic->bInit) {
        return HI_SUCCESS;
    }

    if (pstIspCtx->linkage.fswdr_mode != pstIspCtx->linkage.pre_fswdr_mode) {
        Demosaic_SetLongFrameMode(ViPipe);
    }

    if ((pstIspCtx->frame_cnt % 2 != 0) && (pstIspCtx->linkage.snap_state != HI_TRUE)) {
        return HI_SUCCESS;
    }

    pstDemosaic->bEnable = hi_ext_system_demosaic_enable_read(ViPipe);

    for (i = 0; i < pstReg->cfg_num; i++) {
        pstReg->alg_reg_cfg[i].stDemRegCfg.bVhdmEnable = pstDemosaic->bEnable;
        pstReg->alg_reg_cfg[i].stDemRegCfg.bNddmEnable = pstDemosaic->bEnable;
    }

    pstReg->cfg_key.bit1DemCfg = 1;

    /* check hardware setting */
    if (!CheckDemosaicOpen(pstDemosaic)) {
        return HI_SUCCESS;
    }

    DemosaicReadExtregs(ViPipe);

    for (i = 0; i < pstIspCtx->block_attr.block_num; i++) {
        ISP_DemosaicFw(pstIspCtx->linkage.iso, ViPipe, i, pstReg);
    }

    return HI_SUCCESS;
}

HI_S32 ISP_DemosaicCtrl(VI_PIPE ViPipe, HI_U32 u32Cmd, HI_VOID *pValue)
{
    ISP_DEMOSAIC_S *pstDemosaic = HI_NULL;
    isp_reg_cfg_attr   *pRegCfg     = HI_NULL;

    switch (u32Cmd) {
        case ISP_WDR_MODE_SET :
            ISP_REGCFG_GET_CTX(ViPipe, pRegCfg);
            DEMOSAIC_GET_CTX(ViPipe, pstDemosaic);
            ISP_CHECK_POINTER(pRegCfg);
            ISP_CHECK_POINTER(pstDemosaic);

            pstDemosaic->bInit = HI_FALSE;
            ISP_DemosaicInit(ViPipe, (HI_VOID *)&pRegCfg->reg_cfg);
            break;

        case ISP_PROC_WRITE:
            DemosaicProcWrite(ViPipe, (hi_isp_ctrl_proc_write *)pValue);
            break;

        default :
            break;
    }

    return HI_SUCCESS;
}

HI_S32 ISP_DemosaicExit(VI_PIPE ViPipe)
{
    HI_U16 i;
    isp_reg_cfg_attr *pRegCfg    = HI_NULL;

    ISP_REGCFG_GET_CTX(ViPipe, pRegCfg);

    for (i = 0; i < pRegCfg->reg_cfg.cfg_num; i++) {
        pRegCfg->reg_cfg.alg_reg_cfg[i].stDemRegCfg.bNddmEnable = HI_FALSE;
        pRegCfg->reg_cfg.alg_reg_cfg[i].stDemRegCfg.bVhdmEnable = HI_FALSE;
    }

    pRegCfg->reg_cfg.cfg_key.bit1DemCfg = 1;

    DemosaicCtxExit(ViPipe);

    return HI_SUCCESS;
}

HI_S32 isp_alg_register_demosaic(VI_PIPE ViPipe)
{
    isp_usr_ctx *pstIspCtx = HI_NULL;
    isp_alg_node *pstAlgs = HI_NULL;

    ISP_GET_CTX(ViPipe, pstIspCtx);
    ISP_ALG_CHECK(pstIspCtx->alg_key.bit1_demosaic);
    pstAlgs = ISP_SearchAlg(pstIspCtx->algs);
    ISP_CHECK_POINTER(pstAlgs);

    pstAlgs->alg_type = ISP_ALG_DEMOSAIC;
    pstAlgs->alg_func.pfn_alg_init = ISP_DemosaicInit;
    pstAlgs->alg_func.pfn_alg_run  = ISP_DemosaicRun;
    pstAlgs->alg_func.pfn_alg_ctrl = ISP_DemosaicCtrl;
    pstAlgs->alg_func.pfn_alg_exit = ISP_DemosaicExit;
    pstAlgs->used = HI_TRUE;

    return HI_SUCCESS;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
