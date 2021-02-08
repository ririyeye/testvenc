/*
* Copyright (C) Hisilicon Technologies Co., Ltd. 2012-2019. All rights reserved.
* Description:
* Author: Hisilicon multimedia software group
* Create: 2011/06/28
*/

#include <math.h>
#include "isp_config.h"
#include "isp_alg.h"
#include "isp_sensor.h"
#include "isp_ext_config.h"
#include "isp_math_utils.h"
#include "isp_proc.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

#define HI_WDR_BITDEPTH                  (12)
#define HI_ISP_WDR_NOISE_CWEIGHT_DEFAULT (3)
#define HI_ISP_WDR_NOISE_GWEIGHT_DEFAULT (3)
#define HI_WDR_SQRT_GAIN_BITDEP          (16)
#define WDR_CLIP3(low, high, x) (MAX2( MIN2((x), high), low))

static const  HI_S32 g_as32NoiseAgainSet[NoiseSet_EleNum] = { 1, 2, 4, 8, 16, 32, 64 };
static const  HI_S32 g_as32NoiseFloorSet[NoiseSet_EleNum] = { 1, 2, 3, 6, 11, 17, 21 };
static const  HI_S32 g_as32FusionThr[WDR_MAX_FRAME] = {3855, 3000};
static const  HI_U8  g_au8lutMDTLowThr[ISP_AUTO_ISO_STRENGTH_NUM] = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};                     // u4.2
static const  HI_U8  g_au8lutMDTHigThr[ISP_AUTO_ISO_STRENGTH_NUM] = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};                  // u4.2

typedef struct hiISP_FS_WDR_S {
    /* Public */
    // fw input
    HI_BOOL     bCoefUpdateEn;
    HI_BOOL     bMdtEn;
    HI_BOOL     bFusionMode;
    HI_BOOL     bWDREn;
    HI_BOOL     bErosionEn;
    HI_BOOL     bShortExpoChk;
    HI_BOOL     bMdRefFlicker;
    HI_BOOL     bManualMode;
    HI_U8       u8MdThrLowGain;                         // u4.2, [0,63]
    HI_U8       u8MdThrHigGain;                         // u4.2, [0,63]
    HI_U8       au8MdThrLowGain[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8       au8MdThrHigGain[ISP_AUTO_ISO_STRENGTH_NUM];
    HI_U8       u8BitDepthPrc;
    HI_U8       u8BitDepthInValid;
    HI_U8       u8FramesMerge;
    HI_U8       u8NosGWgtMod;
    HI_U8       u8NosCWgtMod;
    HI_U8       u8MdtLBld;
    HI_U8       u8NoiseModelCoef;
    HI_U8       u8NoiseRatioRg;
    HI_U8       u8NoiseRatioBg;

    HI_U8       u8MdtStillThr;
    HI_U8       u8MdtFullThr;
    HI_U8       u8FullMotSigWgt;
    HI_U8       au8FloorSet[NoiseSet_EleNum];
    HI_U16      u16ShortThr;
    HI_U16      u16LongThr;
    HI_U16      u16FusionBarrier0;      // U14.0
    HI_U16      u16FusionBarrier1;      // U14.0
    HI_U16      u16FusionBarrier2;      // U14.0
    HI_U16      u16FusionBarrier3;      // U14.0
    HI_U32      u32PreIso129;
    HI_U32      u32PreAgain;
    HI_S32      s32PreMDTNoise;
    HI_U32      au32AgainSet[NoiseSet_EleNum];
    HI_U8       u8TextureThdWgt;

    HI_BOOL     bForceLong;         // u1.0,[0,1]
    HI_U16      u16ForceLongLowThr; // u11.0,[0,4095]
    HI_U16      u16ForceLongHigThr; // u11.0,[0,4095]
    HI_U16      u16ShortCheckThd; // u11.0,[0,4095]

    HI_U16      u16FusionSaturateThd;          // u12.0,[0,4095]

    ISP_BNR_MODE_E enBnrMode;
} ISP_FS_WDR_S;

ISP_FS_WDR_S *g_pastFSWDRCtx[ISP_MAX_PIPE_NUM] = {HI_NULL};

#define FS_WDR_GET_CTX(dev, pstCtx)   (pstCtx = g_pastFSWDRCtx[dev])
#define FS_WDR_SET_CTX(dev, pstCtx)   (g_pastFSWDRCtx[dev] = pstCtx)
#define FS_WDR_RESET_CTX(dev)         (g_pastFSWDRCtx[dev] = HI_NULL)

HI_S32 FrameWDRCtxInit(VI_PIPE ViPipe)
{
    ISP_FS_WDR_S *pastFSWDRCtx = HI_NULL;

    FS_WDR_GET_CTX(ViPipe, pastFSWDRCtx);

    if (pastFSWDRCtx == HI_NULL) {
        pastFSWDRCtx = (ISP_FS_WDR_S *)ISP_MALLOC(sizeof(ISP_FS_WDR_S));
        if (pastFSWDRCtx == HI_NULL) {
            ISP_ERR_TRACE("Isp[%d] FsWDRCtx malloc memory failed!\n", ViPipe);
            return HI_ERR_ISP_NOMEM;
        }
    }

    memset(pastFSWDRCtx, 0, sizeof(ISP_FS_WDR_S));

    FS_WDR_SET_CTX(ViPipe, pastFSWDRCtx);

    return HI_SUCCESS;
}

HI_VOID FrameWDRCtxExit(VI_PIPE ViPipe)
{
    ISP_FS_WDR_S *pastFSWDRCtx = HI_NULL;

    FS_WDR_GET_CTX(ViPipe, pastFSWDRCtx);
    ISP_FREE(pastFSWDRCtx);
    FS_WDR_RESET_CTX(ViPipe);
}

static HI_U32 WdrSqrt(HI_U32 Val, HI_U32 u32DstBitDep)
{
    HI_U32 X;                                                       // u10.0
    HI_U32 Y;                                                       // u20.0
    HI_S8 j;

    X = (1 << u32DstBitDep) - 1;
    Y = X * X;

    Val = Val << 2;

    for (j = u32DstBitDep; j >= 0; j--) {
        if (Val > Y) {
            Y = Y + ((HI_U64)1 << (j + 1)) * X + ((HI_U64)1 << (2 * j));
            X = X + ((HI_U64)1 << j);                               // u10.0
        } else {
            Y = Y - ((HI_U64)1 << (j + 1)) * X + ((HI_U64)1 << (2 * j));
            X = X - ((HI_U64)1 << j);                               // u10.0
        }
    }

    if (Val > Y) {
        X = X + 1;
    } else if (Val < Y) {
        X = X - 1;
    }

    X = X >> 1;

    return X;
}


static HI_VOID FrameWDRExtRegsInitialize(VI_PIPE ViPipe)
{
    HI_U8 i, j;
    ISP_FS_WDR_S  *pstFSWDR = HI_NULL;

    FS_WDR_GET_CTX(ViPipe, pstFSWDR);

    hi_ext_system_wdr_en_write(ViPipe, pstFSWDR->bWDREn);
    hi_ext_system_wdr_coef_update_en_write(ViPipe, HI_TRUE);
    hi_ext_system_erosion_en_write(ViPipe, HI_EXT_SYSTEM_EROSION_EN_DEFAULT);
    hi_ext_system_mdt_en_write(ViPipe, pstFSWDR->bMdtEn);
    hi_ext_system_wdr_shortexpo_chk_write(ViPipe, pstFSWDR->bShortExpoChk);
    hi_ext_system_wdr_mdref_flicker_write(ViPipe, pstFSWDR->bMdRefFlicker);
    hi_ext_system_bnr_mode_write(ViPipe, pstFSWDR->enBnrMode);
    hi_ext_system_fusion_mode_write(ViPipe, pstFSWDR->bFusionMode);
    hi_ext_system_wdr_bnr_full_mdt_thr_write(ViPipe, HI_EXT_SYSTEM_WDR_BNR_FULL_MDT_THR_DEFAULT);
    hi_ext_system_wdr_g_sigma_gain1_write(ViPipe, HI_EXT_SYSTEM_WDR_G_SIGMA_GAIN1_DEFAULT);
    hi_ext_system_wdr_g_sigma_gain2_write(ViPipe, HI_EXT_SYSTEM_WDR_G_SIGMA_GAIN2_DEFAULT);
    hi_ext_system_wdr_g_sigma_gain3_write(ViPipe, HI_EXT_SYSTEM_WDR_G_SIGMA_GAIN3_DEFAULT);
    hi_ext_system_wdr_c_sigma_gain1_write(ViPipe, HI_EXT_SYSTEM_WDR_C_SIGMA_GAIN1_DEFAULT);
    hi_ext_system_wdr_c_sigma_gain2_write(ViPipe, HI_EXT_SYSTEM_WDR_C_SIGMA_GAIN2_DEFAULT);
    hi_ext_system_wdr_c_sigma_gain3_write(ViPipe, HI_EXT_SYSTEM_WDR_C_SIGMA_GAIN3_DEFAULT);
    hi_ext_system_wdr_full_mot_sigma_weight_write(ViPipe, HI_EXT_SYSTEM_WDR_FULL_MOT_SIGMA_WEIGHT_DEFAULT);
    hi_ext_system_wdr_mdt_full_thr_write(ViPipe, HI_EXT_SYSTEM_WDR_MDT_FULL_THR_DEFAULT);
    hi_ext_system_wdr_mdt_long_blend_write(ViPipe, pstFSWDR->u8MdtLBld);
    hi_ext_system_wdr_mdt_still_thr_write(ViPipe, pstFSWDR->u8MdtStillThr);
    hi_ext_system_wdr_longthr_write(ViPipe, pstFSWDR->u16LongThr);
    hi_ext_system_wdr_shortthr_write(ViPipe, pstFSWDR->u16ShortThr);
    hi_ext_system_wdr_noise_c_weight_mode_write(ViPipe, HI_EXT_SYSTEM_WDR_NOISE_C_WEIGHT_MODE_DEFAULT);
    hi_ext_system_wdr_noise_g_weight_mode_write(ViPipe, HI_EXT_SYSTEM_WDR_NOISE_G_WEIGHT_MODE_DEFAULT);
    hi_ext_system_wdr_noise_model_coef_write(ViPipe, pstFSWDR->u8NoiseModelCoef);
    hi_ext_system_wdr_noise_ratio_rg_write(ViPipe, HI_EXT_SYSTEM_WDR_NOISE_RATIO_RG_DEFAULT);
    hi_ext_system_wdr_noise_ratio_bg_write(ViPipe, HI_EXT_SYSTEM_WDR_NOISE_RATIO_BG_DEFAULT);

    hi_ext_system_wdr_manual_mode_write(ViPipe, OP_TYPE_AUTO);
    hi_ext_system_wdr_manual_mdthr_low_gain_write(ViPipe, HI_ISP_WDR_MDTHR_LOW_GAIN_DEFAULT);
    hi_ext_system_wdr_manual_mdthr_hig_gain_write(ViPipe, HI_ISP_WDR_MDTHR_HIG_GAIN_DEFAULT);

    for (j = 0; j < ISP_AUTO_ISO_STRENGTH_NUM; j++) {
        hi_ext_system_wdr_auto_mdthr_low_gain_write(ViPipe, j, pstFSWDR->au8MdThrLowGain[j]);
        hi_ext_system_wdr_auto_mdthr_hig_gain_write(ViPipe, j, pstFSWDR->au8MdThrHigGain[j]);
    }

    hi_ext_system_fusion_thr_write(ViPipe, 0, pstFSWDR->u16FusionBarrier0);
    hi_ext_system_fusion_thr_write(ViPipe, 1, pstFSWDR->u16FusionBarrier1);

    for (i = 0; i < NoiseSet_EleNum; i++) {
        hi_ext_system_wdr_floorset_write(ViPipe, i, g_as32NoiseFloorSet[i]);
    }

    hi_ext_system_wdr_sfnr_en_write(ViPipe, HI_EXT_SYSTEM_WDR_WDR_SFNR_EN_WGT);
    hi_ext_system_wdr_forcelong_en_write(ViPipe, pstFSWDR->bForceLong);
    hi_ext_system_wdr_forcelong_low_thd_write(ViPipe, pstFSWDR->u16ForceLongLowThr);
    hi_ext_system_wdr_forcelong_high_thd_write(ViPipe, pstFSWDR->u16ForceLongHigThr);
    hi_ext_system_wdr_shortcheck_thd_write(ViPipe, pstFSWDR->u16ShortCheckThd);

    hi_ext_system_wdr_shortsigmal1_cwgt_write(ViPipe, HI_EXT_SYSTEM_WDR_SHORTSIGMAL1_CWGT_WGT);
    hi_ext_system_wdr_shortsigmal2_cwgt_write(ViPipe, HI_EXT_SYSTEM_WDR_SHORTSIGMAL2_CWGT_WGT);
    hi_ext_system_wdr_shortsigmal1_gwgt_write(ViPipe, HI_EXT_SYSTEM_WDR_SHORTSIGMAL1_GWGT_WGT);
    hi_ext_system_wdr_shortsigmal2_gwgt_write(ViPipe, HI_EXT_SYSTEM_WDR_SHORTSIGMAL2_GWGT_WGT);

    hi_ext_system_wdr_mot2sig_cwgt_high_write(ViPipe, HI_EXT_SYSTEM_WDR_MOT2SIG_CWGT_WGT);
    hi_ext_system_wdr_mot2sig_gwgt_high_write(ViPipe, HI_EXT_SYSTEM_WDR_MOT2SIG_GWGT_WGT);

    hi_ext_system_wdr_fusionsigma_cwgt0_write(ViPipe, HI_EXT_SYSTEM_WDR_FUSIONSIGMA_CWGT0_WGT);
    hi_ext_system_wdr_fusionsigma_cwgt1_write(ViPipe, HI_EXT_SYSTEM_WDR_FUSIONSIGMA_CWGT1_WGT);

    hi_ext_system_wdr_fusionsigma_gwgt0_write(ViPipe, HI_EXT_SYSTEM_WDR_FUSIONSIGMA_GWGT0_WGT);
    hi_ext_system_wdr_fusionsigma_gwgt1_write(ViPipe, HI_EXT_SYSTEM_WDR_FUSIONSIGMA_GWGT1_WGT);

    hi_ext_system_wdr_shortframe_nrstr_write(ViPipe, HI_EXT_SYSTEM_WDR_SHORTFRAME_NR_STR_WGT);
    hi_ext_system_wdr_motionbnrstr_write(ViPipe, HI_EXT_SYSTEM_WDR_MOTION_BNR_STR_WGT);
    hi_ext_system_wdr_fusionbnrstr_write(ViPipe, HI_EXT_SYSTEM_WDR_FUSION_BNR_STR_WGT);
}

static HI_VOID FrameWDRStaticRegsInitialize(VI_PIPE ViPipe, HI_U8 u8WDRMode, ISP_FSWDR_STATIC_CFG_S *pstStaticRegCfg, isp_usr_ctx *pstIspCtx)
{
    HI_U32 SaturateLow, SaturateHig;
    HI_S32 s32BlcValue = 0;
    HI_S32 m_MaxValue_In  = ISP_BITMASK(HI_WDR_BITDEPTH);
    HI_U32 au32ExpoValue[EXP_RATIO_NUM] = { 0 };
    hi_isp_cmos_black_level  *sns_black_level = HI_NULL;
    hi_isp_cmos_default    *sns_dft          = HI_NULL;
    ISP_FS_WDR_S           *pstFSWDR         = HI_NULL;

    isp_sensor_get_default(ViPipe, &sns_dft);
    isp_sensor_get_blc(ViPipe, &sns_black_level);
    FS_WDR_GET_CTX(ViPipe, pstFSWDR);

    s32BlcValue = (HI_S32)(sns_black_level->black_level[0]);

    if (IS_2to1_WDR_MODE(u8WDRMode)) {
        au32ExpoValue[0] = CLIP3(sns_dft->wdr_switch_attr.exp_ratio[0], 0x40, ISP_BITMASK(14));
        au32ExpoValue[1] = 0x40;
        pstStaticRegCfg->au16ExpoValue[0] = au32ExpoValue[0];
        pstStaticRegCfg->au16ExpoValue[1] = au32ExpoValue[1];

        pstStaticRegCfg->au32BlcComp[0]    = (au32ExpoValue[0] - au32ExpoValue[1]) * s32BlcValue;
        pstStaticRegCfg->au16ExpoRRatio[0] = MIN2(ISP_BITMASK(10), (ISP_BITMASK(10) * 64 / DIV_0_TO_1(au32ExpoValue[0])));
    } else {
        pstStaticRegCfg->au16ExpoValue[0]  = 0;
        pstStaticRegCfg->au16ExpoValue[1]  = 0;
        pstStaticRegCfg->au32BlcComp[0]    = 0;
        pstStaticRegCfg->au16ExpoRRatio[0] = 0;
    }

    if (pstFSWDR->bFusionMode == HI_FALSE) {
        pstStaticRegCfg->u32MaxRatio = ((1 << 22) - 1) / DIV_0_TO_1(pstStaticRegCfg->au16ExpoValue[0]);
    } else {
        pstStaticRegCfg->u32MaxRatio = ((1 << 22) - 1) / DIV_0_TO_1(pstStaticRegCfg->au16ExpoValue[0] + 64);
    }

    pstStaticRegCfg->bSaveBLC         = HI_ISP_WDR_SAVE_BLC_EN_DEFAULT;
    pstStaticRegCfg->bGrayScaleMode   = HI_ISP_WDR_GRAYSCALE_DEFAULT;
    pstStaticRegCfg->u8MaskSimilarThr = HI_ISP_WDR_MASK_SIMILAR_THR_DEFAULT;
    pstStaticRegCfg->u8MaskSimilarCnt = HI_ISP_WDR_MASK_SIMILAR_CNT_DEFAULT;
    pstStaticRegCfg->u16dftWgtFL      = HI_ISP_WDR_DFTWGT_FL_DEFAULT;
    pstStaticRegCfg->u8bldrLHFIdx     = HI_ISP_WDR_BLDRLHFIDX_DEFAULT;

    pstStaticRegCfg->u16SaturateThr   = HI_ISP_WDR_SATURATE_THR_DEFAULT;

    SaturateHig = ((HI_U32)(m_MaxValue_In - s32BlcValue));
    SaturateLow = WdrSqrt(SaturateHig, 8);
    pstStaticRegCfg->u16SaturateThr       = (HI_U16)(SaturateHig - SaturateLow);
    pstStaticRegCfg->u16FusionSaturateThd = pstStaticRegCfg->u16SaturateThr;
    pstStaticRegCfg->bForceLongSmoothEn   = HI_ISP_WDR_FORCELONG_SMOOTH_EN;
    pstStaticRegCfg->bResh                = HI_TRUE;
    pstStaticRegCfg->bFirstFrame          = HI_TRUE;
}

static HI_VOID FrameWDRSUsrRegsInitialize(ISP_FSWDR_USR_CFG_S *pstUsrRegCfg, ISP_FS_WDR_S *pstFSWDR)
{
    pstUsrRegCfg->bFusionMode        = pstFSWDR->bFusionMode;
    pstUsrRegCfg->bShortExpoChk      = pstFSWDR->bShortExpoChk;
    pstUsrRegCfg->u8MdtLBld          = pstFSWDR->u8MdtLBld;
    pstUsrRegCfg->u8MdtStillThr      = pstFSWDR->u8MdtStillThr;
    pstUsrRegCfg->u8MdtFullThr       = pstFSWDR->u8MdtFullThr;
    pstUsrRegCfg->u16PixelAvgMaxDiff = HI_ISP_WDR_PIXEL_AVG_MAX_DIFF_DEFAULT;
    pstUsrRegCfg->bResh              = HI_TRUE;
    pstUsrRegCfg->u32UpdateIndex     = 1;
}

static HI_VOID FrameWDRSyncRegsInitialize(ISP_FSWDR_SYNC_CFG_S *pstSyncRegCfg, ISP_FS_WDR_S *pstFSWDR)
{
    pstSyncRegCfg->bFusionMode       = pstFSWDR->bFusionMode;
    pstSyncRegCfg->bWDRMdtEn         = pstFSWDR->bMdtEn;
    pstSyncRegCfg->u16ShortThr       = pstFSWDR->u16ShortThr;
    pstSyncRegCfg->u16LongThr        = pstFSWDR->u16LongThr;
}

static HI_VOID FrameWDRSDynaRegsInitialize(VI_PIPE ViPipe, HI_U8 u8WDRMode,
                                           ISP_FSWDR_DYNA_CFG_S *pstDynaRegCfg, ISP_FS_WDR_S *pstFSWDR)
{
    hi_isp_cmos_default *sns_dft = HI_NULL;
    isp_sensor_get_default(ViPipe, &sns_dft);

    if (IS_LINEAR_MODE(u8WDRMode)) {
        pstDynaRegCfg->bBcomEn      =   HI_FALSE;
        pstDynaRegCfg->bBdecEn      =   HI_FALSE;
        pstDynaRegCfg->u8FrmMerge   =   1;
        pstDynaRegCfg->u8bcom_alpha =   0;
        pstDynaRegCfg->u8bdec_alpha =   0;
    } else if (IS_BUILT_IN_WDR_MODE(u8WDRMode)) {
        pstDynaRegCfg->bBcomEn      =   HI_TRUE;
        pstDynaRegCfg->bBdecEn      =   HI_TRUE;
        pstDynaRegCfg->u8FrmMerge   =   1;
        pstDynaRegCfg->u8bcom_alpha =   2;
        pstDynaRegCfg->u8bdec_alpha =   4;
    } else if (IS_2to1_WDR_MODE(u8WDRMode)) {
        pstDynaRegCfg->bBcomEn      =   HI_TRUE;
        pstDynaRegCfg->bBdecEn      =   HI_TRUE;
        pstDynaRegCfg->u8FrmMerge   =   2;
        pstDynaRegCfg->u8bcom_alpha =   2;
        pstDynaRegCfg->u8bdec_alpha =   4;
    } else {
        pstDynaRegCfg->bBcomEn      =   HI_FALSE;
        pstDynaRegCfg->bBdecEn      =   HI_FALSE;
        pstDynaRegCfg->u8FrmMerge   =   1;
        pstDynaRegCfg->u8bcom_alpha =   0;
        pstDynaRegCfg->u8bdec_alpha =   0;
    }

    pstDynaRegCfg->bWDRMdtEn         = pstFSWDR->bMdtEn;
    pstDynaRegCfg->u8SqrtAgainG      = HI_ISP_WDR_SQRT_AGAIN_G_DEFAULT;
    pstDynaRegCfg->u8SqrtDgainG      = HI_ISP_WDR_SQRT_DGAIN_G_DEFAULT;
    pstDynaRegCfg->u8MdtNosFloor     = HI_ISP_WDR_MDT_NOS_FLOOR_DEFAULT;
    pstDynaRegCfg->u16ShortThr       = pstFSWDR->u16ShortThr;
    pstDynaRegCfg->u16LongThr        = pstFSWDR->u16LongThr;
    pstDynaRegCfg->au16StillThr[0]   = HI_ISP_WDR_STILL_THR0_DEFAULT;

    pstDynaRegCfg->u8MdThrLowGain    = HI_ISP_WDR_MDTHR_LOW_GAIN_DEFAULT;
    pstDynaRegCfg->u8MdThrHigGain    = HI_ISP_WDR_MDTHR_HIG_GAIN_DEFAULT;
    pstDynaRegCfg->bErosionEn        = pstFSWDR->bErosionEn;

    pstDynaRegCfg->au16FusionThrR[0]  = HI_ISP_WDR_FUSION_F0_THR_R;
    pstDynaRegCfg->au16FusionThrR[1]  = HI_ISP_WDR_FUSION_F1_THR_R;

    pstDynaRegCfg->au16FusionThrG[0]  = HI_ISP_WDR_FUSION_F0_THR_G;
    pstDynaRegCfg->au16FusionThrG[1]  = HI_ISP_WDR_FUSION_F1_THR_G;

    pstDynaRegCfg->au16FusionThrB[0]  = HI_ISP_WDR_FUSION_F0_THR_B;
    pstDynaRegCfg->au16FusionThrB[1]  = HI_ISP_WDR_FUSION_F1_THR_B;

    pstDynaRegCfg->bForceLong         = pstFSWDR->bForceLong;
    pstDynaRegCfg->u16ForceLongLowThr = pstFSWDR->u16ForceLongLowThr;
    pstDynaRegCfg->u16ForceLongHigThr = pstFSWDR->u16ForceLongHigThr;

    pstDynaRegCfg->u16ShortCheckThd   = pstFSWDR->u16ShortCheckThd;

    if (sns_dft->wdr_switch_attr.exp_ratio[0] < 576) {
        pstDynaRegCfg->bFusionDataMode = 0x1;
    } else {
        pstDynaRegCfg->bFusionDataMode = 0x0;
    }

    pstDynaRegCfg->bResh          = HI_TRUE;
}

static HI_VOID FrameWDRRegsInitialize(VI_PIPE ViPipe, isp_reg_cfg *pstRegCfg)
{
    HI_U8 u8WDRMode, i, u8BlockNum;
    isp_usr_ctx  *pstIspCtx;

    ISP_FS_WDR_S    *pstFSWDR   = HI_NULL;

    FS_WDR_GET_CTX(ViPipe, pstFSWDR);
    ISP_GET_CTX(ViPipe, pstIspCtx);

    u8WDRMode   = pstIspCtx->sns_wdr_mode;
    u8BlockNum  = pstIspCtx->block_attr.block_num;

    for (i = 0; i < u8BlockNum; i++) {
        FrameWDRStaticRegsInitialize(ViPipe, u8WDRMode, &pstRegCfg->alg_reg_cfg[i].stWdrRegCfg.stStaticRegCfg, pstIspCtx);
        FrameWDRSUsrRegsInitialize(&pstRegCfg->alg_reg_cfg[i].stWdrRegCfg.stUsrRegCfg, pstFSWDR);
        FrameWDRSDynaRegsInitialize(ViPipe, u8WDRMode, &pstRegCfg->alg_reg_cfg[i].stWdrRegCfg.stDynaRegCfg, pstFSWDR);
        FrameWDRSyncRegsInitialize(&pstRegCfg->alg_reg_cfg[i].stWdrRegCfg.stSyncRegCfg, pstFSWDR);
        pstRegCfg->alg_reg_cfg[i].stWdrRegCfg.bWDREn = pstFSWDR->bWDREn;
    }

    pstRegCfg->cfg_key.bit1FsWdrCfg = 1;
}

static HI_S32 FrameWDRReadExtRegs(VI_PIPE ViPipe)
{
    HI_U8 i;
    ISP_FS_WDR_S *pstFSWDRCtx;

    FS_WDR_GET_CTX(ViPipe, pstFSWDRCtx);

    pstFSWDRCtx->bCoefUpdateEn  =   hi_ext_system_wdr_coef_update_en_read(ViPipe);
    hi_ext_system_wdr_coef_update_en_write(ViPipe, HI_FALSE);

    if (pstFSWDRCtx->bCoefUpdateEn) {
        pstFSWDRCtx->bFusionMode        = hi_ext_system_fusion_mode_read(ViPipe);
        pstFSWDRCtx->bMdtEn             = hi_ext_system_mdt_en_read(ViPipe);
        pstFSWDRCtx->bWDREn             = hi_ext_system_wdr_en_read(ViPipe);
        pstFSWDRCtx->bShortExpoChk      = hi_ext_system_wdr_shortexpo_chk_read(ViPipe);
        pstFSWDRCtx->bMdRefFlicker      = hi_ext_system_wdr_mdref_flicker_read(ViPipe);
        pstFSWDRCtx->enBnrMode          = hi_ext_system_bnr_mode_read(ViPipe);
        pstFSWDRCtx->u16LongThr         = hi_ext_system_wdr_longthr_read(ViPipe);
        pstFSWDRCtx->u16ShortThr        = hi_ext_system_wdr_shortthr_read(ViPipe);
        pstFSWDRCtx->u8NoiseModelCoef   = hi_ext_system_wdr_noise_model_coef_read(ViPipe);
        pstFSWDRCtx->u16FusionBarrier0  = hi_ext_system_fusion_thr_read(ViPipe, 0);
        pstFSWDRCtx->u16FusionBarrier1  = hi_ext_system_fusion_thr_read(ViPipe, 1);
        pstFSWDRCtx->u16FusionBarrier2  = hi_ext_system_fusion_thr_read(ViPipe, 2);
        pstFSWDRCtx->u16FusionBarrier3  = hi_ext_system_fusion_thr_read(ViPipe, 3);
        pstFSWDRCtx->bManualMode        = hi_ext_system_wdr_manual_mode_read(ViPipe);
        pstFSWDRCtx->u8MdThrLowGain     = hi_ext_system_wdr_manual_mdthr_low_gain_read(ViPipe);
        pstFSWDRCtx->u8MdThrHigGain     = hi_ext_system_wdr_manual_mdthr_hig_gain_read(ViPipe);
        pstFSWDRCtx->u8NoiseModelCoef   = hi_ext_system_wdr_noise_model_coef_read(ViPipe);
        pstFSWDRCtx->u8TextureThdWgt    = hi_ext_system_wdr_texture_thd_wgt_read(ViPipe);

        for (i = 0; i < ISP_AUTO_ISO_STRENGTH_NUM; i++) {
            pstFSWDRCtx->au8MdThrLowGain[i] = hi_ext_system_wdr_auto_mdthr_low_gain_read(ViPipe, i);
            pstFSWDRCtx->au8MdThrHigGain[i] = hi_ext_system_wdr_auto_mdthr_hig_gain_read(ViPipe, i);
        }

        pstFSWDRCtx->bForceLong         = hi_ext_system_wdr_forcelong_en_read(ViPipe);
        pstFSWDRCtx->u16ForceLongHigThr = hi_ext_system_wdr_forcelong_high_thd_read(ViPipe);
        pstFSWDRCtx->u16ForceLongLowThr = hi_ext_system_wdr_forcelong_low_thd_read(ViPipe);
        pstFSWDRCtx->u16ShortCheckThd   = hi_ext_system_wdr_shortcheck_thd_read(ViPipe);

        pstFSWDRCtx->u8MdtStillThr      = hi_ext_system_wdr_mdt_still_thr_read(ViPipe);
        pstFSWDRCtx->u8MdtLBld          = hi_ext_system_wdr_mdt_long_blend_read(ViPipe);
    }

    return HI_SUCCESS;
}

static HI_S32 WdrCheckCmosParam(VI_PIPE ViPipe, const hi_isp_cmos_wdr *cmos_wdr)
{
    HI_U8 j;

    ISP_CHECK_BOOL(cmos_wdr->fusion_mode);
    ISP_CHECK_BOOL(cmos_wdr->motion_comp);
    ISP_CHECK_BOOL(cmos_wdr->short_expo_chk);
    ISP_CHECK_BOOL(cmos_wdr->md_ref_flicker);

    if (cmos_wdr->short_thr > 0xFFF) {
        ISP_ERR_TRACE("Invalid u16ShortThr!\n");
        return HI_ERR_ISP_ILLEGAL_PARAM;
    }
    if (cmos_wdr->long_thr > 0xFFF) {
        ISP_ERR_TRACE("Invalid u16LongThr!\n");
        return HI_ERR_ISP_ILLEGAL_PARAM;
    }
    if (cmos_wdr->long_thr > cmos_wdr->short_thr) {
        ISP_ERR_TRACE("u16LongThresh should NOT be larger than u16ShortThresh!\n");
        return HI_ERR_ISP_ILLEGAL_PARAM;
    }

    for (j = 0; j < ISP_AUTO_ISO_STRENGTH_NUM; j++) {
        if (cmos_wdr->md_thr_low_gain[j] > cmos_wdr->md_thr_hig_gain[j]) {
            ISP_ERR_TRACE("au8MdThrLowGain[%d] should NOT be larger than au8MdThrHigGain[%d]\n", j, j);
            return HI_ERR_ISP_ILLEGAL_PARAM;
        }
    }

    if (cmos_wdr->bnr_mode >= BNR_BUTT) {
        ISP_ERR_TRACE("Invalid enBnrMode!\n");
        return HI_ERR_ISP_ILLEGAL_PARAM;
    }

    for (j = 0; j < WDR_MAX_FRAME; j++) {
        if (cmos_wdr->fusion_thr[j] > 0x3FFF) {
            ISP_ERR_TRACE("Invalid au16FusionThr!\n");
            return HI_ERR_ISP_ILLEGAL_PARAM;
        }
    }

    return HI_SUCCESS;
}

static HI_S32 FrameWDRInitialize(VI_PIPE ViPipe, hi_isp_cmos_default *sns_dft)
{
    HI_U8  i;
    HI_U8  u8WDRMode;
    HI_S32 s32Ret;
    isp_usr_ctx    *pstIspCtx = HI_NULL;
    ISP_FS_WDR_S *pstFSWDR  = HI_NULL;
    FS_WDR_GET_CTX(ViPipe, pstFSWDR);
    ISP_GET_CTX(ViPipe, pstIspCtx);

    u8WDRMode   = pstIspCtx->sns_wdr_mode;

    if (IS_LINEAR_MODE(u8WDRMode) || IS_BUILT_IN_WDR_MODE(u8WDRMode)) {
        pstFSWDR->bWDREn = HI_FALSE;
    } else {
        pstFSWDR->bWDREn = HI_TRUE;
    }

    pstFSWDR->u8BitDepthPrc     = HI_WDR_BITDEPTH;
    pstFSWDR->u32PreIso129      = 0;
    pstFSWDR->u32PreAgain       = 0;
    pstFSWDR->bManualMode       = OP_TYPE_AUTO;
    pstFSWDR->u8MdThrLowGain    = HI_ISP_WDR_MDTHR_LOW_GAIN_DEFAULT;
    pstFSWDR->u8MdThrHigGain    = HI_ISP_WDR_MDTHR_HIG_GAIN_DEFAULT;

    for (i = 0; i < NoiseSet_EleNum; i++) {
        pstFSWDR->au8FloorSet[i]    =   g_as32NoiseFloorSet[i];
        pstFSWDR->au32AgainSet[i]   =   g_as32NoiseAgainSet[i];
    }

    pstFSWDR->bErosionEn       = HI_EXT_SYSTEM_EROSION_EN_DEFAULT;
    pstFSWDR->u8MdtFullThr     = HI_EXT_SYSTEM_WDR_MDT_FULL_THR_DEFAULT;
    pstFSWDR->u8NosCWgtMod     = HI_EXT_SYSTEM_WDR_NOISE_C_WEIGHT_MODE_DEFAULT;
    pstFSWDR->u8NosGWgtMod     = HI_EXT_SYSTEM_WDR_NOISE_G_WEIGHT_MODE_DEFAULT;
    pstFSWDR->u8NoiseRatioRg   = HI_EXT_SYSTEM_WDR_NOISE_RATIO_RG_DEFAULT;
    pstFSWDR->u8NoiseRatioBg   = HI_EXT_SYSTEM_WDR_NOISE_RATIO_BG_DEFAULT;
    pstFSWDR->u8NoiseModelCoef = HI_EXT_SYSTEM_WDR_NOISE_MODEL_COEF_DEFAULT;

    if (sns_dft->key.bit1_wdr) {
        ISP_CHECK_POINTER(sns_dft->wdr);

        s32Ret = WdrCheckCmosParam(ViPipe, sns_dft->wdr);
        if (s32Ret != HI_SUCCESS) {
            return s32Ret;
        }

        pstFSWDR->bFusionMode   = sns_dft->wdr->fusion_mode;
        pstFSWDR->bMdtEn        = sns_dft->wdr->motion_comp;
        pstFSWDR->u16ShortThr   = sns_dft->wdr->short_thr;
        pstFSWDR->u16LongThr    = sns_dft->wdr->long_thr;
        pstFSWDR->bShortExpoChk = sns_dft->wdr->short_expo_chk;
        pstFSWDR->bMdRefFlicker = sns_dft->wdr->md_ref_flicker;

        for (i = 0; i < ISP_AUTO_ISO_STRENGTH_NUM; i++) {
            pstFSWDR->au8MdThrLowGain[i] = sns_dft->wdr->md_thr_low_gain[i];
            pstFSWDR->au8MdThrHigGain[i] = sns_dft->wdr->md_thr_hig_gain[i];
        }

        pstFSWDR->enBnrMode         = sns_dft->wdr->bnr_mode;
        pstFSWDR->u16FusionBarrier0 = sns_dft->wdr->fusion_thr[0];
        pstFSWDR->u16FusionBarrier1 = sns_dft->wdr->fusion_thr[1];

        pstFSWDR->u8MdtStillThr     = sns_dft->wdr->mdt_still_thd;
        pstFSWDR->u8MdtLBld         = sns_dft->wdr->mdt_long_blend;
        pstFSWDR->u16ShortCheckThd  = sns_dft->wdr->short_check_thd;

        pstFSWDR->bForceLong         = sns_dft->wdr->force_long;
        pstFSWDR->u16ForceLongLowThr = sns_dft->wdr->force_long_low_thr;
        pstFSWDR->u16ForceLongHigThr = sns_dft->wdr->force_long_hig_thr;
    } else {
        pstFSWDR->bFusionMode        = HI_EXT_SYSTEM_FUSION_MODE_DEFAULT;
        pstFSWDR->bMdtEn             = HI_EXT_SYSTEM_MDT_EN_DEFAULT;
        pstFSWDR->u16ShortThr        = HI_EXT_SYSTEM_WDR_SHORTTHR_WRITE_DEFAULT;
        pstFSWDR->u16LongThr         = HI_EXT_SYSTEM_WDR_LONGTHR_WRITE_DEFAULT;
        pstFSWDR->bShortExpoChk      = HI_ISP_WDR_SHORT_EXPO_CHK_DEFAULT;
        pstFSWDR->bMdRefFlicker      = HI_EXT_SYSTEM_WDR_MDREF_FLICKER_DEFAULT;
        pstFSWDR->bForceLong         = HI_EXT_SYSTEM_WDR_FORCELONG_EN;
        pstFSWDR->u16ForceLongLowThr = HI_EXT_SYSTEM_WDR_FORCELONG_LOW_THD;
        pstFSWDR->u16ForceLongHigThr = HI_EXT_SYSTEM_WDR_FORCELONG_HIGH_THD;

        for (i = 0; i < ISP_AUTO_ISO_STRENGTH_NUM; i++) {
            pstFSWDR->au8MdThrLowGain[i] = g_au8lutMDTLowThr[i];
            pstFSWDR->au8MdThrHigGain[i] = g_au8lutMDTHigThr[i];
        }

        pstFSWDR->enBnrMode         = HI_EXT_SYSTEM_BNR_MODE_WRITE_DEFAULT;
        pstFSWDR->u16FusionBarrier0 = g_as32FusionThr[0];
        pstFSWDR->u16FusionBarrier1 = g_as32FusionThr[1];
        pstFSWDR->u8MdtLBld         = HI_EXT_SYSTEM_WDR_MDT_LONG_BLEND_DEFAULT;
        pstFSWDR->u8MdtStillThr     = HI_EXT_SYSTEM_WDR_MDT_STILL_THR_DEFAULT;
        pstFSWDR->u16ShortCheckThd  = HI_EXT_SYSTEM_WDR_SHORTCHECK_THD;
    }

    return HI_SUCCESS;
}

HI_S32 ISP_FrameWDRInit(VI_PIPE ViPipe, HI_VOID *pRegCfg)
{
    HI_S32 s32Ret = HI_SUCCESS;
    isp_reg_cfg      *pstRegCfg = (isp_reg_cfg *)pRegCfg;
    hi_isp_cmos_default *sns_dft = HI_NULL;

    s32Ret = FrameWDRCtxInit(ViPipe);
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    isp_sensor_get_default(ViPipe, &sns_dft);

    s32Ret = FrameWDRInitialize(ViPipe, sns_dft);
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }
    FrameWDRExtRegsInitialize(ViPipe);
    FrameWDRRegsInitialize(ViPipe, pstRegCfg);

    return HI_SUCCESS;
}

static HI_VOID ISP_FrameWDRSwitchMode(VI_PIPE ViPipe, HI_VOID *pRegCfg)
{
    HI_U8 i;
    isp_reg_cfg *pstRegCfg = (isp_reg_cfg *)pRegCfg;

    ISP_FrameWDRInit(ViPipe, pRegCfg);

    for (i = 0; i < pstRegCfg->cfg_num; i++) {
        pstRegCfg->alg_reg_cfg[i].stWdrRegCfg.stStaticRegCfg.bFirstFrame = HI_FALSE;
    }
}

static HI_BOOL __inline CheckWdrOpen(ISP_FS_WDR_S *pstFsWdr)
{
    return (pstFsWdr->bWDREn == HI_TRUE);
}

HI_S32 CheckWDRMode(VI_PIPE ViPipe, ISP_FS_WDR_S *pstFsWdr)
{
    HI_U8  u8WDRMode;
    isp_usr_ctx     *pstIspCtx;

    ISP_GET_CTX(ViPipe, pstIspCtx);

    u8WDRMode   = pstIspCtx->sns_wdr_mode;

    if (IS_LINEAR_MODE(u8WDRMode) || IS_BUILT_IN_WDR_MODE(u8WDRMode)) {
        pstFsWdr->bWDREn = HI_FALSE;
    }

    return HI_SUCCESS;
}

static HI_U8 GetValueFromLut(HI_S32 x, HI_U32 const *pLutX, HI_U8 *pLutY, HI_S32 length)
{
    HI_S32 n = 0;

    if (x <= pLutX[0]) {
        return pLutY[0];
    }

    for (n = 1; n < length; n++) {
        if (x <= pLutX[n]) {
            return (HI_U8)(pLutY[n - 1] + (pLutY[n] - pLutY[n - 1]) * (x - (HI_S32)pLutX[n - 1]) / DIV_0_TO_1((HI_S32)pLutX[n] - (HI_S32)pLutX[n - 1]));
        }
    }

    return pLutY[length - 1];
}

HI_S32 wdr_FwBlend(HI_S64 v, HI_S64 x0, HI_S64 x1, HI_S64 y0, HI_S64 y1)
{
    HI_S32 y;

    if (v <= x0) {
        return (HI_S32)y0;
    }

    if (v >= x1) {
        return (HI_S32)y1;
    }

    y = y0 + ((((y1 - y0) * (v - x0) << 8) / (x1 - x0) + 127) >> 8);

    return (HI_S32)y;
}

static HI_VOID hiisp_wdr_func(VI_PIPE ViPipe, isp_usr_ctx *pstIspCtx, ISP_FS_WDR_S *pstFsWdr, ISP_FSWDR_DYNA_CFG_S *pstWDRReg, ISP_FSWDR_STATIC_CFG_S *stStaticRegCfg)
{
    HI_S32 s32BlcValue = 0;
    HI_U32 i;

    // noise init
    HI_U32 m_fSensorAgain = ((pstIspCtx->linkage.again << 16) + 512) / 1024;
    HI_U32 m_fSensorDgain = ((pstIspCtx->linkage.dgain << 16) + 512) / 1024 + (1 << 16);

    HI_U32 u32AwbRGain    = pstIspCtx->linkage.white_balance_gain[0] >> 8;
    HI_U32 u32AwbGGain    = pstIspCtx->linkage.white_balance_gain[1] >> 8;
    HI_U32 u32AwbBGain    = pstIspCtx->linkage.white_balance_gain[3] >> 8;

    HI_S32 m_Again_G;
    HI_S32 m_Sqrt_AgainG, m_Sqrt_DgainG;
    HI_U32 StillExpSLow, StillExpSHig;
    HI_S32 m_MaxValue_In  = ISP_BITMASK(pstFsWdr->u8BitDepthPrc);

    HI_U32 m_NoiseFloor = 0;
    HI_U32 fGnoisefloor, fRnoisefloor, fBnoisefloor;
    HI_U32 m_Noise_Ratio_Rg_Wgt = 3; /* 1.5 */
    HI_U32 m_Noise_Ratio_Bg_Wgt = 3; /* 1.5 */

    HI_U16 u16NosFloorG = 0;

    HI_U32 Ratio = pstIspCtx->linkage.exp_ratio;

    hi_isp_cmos_default     *sns_dft        = HI_NULL;
    hi_isp_cmos_black_level  *sns_black_level = HI_NULL;

    isp_sensor_get_default(ViPipe, &sns_dft);
    isp_sensor_get_blc(ViPipe, &sns_black_level);

    Ratio = MIN2(MAX2(Ratio, 64), 16384);

    s32BlcValue = sns_black_level->black_level[0];

    pstWDRReg->bWDRMdtEn        = pstFsWdr->bMdtEn;
    pstWDRReg->u16LongThr       = pstFsWdr->u16LongThr;
    pstWDRReg->u16ShortThr      = pstFsWdr->u16ShortThr;

    pstWDRReg->u8TextureThdWgt = pstFsWdr->u8TextureThdWgt;

    if (Ratio < 576) {
        pstWDRReg->bFusionDataMode = 0x1;
    } else {
        pstWDRReg->bFusionDataMode = 0x0;
    }

    if (u32AwbRGain != 0 && u32AwbBGain != 0 && u32AwbGGain != 0) {
        pstWDRReg->au16FusionThrR[0] = MIN2(ISP_BITMASK(12), (pstFsWdr->u16FusionBarrier0 << 8) / DIV_0_TO_1(u32AwbRGain));
        pstWDRReg->au16FusionThrR[1] = MIN2(ISP_BITMASK(12), (pstFsWdr->u16FusionBarrier1 << 8) / DIV_0_TO_1(u32AwbRGain));

        pstWDRReg->au16FusionThrG[0] = MIN2(ISP_BITMASK(12), (pstFsWdr->u16FusionBarrier0 << 8) / DIV_0_TO_1(u32AwbGGain));
        pstWDRReg->au16FusionThrG[1] = MIN2(ISP_BITMASK(12), (pstFsWdr->u16FusionBarrier1 << 8) / DIV_0_TO_1(u32AwbGGain));

        pstWDRReg->au16FusionThrB[0] = MIN2(ISP_BITMASK(12), (pstFsWdr->u16FusionBarrier0 << 8) / DIV_0_TO_1(u32AwbBGain));
        pstWDRReg->au16FusionThrB[1] = MIN2(ISP_BITMASK(12), (pstFsWdr->u16FusionBarrier1 << 8) / DIV_0_TO_1(u32AwbBGain));
    }

    // noise cal
    /* Fix point */
    m_Again_G = (HI_S32)((m_fSensorAgain * 2) * 32) >> 16;

    m_Sqrt_AgainG = (HI_S32)(WdrSqrt(m_fSensorAgain, HI_WDR_SQRT_GAIN_BITDEP)) >> 8;
    m_Sqrt_DgainG = (HI_S32)(WdrSqrt(m_fSensorDgain, HI_WDR_SQRT_GAIN_BITDEP)) >> 8;

    pstFsWdr->u32PreAgain = pstIspCtx->linkage.again;

    for (i = 0; i < NoiseSet_EleNum; i++) {
        pstFsWdr->au32AgainSet[i] = g_as32NoiseAgainSet[i] * 64;
    }

    /* noise floor interpolation */
    for (i = 0; i < (NoiseSet_EleNum - 1); i++) {
        if (m_Again_G >= pstFsWdr->au32AgainSet[i] && m_Again_G <= pstFsWdr->au32AgainSet[i + 1]) {
            m_NoiseFloor = pstFsWdr->au8FloorSet[i] + ((pstFsWdr->au8FloorSet[i + 1] - pstFsWdr->au8FloorSet[i]) * (m_Again_G - pstFsWdr->au32AgainSet[i])) / DIV_0_TO_1(pstFsWdr->au32AgainSet[i + 1] - pstFsWdr->au32AgainSet[i]);
        }
    }

    fGnoisefloor =  m_NoiseFloor;
    fRnoisefloor = ((((m_NoiseFloor * 90 * m_Noise_Ratio_Rg_Wgt + 1) >> 1) + 32) >> 6) + ((((m_NoiseFloor * m_Noise_Ratio_Rg_Wgt + 1) >> 1) + 64) >> 7);
    fBnoisefloor = ((((m_NoiseFloor * 90 * m_Noise_Ratio_Bg_Wgt + 1) >> 1) + 32) >> 6) + ((((m_NoiseFloor * m_Noise_Ratio_Bg_Wgt + 1) >> 1) + 64) >> 7);

    pstWDRReg->u16TNosFloor  = MIN2(ISP_BITMASK(12), ((fGnoisefloor + fRnoisefloor + fBnoisefloor) * WdrSqrt(m_fSensorDgain, HI_WDR_SQRT_GAIN_BITDEP) + 32) >> 8);

    u16NosFloorG             = MIN2(ISP_BITMASK(9), (HI_S32)(m_NoiseFloor * m_fSensorDgain + (1 << 15)) >> 16);

    pstWDRReg->u8MdtNosFloor = MIN2(ISP_BITMASK(7), u16NosFloorG * WdrSqrt(WdrSqrt(Ratio, 11), 11));

    pstWDRReg->u8SqrtAgainG  = MIN2(6, m_Sqrt_AgainG);
    pstWDRReg->u8SqrtDgainG  = MIN2(6, m_Sqrt_DgainG);

    if (pstFsWdr->bManualMode) {
        pstWDRReg->u8MdThrLowGain = pstFsWdr->u8MdThrLowGain;
        pstWDRReg->u8MdThrHigGain = pstFsWdr->u8MdThrHigGain;
    } else {
        pstWDRReg->u8MdThrLowGain =  GetValueFromLut(pstIspCtx->linkage.iso, g_au32IsoLut, pstFsWdr->au8MdThrLowGain, ISP_AUTO_ISO_STRENGTH_NUM);
        pstWDRReg->u8MdThrHigGain =  GetValueFromLut(pstIspCtx->linkage.iso, g_au32IsoLut, pstFsWdr->au8MdThrHigGain, ISP_AUTO_ISO_STRENGTH_NUM);
    }

    pstWDRReg->bErosionEn = pstFsWdr->bErosionEn;

    if (HI_TRUE == hi_ext_system_flicker_result_read(ViPipe) && (HI_TRUE == pstFsWdr->bMdRefFlicker)) {
        pstWDRReg->u8MdThrLowGain = 45;
        pstWDRReg->u8MdThrHigGain = 45;
    }

    StillExpSHig = ((HI_U32)(m_MaxValue_In - s32BlcValue)) << 6;
    StillExpSLow = ((HI_U32)WdrSqrt((m_MaxValue_In - s32BlcValue), 8));

    for (i = 0; i < (pstWDRReg->u8FrmMerge - 1); i++) {
        pstWDRReg->au16StillThr[i] = WDR_CLIP3(0, ISP_BITMASK(12), ((HI_S32)(StillExpSHig / DIV_0_TO_1(pstIspCtx->linkage.exp_ratio_lut[i])) - (HI_S32)StillExpSLow));
    }

    pstWDRReg->bForceLong         = pstFsWdr->bForceLong;
    pstWDRReg->u16ForceLongLowThr = MIN2(ISP_BITMASK(12), pstFsWdr->u16ForceLongLowThr);
    pstWDRReg->u16ForceLongHigThr = MIN2(ISP_BITMASK(12), pstFsWdr->u16ForceLongHigThr);
    pstWDRReg->u16ShortCheckThd   = MIN2(ISP_BITMASK(12), pstFsWdr->u16ShortCheckThd);

    pstWDRReg->bResh = HI_TRUE;
}

static HI_VOID hiisp_wdr_sync_Fw(ISP_FS_WDR_S *pstFSWDR, ISP_FSWDR_SYNC_CFG_S *pstSyncRegCfg)
{
    pstSyncRegCfg->bFusionMode = pstFSWDR->bFusionMode;
    pstSyncRegCfg->bWDRMdtEn   = pstFSWDR->bMdtEn;
    pstSyncRegCfg->u16ShortThr = pstFSWDR->u16ShortThr;
    pstSyncRegCfg->u16LongThr  = pstFSWDR->u16LongThr;
}

static HI_VOID hiisp_wdr_usr_Fw(ISP_FS_WDR_S *pstFSWDR, ISP_FSWDR_USR_CFG_S *pstUsrRegCfg)
{
    pstUsrRegCfg->bFusionMode     = pstFSWDR->bFusionMode;
    pstUsrRegCfg->bShortExpoChk   = pstFSWDR->bShortExpoChk;
    pstUsrRegCfg->u8MdtFullThr    = pstFSWDR->u8MdtFullThr;
    pstUsrRegCfg->u8MdtLBld       = pstFSWDR->u8MdtLBld;
    pstUsrRegCfg->u8MdtStillThr   = pstFSWDR->u8MdtStillThr;
    pstUsrRegCfg->bResh           = HI_TRUE;
    pstUsrRegCfg->u32UpdateIndex += 1;
}

static HI_VOID  wdr_set_long_frame_mode(VI_PIPE ViPipe, isp_reg_cfg *pstRegCfg)
{
    HI_U8 i;
    isp_usr_ctx *pstIspCtx = HI_NULL;
    ISP_FSWDR_DYNA_CFG_S *pstDynaRegCfg = HI_NULL;
    ISP_GET_CTX(ViPipe, pstIspCtx);

    for (i = 0; i < pstRegCfg->cfg_num; i++) {
        pstDynaRegCfg = &pstRegCfg->alg_reg_cfg[i].stWdrRegCfg.stDynaRegCfg;
        if ((pstIspCtx->linkage.fswdr_mode == ISP_FSWDR_LONG_FRAME_MODE) ||
            (pstIspCtx->linkage.fswdr_mode == ISP_FSWDR_AUTO_LONG_FRAME_MODE)) {
            pstDynaRegCfg->bBcomEn = HI_FALSE;
            pstDynaRegCfg->bBdecEn = HI_FALSE;
        } else {
            pstDynaRegCfg->bBcomEn = HI_TRUE;
            pstDynaRegCfg->bBdecEn = HI_TRUE;
        }
    }
}

HI_S32 ISP_FrameWDRRun(VI_PIPE ViPipe, const HI_VOID *pStatInfo,
                       HI_VOID *pRegCfg, HI_S32 s32Rsv)
{
    HI_U8 i = 0;
    isp_usr_ctx     *pstIspCtx = HI_NULL;
    ISP_FS_WDR_S  *pstFSWDR  = HI_NULL;
    isp_reg_cfg *pstRegCfg = (isp_reg_cfg *)pRegCfg;

    ISP_GET_CTX(ViPipe, pstIspCtx);
    FS_WDR_GET_CTX(ViPipe, pstFSWDR);

    ISP_CHECK_POINTER(pstFSWDR);

    if (pstIspCtx->linkage.defect_pixel) {
        return HI_SUCCESS;
    }

    pstFSWDR->bWDREn  = hi_ext_system_wdr_en_read(ViPipe);

    CheckWDRMode(ViPipe, pstFSWDR);

    for (i = 0; i < pstRegCfg->cfg_num; i++) {
        pstRegCfg->alg_reg_cfg[i].stWdrRegCfg.bWDREn = pstFSWDR->bWDREn;
    }

    pstRegCfg->cfg_key.bit1FsWdrCfg = 1;

    /* check hardware setting */
    if (!CheckWdrOpen(pstFSWDR)) {
        return HI_SUCCESS;
    }

    FrameWDRReadExtRegs(ViPipe);
    if (pstIspCtx->linkage.fswdr_mode != pstIspCtx->linkage.pre_fswdr_mode) {
        wdr_set_long_frame_mode(ViPipe, pstRegCfg);
    }

    if (pstFSWDR->bCoefUpdateEn) {
        for (i = 0; i < pstRegCfg->cfg_num; i++) {
            hiisp_wdr_usr_Fw(pstFSWDR, &pstRegCfg->alg_reg_cfg[i].stWdrRegCfg.stUsrRegCfg);
        }
    }

    for (i = 0; i < pstRegCfg->cfg_num; i++) {
        hiisp_wdr_sync_Fw(pstFSWDR, &pstRegCfg->alg_reg_cfg[i].stWdrRegCfg.stSyncRegCfg);
    }

    hiisp_wdr_func(ViPipe, pstIspCtx, pstFSWDR, &pstRegCfg->alg_reg_cfg[0].stWdrRegCfg.stDynaRegCfg, &pstRegCfg->alg_reg_cfg[0].stWdrRegCfg.stStaticRegCfg);

    for (i = 1; i < pstRegCfg->cfg_num; i++) {
        memcpy(&pstRegCfg->alg_reg_cfg[i].stWdrRegCfg.stDynaRegCfg, &pstRegCfg->alg_reg_cfg[0].stWdrRegCfg.stDynaRegCfg, sizeof(ISP_FSWDR_DYNA_CFG_S));
    }

    return HI_SUCCESS;
}

HI_S32 FrameWDRProcWrite(VI_PIPE ViPipe, hi_isp_ctrl_proc_write *pstProc)
{
    hi_isp_ctrl_proc_write stProcTmp;
    ISP_FS_WDR_S *pstFSWDR = HI_NULL;

    FS_WDR_GET_CTX(ViPipe, pstFSWDR);
    ISP_CHECK_POINTER(pstFSWDR);

    if ((pstProc->proc_buff == HI_NULL) || (pstProc->buff_len == 0)) {
        return HI_FAILURE;
    }

    stProcTmp.proc_buff = pstProc->proc_buff;
    stProcTmp.buff_len = pstProc->buff_len;

    ISP_PROC_PRINTF(&stProcTmp, pstProc->write_len,
                    "-----FrameWDR INFO------------------------------------------------------------------\n");

    ISP_PROC_PRINTF(&stProcTmp, pstProc->write_len,
                    "%16s"      "%16s"    "%16s\n",
                    "MdtEn",  "LongThr", "ShortThr");

    ISP_PROC_PRINTF(&stProcTmp, pstProc->write_len,
                    "%16d"   "%16d"     "%16d\n",
                    pstFSWDR->bMdtEn,
                    pstFSWDR->u16LongThr,
                    pstFSWDR->u16ShortThr);

    pstProc->write_len += 1;

    return HI_SUCCESS;
}

HI_S32 ISP_FrameWDRCtrl(VI_PIPE ViPipe, HI_U32 u32Cmd, HI_VOID *pValue)
{
    isp_reg_cfg_attr *pRegCfg = HI_NULL;

    switch (u32Cmd) {
        case ISP_WDR_MODE_SET:
            ISP_REGCFG_GET_CTX(ViPipe, pRegCfg);
            ISP_CHECK_POINTER(pRegCfg);
            ISP_FrameWDRSwitchMode(ViPipe, (HI_VOID *)&pRegCfg->reg_cfg);
            break;

        case ISP_PROC_WRITE:
            FrameWDRProcWrite(ViPipe, (hi_isp_ctrl_proc_write *)pValue);
            break;

        default:
            break;
    }

    return HI_SUCCESS;
}

HI_S32 ISP_FrameWDRExit(VI_PIPE ViPipe)
{
    HI_U8 i;
    isp_reg_cfg_attr *pRegCfg = HI_NULL;

    ISP_REGCFG_GET_CTX(ViPipe, pRegCfg);

    for (i = 0; i < pRegCfg->reg_cfg.cfg_num; i++) {
        pRegCfg->reg_cfg.alg_reg_cfg[i].stWdrRegCfg.stDynaRegCfg.bBcomEn = HI_FALSE;
        pRegCfg->reg_cfg.alg_reg_cfg[i].stWdrRegCfg.stDynaRegCfg.bBdecEn = HI_FALSE;
    }

    pRegCfg->reg_cfg.cfg_key.bit1FsWdrCfg = 1;

    FrameWDRCtxExit(ViPipe);

    return HI_SUCCESS;
}

HI_S32 isp_alg_register_frame_wdr(VI_PIPE ViPipe)
{
    isp_usr_ctx *pstIspCtx = HI_NULL;
    isp_alg_node *pstAlgs = HI_NULL;

    ISP_GET_CTX(ViPipe, pstIspCtx);
    ISP_ALG_CHECK(pstIspCtx->alg_key.bit1_wdr);
    pstAlgs = ISP_SearchAlg(pstIspCtx->algs);
    ISP_CHECK_POINTER(pstAlgs);

    pstAlgs->alg_type = ISP_ALG_FrameWDR;
    pstAlgs->alg_func.pfn_alg_init = ISP_FrameWDRInit;
    pstAlgs->alg_func.pfn_alg_run  = ISP_FrameWDRRun;
    pstAlgs->alg_func.pfn_alg_ctrl = ISP_FrameWDRCtrl;
    pstAlgs->alg_func.pfn_alg_exit = ISP_FrameWDRExit;
    pstAlgs->used = HI_TRUE;

    return HI_SUCCESS;
}
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
