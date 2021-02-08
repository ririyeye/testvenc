/*
* Copyright (C) Hisilicon Technologies Co., Ltd. 2012-2019. All rights reserved.
* Description:
* Author: Hisilicon multimedia software group
* Create: 2011/06/28
*/

#include <math.h>
#include "isp_alg.h"
#include "isp_sensor.h"
#include "isp_config.h"
#include "isp_ext_config.h"
#include "isp_proc.h"
#include "isp_math_utils.h"


#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */
#define HI_ISP_DE_LUT_LEN     (16)

typedef struct hiISP_DETAIL_S {
    HI_BOOL         bInit;
    HI_BOOL         bEnable;
    HI_BOOL         bBnrEn;
    HI_BOOL         bDeAttrUpdate;

    HI_U16          au16LumaGainLut[HI_ISP_DE_LUMA_GAIN_LUT_N]; // U1.8  [0,256]

    HI_U16          u16ProcGlobalGain;
    HI_U16          u16ProcGainLF;
    HI_U16          u16ProcGainHF;

    ISP_OP_TYPE_E        enOpType;
    ISP_DE_AUTO_ATTR_S   stAuto;
    ISP_DE_MANUAL_ATTR_S stManual;
} ISP_DETAIL_S;

ISP_DETAIL_S *g_astDetailCtx[ISP_MAX_PIPE_NUM] = {HI_NULL};

#define DETAIL_GET_CTX(dev, pstCtx)   (pstCtx = g_astDetailCtx[dev])
#define DETAIL_SET_CTX(dev, pstCtx)   (g_astDetailCtx[dev] = pstCtx)
#define DETAIL_RESET_CTX(dev)         (g_astDetailCtx[dev] = HI_NULL)

static HI_U16  g_au16swLumaGainLut[HI_ISP_DE_LUMA_GAIN_LUT_N] = {256, 192, 180, 170, 162, 154, 146, 138, 132, 128, 128, 128, 128, 128, 128, 128, 128};
static HI_U16  g_au16Sgm4Gain[ISP_AUTO_ISO_STRENGTH_NUM]      = {12,   12,   12,  12,   18,     30,    12,    12,      12,      12,      12,       12,      12,         12,       12,         12};
static HI_U16  g_au16Sgm3Gain[ISP_AUTO_ISO_STRENGTH_NUM]      = {4,    4,     4,    4,     8,      10,    4,      4,       4,        4,       4,         4,        4,          4,         4,          4};
static HI_U16  g_au16Sgm2Gain[ISP_AUTO_ISO_STRENGTH_NUM]      = {2,    2,     2,    2,     4,       5,     2,      2,       2,        2,       2,         2,        2,          2,         2,          2};

HI_S32 DetailCtxInit(VI_PIPE ViPipe)
{
    ISP_DETAIL_S *pstDetail = HI_NULL;

    DETAIL_GET_CTX(ViPipe, pstDetail);

    if (pstDetail == HI_NULL) {
        pstDetail = (ISP_DETAIL_S *)ISP_MALLOC(sizeof(ISP_DETAIL_S));
        if (pstDetail == HI_NULL) {
            ISP_ERR_TRACE("Isp[%d] DetailCtx malloc memory failed!\n", ViPipe);
            return HI_ERR_ISP_NOMEM;
        }
    }

    memset(pstDetail, 0, sizeof(ISP_DETAIL_S));

    DETAIL_SET_CTX(ViPipe, pstDetail);

    return HI_SUCCESS;
}

HI_VOID DetailCtxExit(VI_PIPE ViPipe)
{
    ISP_DETAIL_S *pstDetail = HI_NULL;

    DETAIL_GET_CTX(ViPipe, pstDetail);
    ISP_FREE(pstDetail);
    DETAIL_RESET_CTX(ViPipe);
}

static HI_S32 DetailCheckCmosParam(VI_PIPE ViPipe, const hi_isp_cmos_detail *cmos_detail)
{
    HI_U8 i;

    ISP_CHECK_BOOL(cmos_detail->attr.enable);

    if (cmos_detail->attr.op_type >= OP_TYPE_BUTT) {
        ISP_ERR_TRACE("Invalid enOpType!\n");
        return HI_ERR_ISP_ILLEGAL_PARAM;
    }

    for (i = 0; i < HI_ISP_DE_LUMA_GAIN_LUT_N; i++) {
        if (cmos_detail->attr.luma_gain_lut[i] > 0x100) {
            ISP_ERR_TRACE("Invalid au16LumaGainLut[%d]!\n", i);
            return HI_ERR_ISP_ILLEGAL_PARAM;
        }
    }

    for (i = 0; i < ISP_AUTO_ISO_STRENGTH_NUM; i++) {
        if (cmos_detail->attr.auto_attr.global_gain[i] > 0x100) {
            ISP_ERR_TRACE("Invalid au16GlobalGain[%d]!\n", i);
            return HI_ERR_ISP_ILLEGAL_PARAM;
        }

        if (cmos_detail->attr.auto_attr.gain_lf[i] > 0x20) {
            ISP_ERR_TRACE("Invalid au16GainLF[%d]\n", i);
            return HI_ERR_ISP_ILLEGAL_PARAM;
        }

        if (cmos_detail->attr.auto_attr.gain_hf[i] > 0x20) {
            ISP_ERR_TRACE("Invalid au16GainHF[%d]!\n", i);
            return HI_ERR_ISP_ILLEGAL_PARAM;
        }
    }

    if (cmos_detail->attr.manual_attr.global_gain > 0x100) {
        ISP_ERR_TRACE("Invalid u16GlobalGain!\n");
        return HI_ERR_ISP_ILLEGAL_PARAM;
    }

    if (cmos_detail->attr.manual_attr.gain_lf > 0x20) {
        ISP_ERR_TRACE("Invalid u16GainLF!\n");
        return HI_ERR_ISP_ILLEGAL_PARAM;
    }

    if (cmos_detail->attr.manual_attr.gain_hf > 0x20) {
        ISP_ERR_TRACE("Invalid u16GainHF!\n");
        return HI_ERR_ISP_ILLEGAL_PARAM;
    }

    return HI_SUCCESS;
}

static HI_S32 DetailExtRegsInitialize(VI_PIPE ViPipe)
{
    HI_U8  i;
    HI_S32 s32Ret;
    ISP_DETAIL_S       *pstDetail = HI_NULL;
    hi_isp_cmos_default *sns_dft = HI_NULL;

    DETAIL_GET_CTX(ViPipe, pstDetail);
    isp_sensor_get_default(ViPipe, &sns_dft);

    pstDetail->bBnrEn = hi_ext_system_bayernr_enable_read(ViPipe);

    if (sns_dft->key.bit1_detail) {
        ISP_CHECK_POINTER(sns_dft->detail);

        s32Ret = DetailCheckCmosParam(ViPipe, sns_dft->detail);
        if (s32Ret != HI_SUCCESS) {
            return s32Ret;
        }

        pstDetail->bEnable  = (pstDetail->bBnrEn == HI_FALSE) ? (HI_FALSE) : (sns_dft->detail->attr.enable);
        pstDetail->enOpType = sns_dft->detail->attr.op_type;

        memcpy(pstDetail->au16LumaGainLut,       sns_dft->detail->attr.luma_gain_lut,       HI_ISP_DE_LUMA_GAIN_LUT_N * sizeof(HI_U16));
        memcpy(pstDetail->stAuto.au16GlobalGain, sns_dft->detail->attr.auto_attr.global_gain, ISP_AUTO_ISO_STRENGTH_NUM * sizeof(HI_U16));
        memcpy(pstDetail->stAuto.au16GainLF,     sns_dft->detail->attr.auto_attr.gain_lf,     ISP_AUTO_ISO_STRENGTH_NUM * sizeof(HI_U16));
        memcpy(pstDetail->stAuto.au16GainHF,     sns_dft->detail->attr.auto_attr.gain_hf,     ISP_AUTO_ISO_STRENGTH_NUM * sizeof(HI_U16));

        pstDetail->stManual.u16GlobalGain    =  sns_dft->detail->attr.manual_attr.global_gain;
        pstDetail->stManual.u16GainLF        =  sns_dft->detail->attr.manual_attr.gain_lf;
        pstDetail->stManual.u16GainHF        =  sns_dft->detail->attr.manual_attr.gain_hf;
    } else {
        pstDetail->bEnable  = (pstDetail->bBnrEn == HI_FALSE) ? (HI_FALSE) : (HI_EXT_SYSTEM_DETAIL_ENABLE_DEFAULT);
        pstDetail->enOpType = HI_EXT_SYSTEM_DETAIL_MANU_MODE_DEFAULT;

        memcpy(pstDetail->au16LumaGainLut, g_au16swLumaGainLut, HI_ISP_DE_LUMA_GAIN_LUT_N * sizeof(HI_U16));

        for (i = 0; i < ISP_AUTO_ISO_STRENGTH_NUM; i++) {
            pstDetail->stAuto.au16GlobalGain[i] =  HI_EXT_SYSTEM_DETAIL_MANUAL_GLOBAL_GAIN_DEFAULT;
            pstDetail->stAuto.au16GainLF[i]     =  HI_EXT_SYSTEM_DETAIL_MANUAL_GAIN_LF_DEFAULT;
            pstDetail->stAuto.au16GainHF[i]     =  HI_EXT_SYSTEM_DETAIL_MANUAL_GAIN_HF_DEFAULT;
        }

        // Manual
        pstDetail->stManual.u16GlobalGain =  HI_EXT_SYSTEM_DETAIL_MANUAL_GLOBAL_GAIN_DEFAULT;
        pstDetail->stManual.u16GainLF     =  HI_EXT_SYSTEM_DETAIL_MANUAL_GAIN_LF_DEFAULT;
        pstDetail->stManual.u16GainHF     =  HI_EXT_SYSTEM_DETAIL_MANUAL_GAIN_HF_DEFAULT;
    }

    hi_ext_system_detail_enable_write(ViPipe, pstDetail->bEnable);
    hi_ext_system_detail_manual_mode_write(ViPipe, pstDetail->enOpType);

    for (i = 0; i < HI_ISP_DE_LUMA_GAIN_LUT_N; i++) {
        hi_ext_system_detail_luma_gain_lut_write(ViPipe, i, pstDetail->au16LumaGainLut[i]);
    }

    for (i = 0; i < ISP_AUTO_ISO_STRENGTH_NUM; i++) {
        hi_ext_system_detail_auto_global_gain_write(ViPipe, i, pstDetail->stAuto.au16GlobalGain[i]);
        hi_ext_system_detail_auto_gain_lf_write(ViPipe, i, pstDetail->stAuto.au16GainLF[i]);
        hi_ext_system_detail_auto_gain_hf_write(ViPipe, i, pstDetail->stAuto.au16GainHF[i]);
    }

    hi_ext_system_detail_manual_global_gain_write(ViPipe, pstDetail->stManual.u16GlobalGain);
    hi_ext_system_detail_manual_gain_lf_write(ViPipe, pstDetail->stManual.u16GainLF);
    hi_ext_system_detail_manual_gain_hf_write(ViPipe, pstDetail->stManual.u16GainHF);
    hi_ext_system_detail_attr_update_write(ViPipe, HI_FALSE);

    pstDetail->bInit = HI_TRUE;

    return HI_SUCCESS;
}

static HI_VOID DetailStaticRegsInitialize(ISP_DETAIL_STATIC_CFG_S *pstStaticRegCfg)
{
    pstStaticRegCfg->u8DeNegClip     = HI_ISP_BNR_DE_NEGCLIP_DEFAULT;
    pstStaticRegCfg->u8DePosClip     = HI_ISP_BNR_DE_POSCLIP_DEFAULT;
    pstStaticRegCfg->u16ClipRatio    = 80;
    pstStaticRegCfg->u16LumaScaleX0  = 70;
    pstStaticRegCfg->u16LumaScaleX1  = 210;
    pstStaticRegCfg->u16LumaScaleY0  = 256;
    pstStaticRegCfg->u16LumaScaleY1  = 40;
    pstStaticRegCfg->u16LumaScaleK   = 24;
    pstStaticRegCfg->u16SatuGainX0   = 140;
    pstStaticRegCfg->u16SatuGainX1   = 200;
    pstStaticRegCfg->u16SatuGainY0   = 256;
    pstStaticRegCfg->u16SatuGainY1   = 80;
    pstStaticRegCfg->u16SatuGainK    = 25;
    pstStaticRegCfg->u16EdgeScaleX0  = 0;
    pstStaticRegCfg->u16EdgeScaleX1  = 256;
    pstStaticRegCfg->u16EdgeScaleY0  = 256;
    pstStaticRegCfg->u16EdgeScaleY1  = 256;
    pstStaticRegCfg->u16EdgeScaleK   = 0;
    pstStaticRegCfg->u16GlobalRatio  = 16;
    pstStaticRegCfg->bSgmMode        = HI_TRUE;
    pstStaticRegCfg->bEdgeMode       = HI_FALSE;
    pstStaticRegCfg->bLumaGainMode   = HI_FALSE;
    pstStaticRegCfg->u8LumaBitMode   = HI_FALSE;

    pstStaticRegCfg->bResh           = HI_TRUE;
}

static HI_VOID DetailDynaRegsInitialize(VI_PIPE ViPipe, ISP_DETAIL_DYNA_CFG_S *pstDetailDynaRegCfg, isp_usr_ctx *pstIspCtx)
{
    pstDetailDynaRegCfg->u8SgmPos4Gain   = 12;
    pstDetailDynaRegCfg->u8SgmPos3Gain   = 4;
    pstDetailDynaRegCfg->u8SgmPos2Gain   = 2;
    pstDetailDynaRegCfg->u8SgmNeg4Gain   = 12;
    pstDetailDynaRegCfg->u8SgmNeg3Gain   = 4;
    pstDetailDynaRegCfg->u8SgmNeg2Gain   = 2;
    pstDetailDynaRegCfg->u16GainLF       = 9;
    pstDetailDynaRegCfg->u16GainHFPos    = 12;
    pstDetailDynaRegCfg->u16GainHFNeg    = 12;

    pstDetailDynaRegCfg->bResh           = HI_TRUE;
}

static HI_VOID DetailUsrRegsInitialize(ISP_DETAIL_USR_CFG_S *pstDetailUsrRegCfg)
{
    HI_U16 i;

    for (i = 0; i < HI_ISP_DE_LUMA_GAIN_LUT_N; i++) {
        pstDetailUsrRegCfg->au16LumaGainLut[i] = g_au16swLumaGainLut[i];
    }

    pstDetailUsrRegCfg->bResh = HI_TRUE;
}

static HI_VOID DetailRegsInitialize(VI_PIPE ViPipe, isp_reg_cfg *pstRegCfg)
{
    HI_U8  i;
    HI_U8  u8BlockNum;
    isp_usr_ctx    *pstIspCtx = HI_NULL;
    ISP_DETAIL_S *pstDetail = HI_NULL;
    ISP_DETAIL_STATIC_CFG_S *pstStaticRegCfg = HI_NULL;
    ISP_DETAIL_DYNA_CFG_S   *pstDynaRegCfg   = HI_NULL;
    ISP_DETAIL_USR_CFG_S    *pstUsrRegCfg    = HI_NULL;

    DETAIL_GET_CTX(ViPipe, pstDetail);
    ISP_GET_CTX(ViPipe, pstIspCtx);

    u8BlockNum = pstIspCtx->block_attr.block_num;

    for (i = 0; i < u8BlockNum; i++) {
        pstStaticRegCfg = &pstRegCfg->alg_reg_cfg[i].stDeRegCfg.stStaticRegCfg;
        pstDynaRegCfg   = &pstRegCfg->alg_reg_cfg[i].stDeRegCfg.stDynaRegCfg;
        pstUsrRegCfg    = &pstRegCfg->alg_reg_cfg[i].stDeRegCfg.stUsrRegCfg;

        pstRegCfg->alg_reg_cfg[i].stDeRegCfg.bDeEnable                = pstDetail->bEnable;
        pstRegCfg->alg_reg_cfg[i].stDeRegCfg.stKernelRegCfg.bDeEnable = pstDetail->bEnable;
        DetailStaticRegsInitialize(pstStaticRegCfg);
        DetailDynaRegsInitialize(ViPipe, pstDynaRegCfg, pstIspCtx);
        DetailUsrRegsInitialize(pstUsrRegCfg);
    }

    pstRegCfg->cfg_key.bit1DetailCfg = 1;
}

static HI_BOOL __inline CheckDeOpen(ISP_DETAIL_S *pstDetail)
{
    return (pstDetail->bEnable == HI_TRUE);
}

static HI_S32 DetailReadExtregs(VI_PIPE ViPipe)
{
    HI_U8 i;
    HI_U32 au32ExpRatio[3] = {0};
    ISP_DETAIL_S *pstDetail = HI_NULL;
    isp_usr_ctx    *pstIspCtx = HI_NULL;

    DETAIL_GET_CTX(ViPipe, pstDetail);
    ISP_GET_CTX(ViPipe, pstIspCtx);

    pstDetail->bDeAttrUpdate     = hi_ext_system_detail_attr_update_read(ViPipe);
    hi_ext_system_detail_attr_update_write(ViPipe, HI_FALSE);

    if (pstDetail->bDeAttrUpdate == HI_TRUE) {
        pstDetail->enOpType              = hi_ext_system_detail_manual_mode_read(ViPipe);

        for (i = 0; i < HI_ISP_DE_LUMA_GAIN_LUT_N; i++) {
            pstDetail->au16LumaGainLut[i] = hi_ext_system_detail_luma_gain_lut_read(ViPipe, i);
        }

        memcpy(au32ExpRatio, pstIspCtx->linkage.exp_ratio_lut, 3 * sizeof(HI_U32));  // check

        for (i = 0; i < ISP_AUTO_ISO_STRENGTH_NUM; i++) {
            pstDetail->stAuto.au16GlobalGain[i]   = hi_ext_system_detail_auto_global_gain_read(ViPipe, i);
            pstDetail->stAuto.au16GainLF[i]       = hi_ext_system_detail_auto_gain_lf_read(ViPipe, i);
            pstDetail->stAuto.au16GainHF[i]       = hi_ext_system_detail_auto_gain_hf_read(ViPipe, i);
        }

        pstDetail->stManual.u16GlobalGain   = hi_ext_system_detail_manual_global_gain_read(ViPipe);
        pstDetail->stManual.u16GainLF       = hi_ext_system_detail_manual_gain_lf_read(ViPipe);
        pstDetail->stManual.u16GainHF       = hi_ext_system_detail_manual_gain_hf_read(ViPipe);
    }

    return HI_SUCCESS;
}

HI_S32 DetailProcWrite(VI_PIPE ViPipe, hi_isp_ctrl_proc_write *pstProc)
{
    hi_isp_ctrl_proc_write stProcTmp;
    ISP_DETAIL_S *pstDetail = HI_NULL;

    DETAIL_GET_CTX(ViPipe, pstDetail);
    ISP_CHECK_POINTER(pstDetail);

    if ((pstProc->proc_buff == HI_NULL)
        || (pstProc->buff_len == 0)) {
        return HI_FAILURE;
    }

    stProcTmp.proc_buff = pstProc->proc_buff;
    stProcTmp.buff_len = pstProc->buff_len;

    ISP_PROC_PRINTF(&stProcTmp, pstProc->write_len,
                    "-----DetailEnhance INFO----------------------------------------------------------------------------\n");

    ISP_PROC_PRINTF(&stProcTmp, pstProc->write_len,
                    "%16s"      "%16s"      "%16s"      "%16s"      "%16s\n",
                    "Enable",   "enOpType",   "GlobalGain", "GainHF", "GainLF");

    ISP_PROC_PRINTF(&stProcTmp, pstProc->write_len,
                    "%16u"      "%16u"      "%16u"      "%16u"      "%16u\n",
                    pstDetail->bEnable,
                    pstDetail->enOpType,
                    (HI_U16)pstDetail->u16ProcGlobalGain,
                    (HI_U16)pstDetail->u16ProcGainHF,
                    (HI_U16)pstDetail->u16ProcGainLF);

    pstProc->write_len += 1;

    return HI_SUCCESS;
}


HI_S32 ISP_DetailInit(VI_PIPE ViPipe, HI_VOID *pRegCfg)
{
    HI_S32 s32Ret;
    isp_reg_cfg *pstRegCfg = (isp_reg_cfg *)pRegCfg;

    s32Ret = DetailCtxInit(ViPipe);

    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    s32Ret = DetailExtRegsInitialize(ViPipe);
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    DetailRegsInitialize(ViPipe, pstRegCfg);
    return HI_SUCCESS;
}

static HI_S32  DEExtCfg(VI_PIPE ViPipe,  ISP_DETAIL_DYNA_CFG_S *pstDetailDynaCfg, ISP_DETAIL_USR_CFG_S *pstDetailUsrCfg, ISP_DETAIL_S *pstDetail, HI_U8  u8IsoIndexUpper, HI_U8 u8IsoIndexLower, HI_U32 u32ISO2, HI_U32 u32ISO1, HI_U32 u32Iso)
{
    HI_U32 i = 0;
    HI_S32 swGlobalGain;

    if (pstDetail->enOpType == OP_TYPE_AUTO) {
        pstDetailDynaCfg->u16GainLF    = (HI_U16)LinearInter(u32Iso, u32ISO1, pstDetail->stAuto.au16GainLF[u8IsoIndexLower], \
                                                             u32ISO2, pstDetail->stAuto.au16GainLF[u8IsoIndexUpper]);
        pstDetailDynaCfg->u16GainHFPos = (HI_U16)LinearInter(u32Iso, u32ISO1, pstDetail->stAuto.au16GainHF[u8IsoIndexLower], \
                                                             u32ISO2, pstDetail->stAuto.au16GainHF[u8IsoIndexUpper]);
        pstDetailDynaCfg->u16GainHFNeg = (HI_U16)pstDetailDynaCfg->u16GainHFPos;
        swGlobalGain                   = (HI_S32)LinearInter(u32Iso, u32ISO1, pstDetail->stAuto.au16GlobalGain[u8IsoIndexLower], \
                                                             u32ISO2, pstDetail->stAuto.au16GlobalGain[u8IsoIndexUpper]);
    } else {
        pstDetailDynaCfg->u16GainLF       = (HI_S32)pstDetail->stManual.u16GainLF;
        pstDetailDynaCfg->u16GainHFPos    = (HI_S32)pstDetail->stManual.u16GainHF;
        pstDetailDynaCfg->u16GainHFNeg    = (HI_S32)pstDetail->stManual.u16GainHF;
        swGlobalGain                      = (HI_S32)pstDetail->stManual.u16GlobalGain;
    }

    {
        pstDetailDynaCfg->u8SgmPos4Gain  = (HI_U8)LinearInter(u32Iso, u32ISO1, g_au16Sgm4Gain[u8IsoIndexLower], u32ISO2, g_au16Sgm4Gain[u8IsoIndexUpper]);
        pstDetailDynaCfg->u8SgmPos3Gain  = (HI_U8)LinearInter(u32Iso, u32ISO1, g_au16Sgm3Gain[u8IsoIndexLower], u32ISO2, g_au16Sgm3Gain[u8IsoIndexUpper]);
        pstDetailDynaCfg->u8SgmPos2Gain  = (HI_U8)LinearInter(u32Iso, u32ISO1, g_au16Sgm2Gain[u8IsoIndexLower], u32ISO2, g_au16Sgm2Gain[u8IsoIndexUpper]);
        pstDetailDynaCfg->u8SgmNeg4Gain  = pstDetailDynaCfg->u8SgmPos4Gain;
        pstDetailDynaCfg->u8SgmNeg3Gain  = pstDetailDynaCfg->u8SgmPos3Gain;
        pstDetailDynaCfg->u8SgmNeg2Gain  = pstDetailDynaCfg->u8SgmPos2Gain;
    }

    for (i = 0; i < HI_ISP_DE_LUMA_GAIN_LUT_N; i++) {
        pstDetailUsrCfg->au16LumaGainLut[i] = (HI_S32)pstDetail->au16LumaGainLut[i] * swGlobalGain / 256;
    }

    pstDetail->u16ProcGainHF     = pstDetailDynaCfg->u16GainHFPos;
    pstDetail->u16ProcGainLF     = pstDetailDynaCfg->u16GainLF;
    pstDetail->u16ProcGlobalGain = swGlobalGain;


    return HI_SUCCESS;
}

static HI_S32 hiisp_detail_fw(HI_U32 u32Iso, VI_PIPE ViPipe, HI_U8 u8CurBlk, ISP_DETAIL_DYNA_CFG_S *pstDetailDynaCfg, ISP_DETAIL_USR_CFG_S *pstDetailUsrCfg)
{
    HI_U8  u8IsoIndexUpper, u8IsoIndexLower;
    HI_U32 u32ISO1 = 0, u32ISO2 = 0;
    ISP_DETAIL_S *pstDetail = HI_NULL;

    DETAIL_GET_CTX(ViPipe, pstDetail);

    u8IsoIndexUpper = GetIsoIndex(u32Iso);
    u8IsoIndexLower = MAX2((HI_S8)u8IsoIndexUpper - 1, 0);
    u32ISO1     = g_au32IsoLut[u8IsoIndexLower];
    u32ISO2     = g_au32IsoLut[u8IsoIndexUpper];

    DEExtCfg(ViPipe,  pstDetailDynaCfg, pstDetailUsrCfg, pstDetail, u8IsoIndexUpper, u8IsoIndexLower, u32ISO2, u32ISO1, u32Iso);

    pstDetailDynaCfg->bResh = HI_TRUE;
    pstDetailUsrCfg->bResh  = HI_TRUE;

    return  HI_SUCCESS;
}

HI_S32 ISP_DetailRun(VI_PIPE ViPipe, const HI_VOID *pStatInfo,
                     HI_VOID *pRegCfg, HI_S32 s32Rsv)
{
    HI_U8  i;
    isp_reg_cfg *pstRegCfg = (isp_reg_cfg *)pRegCfg;
    isp_usr_ctx     *pstIspCtx = HI_NULL;
    ISP_DETAIL_S  *pstDetail = HI_NULL;

    ISP_GET_CTX(ViPipe, pstIspCtx);
    DETAIL_GET_CTX(ViPipe, pstDetail);
    ISP_CHECK_POINTER(pstDetail);

    /* calculate every two interrupts */
    if (!pstDetail->bInit) {
        return HI_SUCCESS;
    }

    for (i = 0; i < pstRegCfg->cfg_num; i++) {
        if (pstRegCfg->alg_reg_cfg[0].stBnrRegCfg.bBnrEnable == HI_TRUE) {
            pstDetail->bEnable = hi_ext_system_detail_enable_read(ViPipe);
            pstRegCfg->alg_reg_cfg[i].stDeRegCfg.bDeEnable = pstDetail->bEnable;
        } else {
            pstDetail->bEnable = HI_FALSE;
            pstRegCfg->alg_reg_cfg[i].stDeRegCfg.bDeEnable = HI_FALSE;
        }

        pstRegCfg->alg_reg_cfg[i].stDeRegCfg.stKernelRegCfg.bDeEnable = pstRegCfg->alg_reg_cfg[i].stDeRegCfg.bDeEnable;
    }

    pstRegCfg->cfg_key.bit1DetailCfg = 1;

    /* check hardware setting */
    if (!CheckDeOpen(pstDetail)) {
        return HI_SUCCESS;
    }

    DetailReadExtregs(ViPipe);

    for (i = 0; i < pstIspCtx->block_attr.block_num; i++) {
        hiisp_detail_fw(pstIspCtx->linkage.iso, ViPipe, i, &pstRegCfg->alg_reg_cfg[i].stDeRegCfg.stDynaRegCfg, &pstRegCfg->alg_reg_cfg[i].stDeRegCfg.stUsrRegCfg);
    }

    return HI_SUCCESS;
}

static HI_S32 ISP_DetailCtrl(VI_PIPE ViPipe, HI_U32 u32Cmd, HI_VOID *pValue)
{
    ISP_DETAIL_S *pstDetail = HI_NULL;
    isp_reg_cfg_attr  *pRegCfg  = HI_NULL;

    switch (u32Cmd) {
        case ISP_WDR_MODE_SET :
            ISP_REGCFG_GET_CTX(ViPipe, pRegCfg);
            DETAIL_GET_CTX(ViPipe, pstDetail);
            ISP_CHECK_POINTER(pRegCfg);
            ISP_CHECK_POINTER(pstDetail);

            pstDetail->bInit = HI_FALSE;
            ISP_DetailInit(ViPipe, (HI_VOID *)&pRegCfg->reg_cfg);
            break;
        case ISP_PROC_WRITE:
            DetailProcWrite(ViPipe, (hi_isp_ctrl_proc_write *)pValue);
            break;
        default :
            break;
    }
    return HI_SUCCESS;
}

HI_S32 ISP_DetailExit(VI_PIPE ViPipe)
{
    HI_U8 i;
    isp_reg_cfg_attr *pRegCfg   = HI_NULL;

    ISP_REGCFG_GET_CTX(ViPipe, pRegCfg);

    for (i = 0; i < pRegCfg->reg_cfg.cfg_num; i++) {
        pRegCfg->reg_cfg.alg_reg_cfg[i].stDeRegCfg.bDeEnable = HI_FALSE;
    }

    pRegCfg->reg_cfg.cfg_key.bit1DetailCfg = 1;

    DetailCtxExit(ViPipe);

    return HI_SUCCESS;
}

HI_S32 isp_alg_register_detail(VI_PIPE ViPipe)
{
    isp_usr_ctx *pstIspCtx = HI_NULL;
    isp_alg_node *pstAlgs = HI_NULL;

    ISP_GET_CTX(ViPipe, pstIspCtx);
    ISP_ALG_CHECK(pstIspCtx->alg_key.bit1_detail);
    pstAlgs = ISP_SearchAlg(pstIspCtx->algs);
    ISP_CHECK_POINTER(pstAlgs);

    pstAlgs->alg_type = ISP_ALG_DETAIL;
    pstAlgs->alg_func.pfn_alg_init = ISP_DetailInit;
    pstAlgs->alg_func.pfn_alg_run  = ISP_DetailRun;
    pstAlgs->alg_func.pfn_alg_ctrl = ISP_DetailCtrl;
    pstAlgs->alg_func.pfn_alg_exit = ISP_DetailExit;
    pstAlgs->used = HI_TRUE;

    return HI_SUCCESS;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
