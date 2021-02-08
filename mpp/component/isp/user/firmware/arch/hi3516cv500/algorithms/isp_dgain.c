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

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

typedef struct hiISP_DGain {
    HI_BOOL bEnable;
} ISP_DG_S;

ISP_DG_S g_astDgCtx[ISP_MAX_PIPE_NUM] = {{0}};
#define DG_GET_CTX(dev, pstCtx)   pstCtx = &g_astDgCtx[dev]

static HI_VOID BeDgStaticInit(VI_PIPE ViPipe, ISP_DG_STATIC_CFG_S *pstStaticRegCfg)
{
    pstStaticRegCfg->bResh = HI_TRUE;
}

static HI_VOID BeDgDynaInit(VI_PIPE ViPipe, ISP_DG_DYNA_CFG_S *pstDynaRegCfg, HI_U16 u16Gain)
{
    pstDynaRegCfg->u32ClipValue = 0xFFFFF;
    pstDynaRegCfg->u16GainR     = u16Gain;
    pstDynaRegCfg->u16GainGR    = u16Gain;
    pstDynaRegCfg->u16GainGB    = u16Gain;
    pstDynaRegCfg->u16GainB     = u16Gain;
    pstDynaRegCfg->bResh        = HI_TRUE;
}

static HI_VOID WDRDgStaticInit(VI_PIPE ViPipe, HI_U8 i, ISP_4DG_STATIC_CFG_S *pstStaticRegCfg)
{
    pstStaticRegCfg->u16GainR0  = 0x100;
    pstStaticRegCfg->u16GainGR0 = 0x100;
    pstStaticRegCfg->u16GainGB0 = 0x100;
    pstStaticRegCfg->u16GainB0  = 0x100;
    pstStaticRegCfg->u16GainR1  = 0x100;
    pstStaticRegCfg->u16GainGR1 = 0x100;
    pstStaticRegCfg->u16GainGB1 = 0x100;
    pstStaticRegCfg->u16GainB1  = 0x100;
    pstStaticRegCfg->u16GainR2  = 0x100;
    pstStaticRegCfg->u16GainGR2 = 0x100;
    pstStaticRegCfg->u16GainGB2 = 0x100;
    pstStaticRegCfg->u16GainB2  = 0x100;
    pstStaticRegCfg->u16GainR3  = 0x100;
    pstStaticRegCfg->u16GainGR3 = 0x100;
    pstStaticRegCfg->u16GainGB3 = 0x100;
    pstStaticRegCfg->u16GainB3  = 0x100;
    pstStaticRegCfg->bResh      = HI_TRUE;
}

static HI_VOID WDRDgDynaInit(VI_PIPE ViPipe, ISP_4DG_DYNA_CFG_S *pstDynaRegCfg)
{
    pstDynaRegCfg->u32ClipValue0  = 0xFFFFF;
    pstDynaRegCfg->u32ClipValue1  = 0xFFFFF;
    pstDynaRegCfg->u32ClipValue2  = 0xFFFFF;
    pstDynaRegCfg->u32ClipValue3  = 0xFFFFF;
    pstDynaRegCfg->bResh          = HI_TRUE;
}

static HI_VOID FeDgDynaInit(VI_PIPE ViPipe, ISP_FE_DG_DYNA_CFG_S *pstDynaRegCfg, HI_U16 u16Gain)
{
    HI_U8 i = 0;

    for (i = 0; i < 4; i++) {
        pstDynaRegCfg->au16GainR[i]  = u16Gain;
        pstDynaRegCfg->au16GainGR[i] = u16Gain;
        pstDynaRegCfg->au16GainGB[i] = u16Gain;
        pstDynaRegCfg->au16GainB[i]  = u16Gain;
    }
    pstDynaRegCfg->u32ClipValue  = 0xFFFFF;
    pstDynaRegCfg->bResh         = HI_TRUE;
}

static HI_VOID WDRDgEnInit(VI_PIPE ViPipe, HI_U8 i, isp_reg_cfg *pstRegCfg)
{
    HI_U8 u8WDRMode = 0;
    isp_usr_ctx *pstIspCtx = HI_NULL;

    ISP_GET_CTX(ViPipe, pstIspCtx);

    u8WDRMode = pstIspCtx->sns_wdr_mode;

    if (IS_LINEAR_MODE(u8WDRMode)) {
        pstRegCfg->alg_reg_cfg[i].st4DgRegCfg.bEnable = HI_FALSE;
    } else if (IS_2to1_WDR_MODE(u8WDRMode) || IS_3to1_WDR_MODE(u8WDRMode) || IS_4to1_WDR_MODE(u8WDRMode)) {
        pstRegCfg->alg_reg_cfg[i].st4DgRegCfg.bEnable = HI_TRUE;
    } else if (IS_BUILT_IN_WDR_MODE(u8WDRMode)) {
        pstRegCfg->alg_reg_cfg[i].st4DgRegCfg.bEnable = HI_FALSE;
    }
}

static HI_VOID DgRegsInitialize(VI_PIPE ViPipe, isp_reg_cfg *pstRegCfg)
{
    HI_U8  i;
    HI_U16 u16Gain;
    hi_isp_cmos_black_level *sns_black_level = HI_NULL;

    isp_sensor_get_blc(ViPipe, &sns_black_level);

    u16Gain = CLIP3(4095 * 256 / DIV_0_TO_1(4095 - sns_black_level->black_level[1]) + 1, 0x100, 0x200);

    for (i = 0; i < pstRegCfg->cfg_num; i++) {
        BeDgStaticInit(ViPipe, &pstRegCfg->alg_reg_cfg[i].stDgRegCfg.stStaticRegCfg);
        BeDgDynaInit(ViPipe, &pstRegCfg->alg_reg_cfg[i].stDgRegCfg.stDynaRegCfg, u16Gain);

        WDRDgStaticInit(ViPipe, i, &pstRegCfg->alg_reg_cfg[i].st4DgRegCfg.stStaticRegCfg);
        WDRDgDynaInit(ViPipe, &pstRegCfg->alg_reg_cfg[i].st4DgRegCfg.stDynaRegCfg);

        WDRDgEnInit(ViPipe, i, pstRegCfg);

        pstRegCfg->alg_reg_cfg[i].stDgRegCfg.bDgEn = HI_TRUE;
        pstRegCfg->cfg_key.bit1DgCfg                 = 1;
        pstRegCfg->cfg_key.bit1WDRDgCfg              = 1;
    }

    FeDgDynaInit(ViPipe, &pstRegCfg->alg_reg_cfg[0].stFeDgRegCfg.stDynaRegCfg, u16Gain);
    pstRegCfg->alg_reg_cfg[0].stFeDgRegCfg.bDgEn = HI_TRUE;
    pstRegCfg->cfg_key.bit1FeDgCfg                 = 1;
}

static HI_VOID DgExtRegsInitialize(VI_PIPE ViPipe)
{
    hi_ext_system_isp_dgain_enable_write(ViPipe, HI_TRUE);
}

static HI_VOID DgInitialize(VI_PIPE ViPipe)
{
    ISP_DG_S *pstDg = HI_NULL;

    DG_GET_CTX(ViPipe, pstDg);

    pstDg->bEnable = HI_TRUE;
}

static HI_VOID ISP_DgWdrModeSet(VI_PIPE ViPipe, HI_VOID *pRegCfg)
{
    HI_U8  i, j;
    HI_U16 u16Gain;
    isp_reg_cfg           *pstRegCfg        = (isp_reg_cfg *)pRegCfg;
    hi_isp_cmos_black_level *sns_black_level = HI_NULL;

    isp_sensor_get_blc(ViPipe, &sns_black_level);

    u16Gain = CLIP3(4095 * 256 / DIV_0_TO_1(4095 - sns_black_level->black_level[1]) + 1, 0x100, 0x200);

    for (i = 0; i < pstRegCfg->cfg_num; i++) {
        WDRDgEnInit(ViPipe, i, pstRegCfg);
        pstRegCfg->alg_reg_cfg[i].stDgRegCfg.stStaticRegCfg.bResh   = HI_TRUE;
        pstRegCfg->alg_reg_cfg[i].st4DgRegCfg.stStaticRegCfg.bResh  = HI_TRUE;

        pstRegCfg->alg_reg_cfg[i].stDgRegCfg.stDynaRegCfg.u16GainR  = u16Gain;
        pstRegCfg->alg_reg_cfg[i].stDgRegCfg.stDynaRegCfg.u16GainGR = u16Gain;
        pstRegCfg->alg_reg_cfg[i].stDgRegCfg.stDynaRegCfg.u16GainGB = u16Gain;
        pstRegCfg->alg_reg_cfg[i].stDgRegCfg.stDynaRegCfg.u16GainB  = u16Gain;
    }

    pstRegCfg->cfg_key.bit1WDRDgCfg = 1;
    pstRegCfg->cfg_key.bit1FeDgCfg  = 1;
    pstRegCfg->cfg_key.bit1DgCfg    = 1;

    for (j = 0; j < 4; j++) {
        pstRegCfg->alg_reg_cfg[0].stFeDgRegCfg.stDynaRegCfg.au16GainR[j]  = u16Gain;
        pstRegCfg->alg_reg_cfg[0].stFeDgRegCfg.stDynaRegCfg.au16GainGR[j] = u16Gain;
        pstRegCfg->alg_reg_cfg[0].stFeDgRegCfg.stDynaRegCfg.au16GainGB[j] = u16Gain;
        pstRegCfg->alg_reg_cfg[0].stFeDgRegCfg.stDynaRegCfg.au16GainB[j]  = u16Gain;
    }
}

HI_S32 ISP_DgInit(VI_PIPE ViPipe, HI_VOID *pRegCfg)
{
    isp_reg_cfg *pstRegCfg = (isp_reg_cfg *)pRegCfg;

    DgRegsInitialize(ViPipe, pstRegCfg);
    DgExtRegsInitialize(ViPipe);
    DgInitialize(ViPipe);

    return HI_SUCCESS;
}

static HI_BOOL __inline CheckDgOpen(ISP_DG_S *pstDg)
{
    return (pstDg->bEnable == HI_TRUE);
}

HI_S32 ISP_DgRun(VI_PIPE ViPipe, const HI_VOID *pStatInfo,
                 HI_VOID *pRegCfg, HI_S32 s32Rsv)
{
    HI_S32 i;
    HI_U32 u32IspDgain;
    isp_usr_ctx *pstIspCtx = HI_NULL;
    ISP_DG_S  *pstDg     = HI_NULL;
    isp_reg_cfg *pstRegCfg = (isp_reg_cfg *)pRegCfg;
    HI_U32 u32WDRGain;

    ISP_GET_CTX(ViPipe, pstIspCtx);
    DG_GET_CTX(ViPipe, pstDg);

    if (pstIspCtx->linkage.defect_pixel &&
        (hi_ext_system_dpc_static_defect_type_read(ViPipe) == 0)) {
        for (i = 0; i < pstRegCfg->cfg_num; i++) {
            pstRegCfg->alg_reg_cfg[i].stDgRegCfg.bDgEn = HI_FALSE;
        }

        pstRegCfg->cfg_key.bit1DgCfg   = 1;

        return HI_SUCCESS;
    }

    pstDg->bEnable = hi_ext_system_isp_dgain_enable_read(ViPipe);

    for (i = 0; i < pstRegCfg->cfg_num; i++) {
        pstRegCfg->alg_reg_cfg[i].stDgRegCfg.bDgEn = pstDg->bEnable;
    }

    pstRegCfg->alg_reg_cfg[0].stFeDgRegCfg.bDgEn = pstDg->bEnable;

    pstRegCfg->cfg_key.bit1FeDgCfg = 1;
    pstRegCfg->cfg_key.bit1DgCfg   = 1;

    /* check hardware setting */
    if (!CheckDgOpen(pstDg)) {
        return HI_SUCCESS;
    }

    u32IspDgain = pstIspCtx->linkage.isp_dgain;

    for (i = 0; i < 4; i++) {
        u32WDRGain = ((HI_U64)u32IspDgain * pstIspCtx->linkage.wdr_gain[i]) >> 8;
        pstRegCfg->alg_reg_cfg[0].stFeDgRegCfg.stDynaRegCfg.au16GainR[i]  = u32WDRGain;
        pstRegCfg->alg_reg_cfg[0].stFeDgRegCfg.stDynaRegCfg.au16GainGR[i] = u32WDRGain;
        pstRegCfg->alg_reg_cfg[0].stFeDgRegCfg.stDynaRegCfg.au16GainGB[i] = u32WDRGain;
        pstRegCfg->alg_reg_cfg[0].stFeDgRegCfg.stDynaRegCfg.au16GainB[i]  = u32WDRGain;
    }

    pstRegCfg->alg_reg_cfg[0].stFeDgRegCfg.stDynaRegCfg.bResh     = HI_TRUE;

    return HI_SUCCESS;
}

HI_S32 ISP_DgCtrl(VI_PIPE ViPipe, HI_U32 u32Cmd, HI_VOID *pValue)
{
    isp_reg_cfg_attr  *pRegCfg   = HI_NULL;

    switch (u32Cmd) {
        case ISP_WDR_MODE_SET:
            ISP_REGCFG_GET_CTX(ViPipe, pRegCfg);
            ISP_CHECK_POINTER(pRegCfg);
            ISP_DgWdrModeSet(ViPipe, (HI_VOID *)&pRegCfg->reg_cfg);
            break;
        default:
            break;
    }

    return HI_SUCCESS;
}

HI_S32 ISP_DgExit(VI_PIPE ViPipe)
{
    return HI_SUCCESS;
}

HI_S32 isp_alg_register_dg(VI_PIPE ViPipe)
{
    isp_usr_ctx      *pstIspCtx = HI_NULL;
    isp_alg_node *pstAlgs   = HI_NULL;

    ISP_GET_CTX(ViPipe, pstIspCtx);
    ISP_ALG_CHECK(pstIspCtx->alg_key.bit1_dg);
    pstAlgs = ISP_SearchAlg(pstIspCtx->algs);
    ISP_CHECK_POINTER(pstAlgs);

    pstAlgs->alg_type = ISP_ALG_DG;
    pstAlgs->alg_func.pfn_alg_init = ISP_DgInit;
    pstAlgs->alg_func.pfn_alg_run  = ISP_DgRun;
    pstAlgs->alg_func.pfn_alg_ctrl = ISP_DgCtrl;
    pstAlgs->alg_func.pfn_alg_exit = ISP_DgExit;
    pstAlgs->used = HI_TRUE;

    return HI_SUCCESS;
}
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
