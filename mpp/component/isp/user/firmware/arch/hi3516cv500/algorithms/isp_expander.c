/*
* Copyright (C) Hisilicon Technologies Co., Ltd. 2012-2019. All rights reserved.
* Description:
* Author: Hisilicon multimedia software group
* Create: 2011/06/28
*/


#include "isp_alg.h"
#include "isp_sensor.h"
#include "isp_config.h"
#include "isp_proc.h"
#include "isp_math_utils.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

static HI_VOID ExpanderStaticRegsInitialize(VI_PIPE ViPipe, ISP_EXPANDER_STATIC_CFG_S *pstStaticRegCfg)
{
    HI_U32 _i, _v;
    HI_U32 X0, Y0, X1, Y1, X2, Y2, X3, Y3, X_max, Y_max;
    hi_isp_cmos_default *sns_dft = HI_NULL;

    isp_sensor_get_default(ViPipe, &sns_dft);

    if (sns_dft->key.bit1_expander) {
        pstStaticRegCfg->u8BitDepthIn   = sns_dft->expander->bit_depth_in;
        pstStaticRegCfg->u8BitDepthOut  = sns_dft->expander->bit_depth_out;

        X0       = sns_dft->expander->expander_point[0].x;
        Y0       = sns_dft->expander->expander_point[0].y;
        X1       = sns_dft->expander->expander_point[1].x;
        Y1       = sns_dft->expander->expander_point[1].y;
        X2       = sns_dft->expander->expander_point[2].x;
        Y2       = sns_dft->expander->expander_point[2].y;
        X3       = sns_dft->expander->expander_point[3].x;
        Y3       = sns_dft->expander->expander_point[3].y;
        X_max    = sns_dft->expander->expander_point[4].x;
        Y_max    = sns_dft->expander->expander_point[4].y;

        for (_i = 0; _i < X0; _i++) {
            _v = (((_i * Y0) / DIV_0_TO_1(X0)) >> 0);
            pstStaticRegCfg->au32Lut[_i] = _v;
        }

        for (; _i < X1; _i++) {
            _v = ((((_i - X0) * (Y1 - Y0)) / DIV_0_TO_1(X1 - X0) + Y0)  >> 0);
            pstStaticRegCfg->au32Lut[_i] = _v;
        }

        for (; _i < X2; _i++) {
            _v = ((((_i - X1) * (Y2 - Y1)) / DIV_0_TO_1(X2 - X1) + Y1)  >> 0);
            pstStaticRegCfg->au32Lut[_i] = _v;
        }

        for (; _i < X3; _i++) {
            _v = ((((_i - X2) * (Y3 - Y2)) / DIV_0_TO_1(X3 - X2) + Y2)  >> 0);
            pstStaticRegCfg->au32Lut[_i] = _v;
        }

        for (; _i < X_max; _i++) {
            _v = (Y_max  >> 0);
            pstStaticRegCfg->au32Lut[_i] = _v;
        }
    } else {
        pstStaticRegCfg->u8BitDepthIn   = 12;
        pstStaticRegCfg->u8BitDepthOut  = 16;
        memset(pstStaticRegCfg->au32Lut, 0, EXPANDER_NODE_NUM * sizeof(HI_U32));
    }

    pstStaticRegCfg->bResh = HI_TRUE;

}

static HI_S32 ExpanderCheckCmosParam(VI_PIPE ViPipe, const hi_isp_cmos_expander *cmos_expander)
{
    HI_U8 i;

    ISP_CHECK_BOOL(cmos_expander->enable);

    if ((cmos_expander->bit_depth_in > 0x14) || (cmos_expander->bit_depth_in < 0xC)) {
        ISP_ERR_TRACE("Invalid u8BitDepthIn!\n");
        return HI_ERR_ISP_ILLEGAL_PARAM;
    }

    if ((cmos_expander->bit_depth_out > 0x14) || (cmos_expander->bit_depth_out < 0xC)) {
        ISP_ERR_TRACE("Invalid u8BitDepthOut!\n");
        return HI_ERR_ISP_ILLEGAL_PARAM;
    }

    for (i = 0; i < ISP_EXPANDER_POINT_NUM; i++) {
        if (cmos_expander->expander_point[i].x > 0x101) {
            ISP_ERR_TRACE("Invalid astExpanderPoint[%d].u16X!\n", i);
            return HI_ERR_ISP_ILLEGAL_PARAM;
        }

        if (cmos_expander->expander_point[i].y > 0x100000) {
            ISP_ERR_TRACE("Invalid astExpanderPoint[%d].u32Y!\n", i);
            return HI_ERR_ISP_ILLEGAL_PARAM;
        }
    }

    return HI_SUCCESS;
}

static HI_S32 ExpanderRegsInitialize(VI_PIPE ViPipe, isp_reg_cfg *pstRegCfg)
{
    HI_U8     i;
    HI_S32    s32Ret;
    hi_isp_cmos_default *sns_dft = HI_NULL;
    isp_usr_ctx *pstIspCtx = HI_NULL;

    isp_sensor_get_default(ViPipe, &sns_dft);
    ISP_GET_CTX(ViPipe, pstIspCtx);

    if (sns_dft->key.bit1_expander) {
        ISP_CHECK_POINTER(sns_dft->expander);

        s32Ret = ExpanderCheckCmosParam(ViPipe, sns_dft->expander);
        if (s32Ret != HI_SUCCESS) {
            return s32Ret;
        }
    }

    for (i = 0; i < pstRegCfg->cfg_num; i++) {
        ExpanderStaticRegsInitialize(ViPipe, &pstRegCfg->alg_reg_cfg[i].stExpanderCfg.stStaticCfg);

        if (pstIspCtx->sns_wdr_mode == WDR_MODE_BUILT_IN) {
            if (sns_dft->key.bit1_expander) {
                pstRegCfg->alg_reg_cfg[i].stExpanderCfg.bEnable = sns_dft->expander->enable;
            } else {
                pstRegCfg->alg_reg_cfg[i].stExpanderCfg.bEnable = HI_FALSE;
            }
        } else {
            pstRegCfg->alg_reg_cfg[i].stExpanderCfg.bEnable = HI_FALSE;
        }
    }

    pstRegCfg->cfg_key.bit1ExpanderCfg = 1;

    return HI_SUCCESS;
}

HI_S32 ISP_ExpanderInit(VI_PIPE ViPipe, HI_VOID *pRegCfg)
{
    HI_S32    s32Ret;
    isp_reg_cfg *pstRegCfg = (isp_reg_cfg *)pRegCfg;

    s32Ret = ExpanderRegsInitialize(ViPipe, pstRegCfg);
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    return HI_SUCCESS;
}

HI_S32 ISP_ExpanderRun(VI_PIPE ViPipe, const HI_VOID *pStatInfo,
                       HI_VOID *pRegCfg, HI_S32 s32Rsv)
{
    return HI_SUCCESS;
}

HI_S32 ISP_ExpanderCtrl(VI_PIPE ViPipe, HI_U32 u32Cmd, HI_VOID *pValue)
{
    isp_reg_cfg_attr  *pRegCfg   = HI_NULL;

    switch (u32Cmd) {
        case ISP_WDR_MODE_SET:
            ISP_REGCFG_GET_CTX(ViPipe, pRegCfg);
            ISP_CHECK_POINTER(pRegCfg);
            ISP_ExpanderInit(ViPipe, (HI_VOID *)&pRegCfg->reg_cfg);
            break;
        default :
            break;
    }

    return HI_SUCCESS;
}

HI_S32 ISP_ExpanderExit(VI_PIPE ViPipe)
{
    HI_U8 i;
    isp_reg_cfg_attr  *pRegCfg   = HI_NULL;

    ISP_REGCFG_GET_CTX(ViPipe, pRegCfg);

    for (i = 0; i < pRegCfg->reg_cfg.cfg_num; i++) {
        pRegCfg->reg_cfg.alg_reg_cfg[i].stExpanderCfg.bEnable = HI_FALSE;
    }

    pRegCfg->reg_cfg.cfg_key.bit1ExpanderCfg = 1;

    return HI_SUCCESS;
}

HI_S32 isp_alg_register_expander(VI_PIPE ViPipe)
{
    isp_usr_ctx *pstIspCtx = HI_NULL;
    isp_alg_node *pstAlgs = HI_NULL;

    ISP_GET_CTX(ViPipe, pstIspCtx);
    ISP_ALG_CHECK(pstIspCtx->alg_key.bit1_expander);
    pstAlgs = ISP_SearchAlg(pstIspCtx->algs);
    ISP_CHECK_POINTER(pstAlgs);

    pstAlgs->alg_type = ISP_ALG_EXPANDER;
    pstAlgs->alg_func.pfn_alg_init = ISP_ExpanderInit;
    pstAlgs->alg_func.pfn_alg_run  = ISP_ExpanderRun;
    pstAlgs->alg_func.pfn_alg_ctrl = ISP_ExpanderCtrl;
    pstAlgs->alg_func.pfn_alg_exit = ISP_ExpanderExit;
    pstAlgs->used = HI_TRUE;

    return HI_SUCCESS;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
