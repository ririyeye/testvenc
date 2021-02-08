/*
* Copyright (C) Hisilicon Technologies Co., Ltd. 2012-2019. All rights reserved.
* Description:
* Author: Hisilicon multimedia software group
* Create: 2011/06/28
*/

#include "isp_config.h"
#include "hi_isp_debug.h"
#include "isp_ext_config.h"
#include "isp_math_utils.h"
#include "isp_alg.h"
#include "isp_sensor.h"
#include "isp_proc.h"


#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

typedef struct hiISP_HLC_S {
    HI_BOOL bHlcMpiUpdateEn;
    HI_BOOL bEnable;                        /* RW; Range:[0, 1]; Format:1.0;Enable/Disable Edge Mark */
    HI_U8   u8LumaThr;                      /* RW; range: [0, 255];  Format:8.0; */
    HI_U8   u8LumaTarget;                   /* RW; range: [0, 255];  Format:8.0; */
} ISP_HLC_S;

ISP_HLC_S g_astHlcCtx[ISP_MAX_PIPE_NUM] = {{0}};
#define HLC_GET_CTX(dev, pstCtx)   pstCtx = &g_astHlcCtx[dev]

static HI_VOID HlcExtRegsInitialize(VI_PIPE ViPipe)
{
    ISP_HLC_S *pstHlc = HI_NULL;

    HLC_GET_CTX(ViPipe, pstHlc);

    hi_ext_system_hlc_mpi_update_en_write(ViPipe, HI_FALSE);
    hi_ext_system_manual_isp_hlc_en_write(ViPipe, pstHlc->bEnable);
    hi_ext_system_manual_isp_hlc_lumathr_write(ViPipe, pstHlc->u8LumaThr);
    hi_ext_system_manual_isp_hlc_lumatarget_write(ViPipe, pstHlc->u8LumaTarget);
}

static void HlcCheckReg(ISP_HLC_REG_CFG_S *pstHlcReg)
{
    pstHlcReg->bEnable = MIN2(pstHlcReg->bEnable, 0x1);
}

static HI_VOID HlcRegInit(ISP_HLC_REG_CFG_S *pstHlcReg, ISP_HLC_S *pstHlc)
{
    pstHlcReg->u8yMaxValue     = pstHlc->u8LumaThr;
    pstHlcReg->u8yMaxLoadValue = pstHlc->u8LumaTarget;
    pstHlcReg->u8yMinValue     = 0;
    pstHlcReg->u8yMinLoadValue = 0;
    pstHlcReg->u8cMaxValue     = 255;
    pstHlcReg->u8cMaxLoadValue = 255;
    pstHlcReg->u8cMinValue     = 0;
    pstHlcReg->u8cMinLoadValue = 0;
    HlcCheckReg(pstHlcReg);
}

static HI_VOID HlcRegsInitialize(VI_PIPE ViPipe, isp_reg_cfg *pRegCfg)
{
    HI_U32 i;

    ISP_HLC_S *pstHlc = HI_NULL;

    HLC_GET_CTX(ViPipe, pstHlc);

    for (i = 0; i < pRegCfg->cfg_num; i++) {
        pRegCfg->alg_reg_cfg[i].stHlcRegCfg.bEnable = pstHlc->bEnable;
        HlcRegInit(&pRegCfg->alg_reg_cfg[i].stHlcRegCfg, pstHlc);
    }

    pRegCfg->cfg_key.bit1HlcCfg = 1;
}

static HI_S32 HlcReadExtregs(VI_PIPE ViPipe)
{
    ISP_HLC_S *pstHlc = HI_NULL;

    HLC_GET_CTX(ViPipe, pstHlc);
    pstHlc->bHlcMpiUpdateEn = hi_ext_system_hlc_mpi_update_en_read(ViPipe);
    hi_ext_system_hlc_mpi_update_en_write(ViPipe, HI_FALSE);

    if (pstHlc->bHlcMpiUpdateEn) {
        pstHlc->u8LumaThr    = hi_ext_system_manual_isp_hlc_lumathr_read(ViPipe);
        pstHlc->u8LumaTarget = hi_ext_system_manual_isp_hlc_lumatarget_read(ViPipe);
    }

    return HI_SUCCESS;
}

static HI_S32 HlcCheckCmosParam(VI_PIPE ViPipe, const hi_isp_cmos_hlc *cmos_hlc)
{
    ISP_CHECK_BOOL(cmos_hlc->enable);

    return HI_SUCCESS;
}

static HI_S32 HlcInitialize(VI_PIPE ViPipe)
{
    HI_S32 s32Ret;
    ISP_HLC_S          *pstHlc    = HI_NULL;
    hi_isp_cmos_default *sns_dft = HI_NULL;

    HLC_GET_CTX(ViPipe, pstHlc);
    isp_sensor_get_default(ViPipe, &sns_dft);

    if (sns_dft->key.bit1_hlc) {
        ISP_CHECK_POINTER(sns_dft->hlc);

        s32Ret = HlcCheckCmosParam(ViPipe, sns_dft->hlc);

        if (s32Ret != HI_SUCCESS) {
            return s32Ret;
        }

        pstHlc->bEnable = sns_dft->hlc->enable;
        pstHlc->u8LumaThr = sns_dft->hlc->luma_thr;
        pstHlc->u8LumaTarget = sns_dft->hlc->luma_target;
    } else {
        pstHlc->bEnable      = HI_FALSE;
        pstHlc->u8LumaThr    = 248;
        pstHlc->u8LumaTarget = 10;
    }

    return HI_SUCCESS;
}

HI_S32 ISP_HlcInit(VI_PIPE ViPipe, HI_VOID *pRegCfg)
{
    HI_S32 s32Ret = HI_SUCCESS;

    s32Ret = HlcInitialize(ViPipe);

    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    HlcRegsInitialize(ViPipe, (isp_reg_cfg *)pRegCfg);
    HlcExtRegsInitialize(ViPipe);

    return HI_SUCCESS;
}

static void HlcMPI2Reg(ISP_HLC_REG_CFG_S *pstHlcRegCfg, ISP_HLC_S *pstHlc)
{
    pstHlcRegCfg->u8yMaxValue     = pstHlc->u8LumaThr;
    pstHlcRegCfg->u8yMaxLoadValue = pstHlc->u8LumaTarget;

    HlcCheckReg(pstHlcRegCfg);
}

static HI_BOOL __inline CheckHlcOpen(ISP_HLC_S *pstHlc)
{
    return (pstHlc->bEnable == HI_TRUE);
}

HI_S32 HlcProcWrite(VI_PIPE ViPipe, hi_isp_ctrl_proc_write *pstProc)
{
    hi_isp_ctrl_proc_write stProcTmp;

    ISP_HLC_S *pstHlc = HI_NULL;

    HLC_GET_CTX(ViPipe, pstHlc);

    if ((pstProc->proc_buff == HI_NULL) || (pstProc->buff_len == 0)) {
        return HI_FAILURE;
    }


    stProcTmp.proc_buff = pstProc->proc_buff;
    stProcTmp.buff_len = pstProc->buff_len;

    ISP_PROC_PRINTF(&stProcTmp, pstProc->write_len,
                    "-----HLC INFO--------------------------------------------------------------\n");

    ISP_PROC_PRINTF(&stProcTmp, pstProc->write_len,
                    "%16s"    "%16s"        "%16s\n",
                    "Enable", "LumaThr", "LumaTarget");

    ISP_PROC_PRINTF(&stProcTmp, pstProc->write_len,
                    "%16u"  "%16u"      "%16u\n",
                    (HI_U16)pstHlc->bEnable,
                    (HI_U16)pstHlc->u8LumaThr,
                    (HI_U16)pstHlc->u8LumaTarget);

    pstProc->write_len += 1;

    return HI_SUCCESS;
}

HI_S32 ISP_HlcRun(VI_PIPE ViPipe, const HI_VOID *pStatInfo, HI_VOID *pRegCfg, HI_S32 s32Rsv)
{
    HI_U8  i;
    isp_usr_ctx *pstIspCtx = HI_NULL;
    ISP_HLC_S *pstHlc = HI_NULL;
    isp_reg_cfg *pstRegCfg = (isp_reg_cfg *)pRegCfg;

    ISP_GET_CTX(ViPipe, pstIspCtx);
    HLC_GET_CTX(ViPipe, pstHlc);

    if (pstIspCtx->linkage.defect_pixel) {
        return HI_SUCCESS;
    }

    /* calculate every two interrupts */
    if ((pstIspCtx->frame_cnt % 2 != 0) && (pstIspCtx->linkage.snap_state != HI_TRUE)) {
        return HI_SUCCESS;
    }

    pstHlc->bEnable = hi_ext_system_manual_isp_hlc_en_read(ViPipe);

    for (i = 0; i < pstRegCfg->cfg_num; i++) {
        pstRegCfg->alg_reg_cfg[i].stHlcRegCfg.bEnable = pstHlc->bEnable;
    }

    pstRegCfg->cfg_key.bit1HlcCfg = 1;

    /* check hardware setting */
    if (!CheckHlcOpen(pstHlc)) {
        return HI_SUCCESS;
    }

    HlcReadExtregs(ViPipe);

    if (pstHlc->bHlcMpiUpdateEn) {
        for (i = 0; i < pstRegCfg->cfg_num; i++) {
            HlcMPI2Reg(&(pstRegCfg->alg_reg_cfg[i].stHlcRegCfg), pstHlc);
        }
    }

    return HI_SUCCESS;
}

HI_S32 ISP_HlcCtrl(VI_PIPE ViPipe, HI_U32 u32Cmd, HI_VOID *pValue)
{
    switch (u32Cmd) {
        case ISP_PROC_WRITE:
            HlcProcWrite(ViPipe, (hi_isp_ctrl_proc_write *)pValue);
            break;

        default :
            break;
    }

    return HI_SUCCESS;
}

HI_S32 ISP_HlcExit(VI_PIPE ViPipe)
{
    HI_U8 i;
    isp_reg_cfg_attr  *pRegCfg   = HI_NULL;

    ISP_REGCFG_GET_CTX(ViPipe, pRegCfg);

    for (i = 0; i < pRegCfg->reg_cfg.cfg_num; i++) {
        pRegCfg->reg_cfg.alg_reg_cfg[i].stHlcRegCfg.bEnable = HI_FALSE;
    }

    pRegCfg->reg_cfg.cfg_key.bit1HlcCfg = 1;

    return HI_SUCCESS;
}

HI_S32 isp_alg_register_hlc(VI_PIPE ViPipe)
{
    isp_usr_ctx *pstIspCtx = HI_NULL;
    isp_alg_node *pstAlgs = HI_NULL;

    ISP_GET_CTX(ViPipe, pstIspCtx);
    ISP_ALG_CHECK(pstIspCtx->alg_key.bit1_hlc);
    pstAlgs = ISP_SearchAlg(pstIspCtx->algs);
    ISP_CHECK_POINTER(pstAlgs);

    pstAlgs->alg_type =  ISP_ALG_EDGEAMRK;
    pstAlgs->alg_func.pfn_alg_init = ISP_HlcInit;
    pstAlgs->alg_func.pfn_alg_run  = ISP_HlcRun;
    pstAlgs->alg_func.pfn_alg_ctrl = ISP_HlcCtrl;
    pstAlgs->alg_func.pfn_alg_exit = ISP_HlcExit;
    pstAlgs->used = HI_TRUE;

    return HI_SUCCESS;
}
#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
