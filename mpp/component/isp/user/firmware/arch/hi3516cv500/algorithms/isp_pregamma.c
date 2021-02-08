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
#include "isp_proc.h"
#include "isp_ext_config.h"
#include "hi_math.h"
#include "isp_math_utils.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

static const HI_U32 au32PreGAMMA[PREGAMMA_NODE_NUM] = {
    0, 256, 512, 768, 1024, 1280, 1536, 1792, 2048, 2304, 2560, 2816, 3072, 3328, 3584, 3840,
    4096, 4352, 4608, 4864, 5120, 5376, 5632, 5888, 6144, 6400, 6656, 6912, 7168, 7424, 7680,
    7936, 8192, 8448, 8704, 8960, 9216, 9472, 9728, 9984, 10240, 10496, 10752, 11008, 11264,
    11520, 11776, 12032, 12288, 12544, 12800, 13056, 13312, 13568, 13824, 14080, 14336, 14592,
    14848, 15104, 15360, 15616, 15872, 16128, 16384, 16896, 17408, 17920, 18432, 18944, 19456,
    19968, 20480, 20992, 21504, 22016, 22528, 23040, 23552, 24064, 24576, 25088, 25600, 26112,
    26624, 27136, 27648, 28160, 28672, 29184, 29696, 30208, 30720, 31232, 31744, 32256, 32768,
    33792, 34816, 35840, 36864, 37888, 38912, 39936, 40960, 41984, 43008, 44032, 45056, 46080,
    47104, 48128, 49152, 50176, 51200, 52224, 53248, 54272, 55296, 56320, 57344, 58368, 59392,
    60416, 61440, 62464, 63488, 64512, 65536, 67584, 69632, 71680, 73728, 75776, 77824, 79872,
    81920, 83968, 86016, 88064, 90112, 92160, 94208, 96256, 98304, 100352, 102400, 104448, 106496,
    108544, 110592, 112640, 114688, 116736, 118784, 120832, 122880, 124928, 126976, 129024, 131072,
    135168, 139264, 143360, 147456, 151552, 155648, 159744, 163840, 167936, 172032, 176128, 180224,
    184320, 188416, 192512, 196608, 200704, 204800, 208896, 212992, 217088, 221184, 225280, 229376,
    233472, 237568, 241664, 245760, 249856, 253952, 258048, 262144, 270336, 278528, 286720, 294912,
    303104, 311296, 319488, 327680, 335872, 344064, 352256, 360448, 368640, 376832, 385024, 393216,
    401408, 409600, 417792, 425984, 434176, 442368, 450560, 458752, 466944, 475136, 483328, 491520,
    499712, 507904, 516096, 524288, 540672, 557056, 573440, 589824, 606208, 622592, 638976, 655360,
    671744, 688128, 704512, 720896, 737280, 753664, 770048, 786432, 802816, 819200, 835584, 851968,
    868352, 884736, 901120, 917504, 933888, 950272, 966656, 983040, 999424, 1015808, 1032192, 1048575
};

static const HI_U8 g_au8PregammaSegIdxBase[PREGAMMA_SEG_NUM] = {0, 0, 32, 64, 96, 128, 160, 192};
static const HI_U8 g_au8PregammaSegMaxVal[PREGAMMA_SEG_NUM]  = {0, 2,  4,  8, 16,  32,  64, 128};

typedef struct hiISP_PREGAMMA_S {
    HI_BOOL bEnable;
    HI_BOOL bLutUpdate;
} ISP_PREGAMMA_S;

ISP_PREGAMMA_S g_astPreGammaCtx[ISP_MAX_PIPE_NUM] = {{0}};
#define PREGAMMA_GET_CTX(dev, pstCtx)   pstCtx = &g_astPreGammaCtx[dev]

static HI_S32 PreGammaCheckCmosParam(VI_PIPE ViPipe, const hi_isp_cmos_pregamma *pre_gamma)
{
    hi_u16 i;

    ISP_CHECK_BOOL(pre_gamma->enable);

    for (i = 0; i < PREGAMMA_NODE_NUM; i++) {
        if (pre_gamma->pregamma_lut[i] > HI_ISP_PREGAMMA_LUT_MAX) {
            ISP_ERR_TRACE("Invalid au32PreGamma[%d]!\n", i);
            return HI_ERR_ISP_ILLEGAL_PARAM;
        }
    }

    return HI_SUCCESS;
}

static HI_S32 PreGammaRegsInitialize(VI_PIPE ViPipe, isp_reg_cfg *pstRegCfg)
{
    HI_BOOL bIsOfflineMode;
    HI_U8  u8BlockNum;
    HI_U16 i;
    HI_S32 s32Ret;
    const HI_U32           *pau32PreGAMMA  = HI_NULL;
    ISP_PREGAMMA_REG_CFG_S *pstPreGammaCfg = HI_NULL;
    ISP_PREGAMMA_S         *pstPreGammaCtx = HI_NULL;
    isp_usr_ctx              *pstIspCtx      = HI_NULL;
    hi_isp_cmos_default     *sns_dft      = HI_NULL;

    isp_sensor_get_default(ViPipe, &sns_dft);
    ISP_GET_CTX(ViPipe, pstIspCtx);
    PREGAMMA_GET_CTX(ViPipe, pstPreGammaCtx);

    bIsOfflineMode = (IS_OFFLINE_MODE(pstIspCtx->block_attr.running_mode) ||
                      IS_STRIPING_MODE(pstIspCtx->block_attr.running_mode));

    pstPreGammaCtx->bLutUpdate = HI_FALSE;

    u8BlockNum = pstIspCtx->block_attr.block_num;

    /* Read from CMOS */
    if (sns_dft->key.bit1_pregamma) {
        ISP_CHECK_POINTER(sns_dft->pregamma);

        s32Ret = PreGammaCheckCmosParam(ViPipe, sns_dft->pregamma);
        if (s32Ret != HI_SUCCESS) {
            return s32Ret;
        }

        pau32PreGAMMA              = sns_dft->pregamma->pregamma_lut;
        pstPreGammaCtx->bEnable    = sns_dft->pregamma->enable;
    } else {
        pau32PreGAMMA           = au32PreGAMMA;
        pstPreGammaCtx->bEnable =  HI_FALSE;
    }

    for (i = 0; i < PREGAMMA_NODE_NUM; i++) {
        hi_ext_system_pregamma_lut_write(ViPipe, i, pau32PreGAMMA[i]);
    }

    for (i = 0; i < u8BlockNum; i++) {
        pstPreGammaCfg = &pstRegCfg->alg_reg_cfg[i].stPreGammaCfg;

        /* Static */
        pstPreGammaCfg->stStaticRegCfg.u8BitDepthIn  = 20;
        pstPreGammaCfg->stStaticRegCfg.u8BitDepthOut = 20;
        pstPreGammaCfg->stStaticRegCfg.bStaticResh   = HI_TRUE;
        memcpy(pstPreGammaCfg->stStaticRegCfg.au8SegIdxBase, g_au8PregammaSegIdxBase, PREGAMMA_SEG_NUM * sizeof(HI_U8));
        memcpy(pstPreGammaCfg->stStaticRegCfg.au8SegMaxVal,  g_au8PregammaSegMaxVal,  PREGAMMA_SEG_NUM * sizeof(HI_U8));

        /* Dynamic */
        // Enable Gamma
        pstPreGammaCfg->bPreGammaEn                       = bIsOfflineMode ? (pstPreGammaCtx->bEnable) : (HI_FALSE);
        pstPreGammaCfg->stDynaRegCfg.bPreGammaLutUpdateEn = HI_TRUE;
        pstPreGammaCfg->stDynaRegCfg.u32UpdateIndex       = 1;
        pstPreGammaCfg->stDynaRegCfg.u8BufId              = 0;
        memcpy(pstPreGammaCfg->stDynaRegCfg.u32PreGammaLUT, pau32PreGAMMA, PREGAMMA_NODE_NUM * sizeof(HI_U32));
    }

    pstRegCfg->cfg_key.bit1PreGammaCfg = 1;

    return HI_SUCCESS;
}

static HI_VOID PreGammaExtRegsInitialize(VI_PIPE ViPipe)
{
    ISP_PREGAMMA_S         *pstPreGammaCtx = HI_NULL;

    PREGAMMA_GET_CTX(ViPipe, pstPreGammaCtx);

    hi_ext_system_pregamma_en_write(ViPipe, pstPreGammaCtx->bEnable);
    hi_ext_system_pregamma_lut_update_write(ViPipe, HI_FALSE);
}

HI_S32 ISP_PreGammaInit(VI_PIPE ViPipe, HI_VOID *pRegCfg)
{
    HI_S32 s32Ret;
    isp_reg_cfg *pstRegCfg = (isp_reg_cfg *)pRegCfg;

    s32Ret = PreGammaRegsInitialize(ViPipe, pstRegCfg);
    if (s32Ret != HI_SUCCESS) {
        return s32Ret;
    }

    PreGammaExtRegsInitialize(ViPipe);

    return HI_SUCCESS;
}

static HI_VOID ISP_PreGammaWdrModeSet(VI_PIPE ViPipe, HI_VOID *pRegCfg)
{
    HI_U8  i;
    HI_U32 au32UpdateIdx[ISP_STRIPING_MAX_NUM] = {0};
    isp_reg_cfg *pstRegCfg = (isp_reg_cfg *)pRegCfg;

    for (i = 0; i < pstRegCfg->cfg_num; i++) {
        au32UpdateIdx[i] = pstRegCfg->alg_reg_cfg[i].stPreGammaCfg.stDynaRegCfg.u32UpdateIndex;
    }

    ISP_PreGammaInit(ViPipe, pRegCfg);

    for (i = 0; i < pstRegCfg->cfg_num; i++) {
        pstRegCfg->alg_reg_cfg[i].stPreGammaCfg.stDynaRegCfg.u32UpdateIndex = au32UpdateIdx[i] + 1;
        pstRegCfg->alg_reg_cfg[i].stPreGammaCfg.stDynaRegCfg.bSwitchMode    = HI_TRUE;
    }
}

static HI_S32 PreGammaReadExtRegs(VI_PIPE ViPipe)
{
    ISP_PREGAMMA_S *pstPreGammaCtx = HI_NULL;

    PREGAMMA_GET_CTX(ViPipe, pstPreGammaCtx);

    pstPreGammaCtx->bLutUpdate = hi_ext_system_pregamma_lut_update_read(ViPipe);
    hi_ext_system_pregamma_lut_update_write(ViPipe, HI_FALSE);

    return HI_SUCCESS;
}

HI_S32 PreGammaProcWrite(VI_PIPE ViPipe, hi_isp_ctrl_proc_write *pstProc)
{
    hi_isp_ctrl_proc_write stProcTmp;
    ISP_PREGAMMA_S *pstPreGamma = HI_NULL;

    PREGAMMA_GET_CTX(ViPipe, pstPreGamma);

    if ((pstProc->proc_buff == HI_NULL)
        || (pstProc->buff_len == 0)) {
        return HI_FAILURE;
    }

    stProcTmp.proc_buff = pstProc->proc_buff;
    stProcTmp.buff_len = pstProc->buff_len;

    ISP_PROC_PRINTF(&stProcTmp, pstProc->write_len,
                    "-----PreGamma INFO--------------------------------------------------------------\n");

    ISP_PROC_PRINTF(&stProcTmp, pstProc->write_len, "%16s\n", "Enable");

    ISP_PROC_PRINTF(&stProcTmp, pstProc->write_len, "%16u\n", pstPreGamma->bEnable);

    pstProc->write_len += 1;

    return HI_SUCCESS;
}

static HI_BOOL __inline CheckPreGammaOpen(ISP_PREGAMMA_S *pstPreGamma)
{
    return (pstPreGamma->bEnable == HI_TRUE);
}

HI_S32 ISP_PreGammaRun(VI_PIPE ViPipe, const HI_VOID *pStatInfo,
                       HI_VOID *pRegCfg, HI_S32 s32Rsv)
{
    HI_U16 i, j;
    ISP_PREGAMMA_S *pstPreGammaCtx = HI_NULL;
    isp_reg_cfg  *pstReg         = (isp_reg_cfg *)pRegCfg;
    ISP_PREGAMMA_DYNA_CFG_S *pstDynaRegCfg = HI_NULL;

    PREGAMMA_GET_CTX(ViPipe, pstPreGammaCtx);

    pstPreGammaCtx->bEnable = hi_ext_system_pregamma_en_read(ViPipe);

    for (i = 0; i < pstReg->cfg_num; i++) {
        pstReg->alg_reg_cfg[i].stPreGammaCfg.bPreGammaEn = pstPreGammaCtx->bEnable;
    }

    pstReg->cfg_key.bit1PreGammaCfg = 1;

    /* check hardware setting */
    if (!CheckPreGammaOpen(pstPreGammaCtx)) {
        return HI_SUCCESS;
    }

    PreGammaReadExtRegs(ViPipe);

    if (pstPreGammaCtx->bLutUpdate) {
        for (i = 0; i < pstReg->cfg_num; i++) {
            pstDynaRegCfg = &pstReg->alg_reg_cfg[i].stPreGammaCfg.stDynaRegCfg;
            for (j = 0; j < PREGAMMA_NODE_NUM; j++) {
                pstDynaRegCfg->u32PreGammaLUT[j] = hi_ext_system_pregamma_lut_read(ViPipe, j);
            }

            pstDynaRegCfg->bPreGammaLutUpdateEn = HI_TRUE;
            pstDynaRegCfg->u32UpdateIndex      += 1;
        }
    }

    return HI_SUCCESS;
}

HI_S32 ISP_PreGammaCtrl(VI_PIPE ViPipe, HI_U32 u32Cmd, HI_VOID *pValue)
{
    isp_reg_cfg_attr  *pRegCfg   = HI_NULL;

    switch (u32Cmd) {
        case ISP_WDR_MODE_SET :
            ISP_REGCFG_GET_CTX(ViPipe, pRegCfg);
            ISP_CHECK_POINTER(pRegCfg);
            ISP_PreGammaWdrModeSet(ViPipe, (HI_VOID *)&pRegCfg->reg_cfg);
            break;
        case ISP_PROC_WRITE:
            PreGammaProcWrite(ViPipe, (hi_isp_ctrl_proc_write *)pValue);
            break;
        default :
            break;
    }

    return HI_SUCCESS;
}

HI_S32 ISP_PreGammaExit(VI_PIPE ViPipe)
{
    HI_U8 i;
    isp_reg_cfg_attr *pRegCfg   = HI_NULL;

    ISP_REGCFG_GET_CTX(ViPipe, pRegCfg);

    for (i = 0; i < pRegCfg->reg_cfg.cfg_num; i++) {
        pRegCfg->reg_cfg.alg_reg_cfg[i].stPreGammaCfg.bPreGammaEn = HI_FALSE;
    }

    pRegCfg->reg_cfg.cfg_key.bit1PreGammaCfg = 1;
    return HI_SUCCESS;
}


HI_S32 isp_alg_register_pre_gamma(VI_PIPE ViPipe)
{
    isp_usr_ctx *pstIspCtx = HI_NULL;
    isp_alg_node *pstAlgs = HI_NULL;

    ISP_GET_CTX(ViPipe, pstIspCtx);
    ISP_ALG_CHECK(pstIspCtx->alg_key.bit1_pre_gamma);
    pstAlgs = ISP_SearchAlg(pstIspCtx->algs);
    ISP_CHECK_POINTER(pstAlgs);

    pstAlgs->alg_type = ISP_ALG_PREGAMMA;
    pstAlgs->alg_func.pfn_alg_init = ISP_PreGammaInit;
    pstAlgs->alg_func.pfn_alg_run  = ISP_PreGammaRun;
    pstAlgs->alg_func.pfn_alg_ctrl = ISP_PreGammaCtrl;
    pstAlgs->alg_func.pfn_alg_exit = ISP_PreGammaExit;
    pstAlgs->used = HI_TRUE;

    return HI_SUCCESS;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
