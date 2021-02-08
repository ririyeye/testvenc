/*
* Copyright (C) Hisilicon Technologies Co., Ltd. 2012-2019. All rights reserved.
* Description:
* Author: Hisilicon multimedia software group
* Create: 2011/06/28
*/


#include "mpi_sys.h"
#include "hi_comm_vi.h"
#include "hi_comm_isp.h"
#include "hi_comm_3a.h"
#include "hi_ae_comm.h"
#include "hi_awb_comm.h"
#include "isp_inner.h"
#include "isp_main.h"
#include "isp_vreg.h"
#include "isp_ext_config.h"

#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif
#endif /* End of #ifdef __cplusplus */

hi_void isp_calc_grid_info(hi_u16 wdith, hi_u16 start_pox, hi_u16 block_num, hi_u16 *grid_info)
{
    hi_u16 i;
    hi_u16 integer;
    hi_u16 remainder;

    integer   = wdith / DIV_0_TO_1(block_num);
    remainder = wdith % DIV_0_TO_1(block_num);
    grid_info[0] = start_pox;
    for (i = 1; i < block_num; i++) {
        if (remainder > 0) {
            grid_info[i] = grid_info[i - 1] + integer + 1;
            remainder = remainder - 1;
        } else {
            grid_info[i] = grid_info[i - 1] + integer;
        }
    }

    return;
}
hi_u32 isp_get_striping_active_img_start(hi_u8 block_index, isp_working_mode *isp_work_mode)
{
    hi_u32 over_lap;
    hi_u32 block_start;

    over_lap = isp_work_mode->over_lap;
    if (block_index == 0) {
        block_start = isp_work_mode->block_rect[block_index].x;
    } else {
        block_start = isp_work_mode->block_rect[block_index].x + over_lap;
    }

    return block_start;
}

hi_u32 isp_get_striping_active_img_width(hi_u8 block_index, isp_working_mode *isp_work_mode)
{
    hi_u32 block_width;
    hi_u32 over_lap;
    hi_u8   u8BlockNum;

    over_lap    = isp_work_mode->over_lap;
    block_width = isp_work_mode->block_rect[block_index].width;
    u8BlockNum    = isp_work_mode->block_num;

    if ((block_index == 0) || (block_index == (u8BlockNum - 1))) { // first block and last block
        block_width = block_width - over_lap;
    } else {
        block_width = block_width - over_lap * 2;
    }
    return block_width;
}

hi_u32 isp_get_striping_grid_x_info(hi_u16 *grid_pos, hi_u16 grid_num, isp_working_mode *isp_work_mode)
{
    hi_u8  i;
    hi_u16 start;
    hi_u16 width;
    hi_u16 div_num;
    hi_u16 index = 0;

    for (i = 0; i < isp_work_mode->block_num; i++) {
        start = isp_get_striping_active_img_start(i, isp_work_mode);
        width = isp_get_striping_active_img_width(i, isp_work_mode);

        if (i < grid_num % DIV_0_TO_1(isp_work_mode->block_num)) {
            div_num = grid_num / DIV_0_TO_1(isp_work_mode->block_num) + 1;
        } else {
            div_num = grid_num / DIV_0_TO_1(isp_work_mode->block_num);
        }

        isp_calc_grid_info(width, start, div_num, &(grid_pos[index]));
        index = index + div_num;
    }
    return HI_SUCCESS;
}

hi_s32 isp_get_ae_grid_info(VI_PIPE vi_pipe, hi_isp_ae_grid_info *fe_grid_info, hi_isp_ae_grid_info *be_grid_info)
{
    hi_bool crop_en = HI_FALSE;
    hi_u16  img_total_width, img_total_height;
    hi_u16  img_start_x, img_start_y;
    hi_u16  be_width, be_height;
    hi_u16  be_start_x = 0;
    hi_u16  be_start_y = 0;
    isp_working_mode isp_work_mode;

    memset(fe_grid_info, 0, sizeof(hi_isp_ae_grid_info));
    memset(be_grid_info, 0, sizeof(hi_isp_ae_grid_info));

    if (ioctl(g_as32IspFd[vi_pipe], ISP_WORK_MODE_GET, &isp_work_mode) != HI_SUCCESS) {
        ISP_ERR_TRACE("get work mode error!\n");
        return HI_FAILURE;
    }

    crop_en = hi_ext_system_ae_crop_en_read(vi_pipe);

    if (crop_en == HI_TRUE) {
        img_start_x      = hi_ext_system_ae_crop_x_read(vi_pipe);
        img_start_y      = hi_ext_system_ae_crop_y_read(vi_pipe);
        img_total_width  = hi_ext_system_ae_crop_width_read(vi_pipe);
        img_total_height = hi_ext_system_ae_crop_height_read(vi_pipe);
    } else {
        img_start_x      = 0;
        img_start_y      = 0;
        img_total_width  = hi_ext_sync_total_width_read(vi_pipe);
        img_total_height = hi_ext_sync_total_height_read(vi_pipe);
    }

    isp_calc_grid_info(img_total_width,  img_start_x, AE_ZONE_COLUMN, fe_grid_info->grid_x_pos);
    isp_calc_grid_info(img_total_height, img_start_y, AE_ZONE_ROW, fe_grid_info->grid_y_pos);

    fe_grid_info->grid_x_pos[AE_ZONE_COLUMN] = img_start_x + img_total_width - 1;
    fe_grid_info->grid_y_pos[AE_ZONE_ROW]    = img_start_y + img_total_height - 1;
    fe_grid_info->status = 1;

    if ((IS_STRIPING_MODE(isp_work_mode.running_mode)) ||
        (IS_SIDEBYSIDE_MODE(isp_work_mode.running_mode))) {
        isp_get_striping_grid_x_info(be_grid_info->grid_x_pos, AE_ZONE_COLUMN, &isp_work_mode);
        be_start_y   = isp_work_mode.block_rect[0].y;
        be_height   = isp_work_mode.frame_rect.height;
        isp_calc_grid_info(be_height, be_start_y, AE_ZONE_ROW, be_grid_info->grid_y_pos);
        be_width    = isp_work_mode.frame_rect.width;
    } else {
        if (crop_en == HI_TRUE) {
            be_start_x = hi_ext_system_ae_crop_x_read(vi_pipe);
            be_start_y = hi_ext_system_ae_crop_y_read(vi_pipe);
            be_width  = hi_ext_system_ae_crop_width_read(vi_pipe);
            be_height = hi_ext_system_ae_crop_height_read(vi_pipe);
        } else {
            be_start_x = 0;
            be_start_y = 0;
            be_width  = isp_work_mode.frame_rect.width;
            be_height = isp_work_mode.frame_rect.height;
        }

        isp_calc_grid_info(be_width,  be_start_x, AE_ZONE_COLUMN, be_grid_info->grid_x_pos);
        isp_calc_grid_info(be_height, be_start_y, AE_ZONE_ROW, be_grid_info->grid_y_pos);
    }

    be_grid_info->grid_x_pos[AE_ZONE_COLUMN] = be_start_x + be_width  - 1; // last position
    be_grid_info->grid_y_pos[AE_ZONE_ROW]    = be_start_y + be_height - 1; // last position

    be_grid_info->status = 1;

    return HI_SUCCESS;
}

hi_s32 isp_get_mg_grid_info(VI_PIPE vi_pipe, hi_isp_mg_grid_info *grid_info)
{
    hi_bool crop_en = HI_FALSE;
    hi_u16  be_width, be_height;
    hi_u16  be_start_x = 0;
    hi_u16  be_start_y = 0;
    isp_working_mode isp_work_mode;

    memset(grid_info, 0, sizeof(hi_isp_mg_grid_info));

    if (ioctl(g_as32IspFd[vi_pipe], ISP_WORK_MODE_GET, &isp_work_mode) != HI_SUCCESS) {
        ISP_ERR_TRACE("get work mode error!\n");
        return HI_FAILURE;
    }

    crop_en = hi_ext_system_ae_crop_en_read(vi_pipe);

    if ((IS_STRIPING_MODE(isp_work_mode.running_mode)) ||
        (IS_SIDEBYSIDE_MODE(isp_work_mode.running_mode))) {
        isp_get_striping_grid_x_info(grid_info->grid_x_pos, MG_ZONE_COLUMN, &isp_work_mode);
        be_start_y   = isp_work_mode.block_rect[0].y;
        be_height   = isp_work_mode.frame_rect.height;
        isp_calc_grid_info(be_height, be_start_y, MG_ZONE_ROW, grid_info->grid_y_pos);
        be_width    = isp_work_mode.frame_rect.width;
    } else {
        if (crop_en == HI_TRUE) {
            be_start_x = hi_ext_system_ae_crop_x_read(vi_pipe);
            be_start_y = hi_ext_system_ae_crop_y_read(vi_pipe);
            be_width  = hi_ext_system_ae_crop_width_read(vi_pipe);
            be_height = hi_ext_system_ae_crop_height_read(vi_pipe);
        } else {
            be_start_x = 0;
            be_start_y = 0;
            be_width  = isp_work_mode.frame_rect.width;
            be_height = isp_work_mode.frame_rect.height;
        }

        isp_calc_grid_info(be_width,  be_start_x, MG_ZONE_COLUMN, grid_info->grid_x_pos);
        isp_calc_grid_info(be_height, be_start_y, MG_ZONE_ROW,    grid_info->grid_y_pos);

    }

    grid_info->grid_x_pos[MG_ZONE_COLUMN] = be_start_x + be_width  - 1; // last position
    grid_info->grid_y_pos[MG_ZONE_ROW]    = be_start_y + be_height - 1; // last position
    grid_info->status                     = 1;

    return HI_SUCCESS;
}

hi_s32 isp_get_af_grid_info(VI_PIPE vi_pipe, hi_isp_focus_grid_info *fe_grid_info, hi_isp_focus_grid_info *be_grid_info)
{
    hi_bool crop_en = HI_FALSE;
    hi_u16  be_width, be_height;
    hi_u16  be_start_x = 0;
    hi_u16  be_start_y = 0;
    hi_u16  af_x_grid_num, af_y_grid_num;
    isp_working_mode isp_work_mode;

    memset(fe_grid_info, 0, sizeof(hi_isp_focus_grid_info));
    memset(be_grid_info, 0, sizeof(hi_isp_focus_grid_info));

    if (ioctl(g_as32IspFd[vi_pipe], ISP_WORK_MODE_GET, &isp_work_mode) != HI_SUCCESS) {
        ISP_ERR_TRACE("get work mode error!\n");
        return HI_FAILURE;
    }

    crop_en = hi_ext_af_crop_enable_read(vi_pipe);
    af_y_grid_num = hi_ext_af_window_vnum_read(vi_pipe);
    af_x_grid_num = hi_ext_af_window_hnum_read(vi_pipe);

    if ((IS_STRIPING_MODE(isp_work_mode.running_mode)) ||
        (IS_SIDEBYSIDE_MODE(isp_work_mode.running_mode))) {
        isp_get_striping_grid_x_info(be_grid_info->grid_x_pos, af_x_grid_num, &isp_work_mode);
        be_start_y = isp_work_mode.block_rect[0].y;
        be_height = isp_work_mode.frame_rect.height;
        isp_calc_grid_info(be_height, be_start_y, af_y_grid_num, be_grid_info->grid_y_pos);
        be_width  = isp_work_mode.frame_rect.width;
    } else {
        if (crop_en == HI_TRUE) {
            be_start_x = hi_ext_af_crop_pos_x_read(vi_pipe);
            be_start_y = hi_ext_af_crop_pos_y_read(vi_pipe);
            be_width  = hi_ext_af_crop_hsize_read(vi_pipe);
            be_height = hi_ext_af_crop_vsize_read(vi_pipe);
        } else {
            be_start_x = 0;
            be_start_y = 0;
            be_width  = isp_work_mode.frame_rect.width;
            be_height = isp_work_mode.frame_rect.height;
        }

        isp_calc_grid_info(be_width,  be_start_x, af_x_grid_num, be_grid_info->grid_x_pos);
        isp_calc_grid_info(be_height, be_start_y, af_y_grid_num,    be_grid_info->grid_y_pos);

    }

    be_grid_info->grid_x_pos[af_x_grid_num] = be_start_x + be_width  - 1; // last position
    be_grid_info->grid_y_pos[af_y_grid_num] = be_start_y + be_height - 1; // last position
    be_grid_info->status = 1;

    return HI_SUCCESS;
}

hi_s32 isp_get_wb_grid_info(VI_PIPE vi_pipe, hi_isp_awb_grid_info *grid_info)
{
    hi_bool crop_en = HI_FALSE;
    hi_u16  be_width, be_height;
    hi_u16  be_start_x = 0;
    hi_u16  be_start_y = 0;
    hi_u16  u16awb_x_grid_num, u16awb_y_grid_num;
    isp_working_mode isp_work_mode;

    memset(grid_info, 0, sizeof(hi_isp_awb_grid_info));

    if (ioctl(g_as32IspFd[vi_pipe], ISP_WORK_MODE_GET, &isp_work_mode) != HI_SUCCESS) {
        ISP_ERR_TRACE("get work mode error!\n");
        return HI_FAILURE;
    }

    u16awb_y_grid_num = hi_ext_system_awb_vnum_read(vi_pipe);
    u16awb_x_grid_num = hi_ext_system_awb_hnum_read(vi_pipe);
    crop_en        = hi_ext_system_awb_crop_en_read(vi_pipe);

    if ((IS_STRIPING_MODE(isp_work_mode.running_mode)) ||
        (IS_SIDEBYSIDE_MODE(isp_work_mode.running_mode))) {
        isp_get_striping_grid_x_info(grid_info->grid_x_pos, u16awb_x_grid_num, &isp_work_mode);
        be_start_y = isp_work_mode.block_rect[0].y;
        be_height = isp_work_mode.frame_rect.height;
        isp_calc_grid_info(be_height, be_start_y, u16awb_y_grid_num, grid_info->grid_y_pos);
        be_width    = isp_work_mode.frame_rect.width;
    } else {
        if (crop_en == HI_TRUE) {
            be_start_x = hi_ext_system_awb_crop_x_read(vi_pipe);
            be_start_y = hi_ext_system_awb_crop_y_read(vi_pipe);
            be_width  = hi_ext_system_awb_crop_width_read(vi_pipe);
            be_height = hi_ext_system_awb_crop_height_read(vi_pipe);
        } else {
            be_start_x = 0;
            be_start_y = 0;
            be_width  = isp_work_mode.frame_rect.width;
            be_height = isp_work_mode.frame_rect.height;
        }

        isp_calc_grid_info(be_width, be_start_x, u16awb_x_grid_num, grid_info->grid_x_pos);
        isp_calc_grid_info(be_height, be_start_y, u16awb_y_grid_num, grid_info->grid_y_pos);
    }

    grid_info->grid_x_pos[u16awb_x_grid_num] = be_start_x + be_width  - 1; // last position
    grid_info->grid_y_pos[u16awb_y_grid_num] = be_start_y + be_height - 1; // last position
    grid_info->status                     = 1;

    return HI_SUCCESS;
}

hi_s32 isp_get_ae_stitch_statistics(VI_PIPE vi_pipe, hi_isp_ae_stitch_statistics *ae_stitch_stat)
{
    return HI_ERR_ISP_NOT_SUPPORT;
}

hi_s32 isp_get_wb_stitch_statistics(VI_PIPE vi_pipe, hi_isp_wb_stitch_statistics *stitch_wb_stat)
{
    return HI_ERR_ISP_NOT_SUPPORT;
}

hi_s32 isp_get_fe_focus_statistics(VI_PIPE vi_pipe, hi_isp_fe_focus_statistics *fe_af_stat, isp_stat *isp_act_stat, hi_u8 wdr_chn)
{
    memset(fe_af_stat, 0, sizeof(hi_isp_fe_focus_statistics));

    return HI_SUCCESS;
}

hi_s32 isp_set_radial_shading_attr(VI_PIPE vi_pipe, const hi_isp_radial_shading_attr *radial_shading_attr)
{
    ISP_ERR_TRACE("Not support this interface!\n");
    return HI_ERR_ISP_NOT_SUPPORT;
}

hi_s32 isp_get_radial_shading_attr(VI_PIPE vi_pipe, hi_isp_radial_shading_attr *radial_shading_attr)
{
    ISP_ERR_TRACE("Not support this interface!\n");
    return HI_ERR_ISP_NOT_SUPPORT;
}

hi_s32 isp_set_radial_shading_lut(VI_PIPE vi_pipe, const hi_isp_radial_shading_lut_attr *radial_shading_lut_attr)
{
    ISP_ERR_TRACE("Not support this interface!\n");
    return HI_ERR_ISP_NOT_SUPPORT;
}

hi_s32 isp_get_radial_shading_lut(VI_PIPE vi_pipe, hi_isp_radial_shading_lut_attr *radial_shading_lut_attr)
{
    ISP_ERR_TRACE("Not support this interface!\n");
    return HI_ERR_ISP_NOT_SUPPORT;
}

hi_s32 isp_set_pipe_differ_attr(VI_PIPE vi_pipe, const hi_isp_pipe_diff_attr *pipe_differ)
{
    ISP_ERR_TRACE("Not support this interface!\n");
    return HI_ERR_ISP_NOT_SUPPORT;
}

hi_s32 isp_get_pipe_differ_attr(VI_PIPE vi_pipe, hi_isp_pipe_diff_attr *pipe_differ)
{
    ISP_ERR_TRACE("Not support this interface!\n");
    return HI_ERR_ISP_NOT_SUPPORT;
}

hi_s32 isp_set_rc_attr(VI_PIPE vi_pipe, const hi_isp_rc_attr *rc_attr)
{
    ISP_ERR_TRACE("Not support this interface!\n");
    return HI_ERR_ISP_NOT_SUPPORT;
}

hi_s32 isp_get_rc_attr(VI_PIPE vi_pipe, hi_isp_rc_attr *rc_attr)
{
    ISP_ERR_TRACE("Not support this interface!\n");
    return HI_ERR_ISP_NOT_SUPPORT;
}

hi_s32 isp_set_rgbir_attr(VI_PIPE vi_pipe, const hi_isp_rgbir_attr *rgbir_attr)
{
    ISP_ERR_TRACE("Not support this interface!\n");
    return HI_ERR_ISP_NOT_SUPPORT;
}

hi_s32 isp_get_rgbir_attr(VI_PIPE vi_pipe, hi_isp_rgbir_attr *rgbir_attr)
{
    ISP_ERR_TRACE("Not support this interface!\n");
    return HI_ERR_ISP_NOT_SUPPORT;
}

hi_s32 isp_set_pre_log_lut_attr(VI_PIPE vi_pipe, const hi_isp_preloglut_attr *pre_log_lut_attr)
{
    ISP_ERR_TRACE("Not support this interface!\n");
    return HI_ERR_ISP_NOT_SUPPORT;
}

hi_s32 isp_get_pre_log_lut_attr(VI_PIPE vi_pipe, hi_isp_preloglut_attr *pre_log_lut_attr)
{
    ISP_ERR_TRACE("Not support this interface!\n");
    return HI_ERR_ISP_NOT_SUPPORT;
}

hi_s32 isp_set_log_lut_attr(VI_PIPE vi_pipe, const hi_isp_loglut_attr *log_lut_attr)
{
    ISP_ERR_TRACE("Not support this interface!\n");
    return HI_ERR_ISP_NOT_SUPPORT;
}

hi_s32 isp_get_log_lut_attr(VI_PIPE vi_pipe, hi_isp_loglut_attr *log_lut_attr)
{
    ISP_ERR_TRACE("Not support this interface!\n");
    return HI_ERR_ISP_NOT_SUPPORT;
}

hi_s32 isp_set_clut_coeff(VI_PIPE vi_pipe, const hi_isp_clut_lut *clut_lut)
{
    hi_u32         *vir_addr = HI_NULL;
    isp_mmz_buf_ex clut_buf;

    ISP_CHECK_PIPE(vi_pipe);
    ISP_CHECK_POINTER(clut_lut);
    ISP_CHECK_OPEN(vi_pipe);
    ISP_CHECK_MEM_INIT(vi_pipe);

    if (ioctl(g_as32IspFd[vi_pipe], ISP_CLUT_BUF_GET, &clut_buf.phy_addr) != HI_SUCCESS) {
        ISP_ERR_TRACE("get clut buffer err\n");
        return HI_ERR_ISP_NOMEM;
    }

    clut_buf.vir_addr = HI_MPI_SYS_Mmap(clut_buf.phy_addr, HI_ISP_CLUT_LUT_LENGTH * sizeof(hi_u32));

    if (clut_buf.vir_addr == HI_NULL) {
        return HI_ERR_ISP_NULL_PTR;
    }

    vir_addr = (hi_u32 *)clut_buf.vir_addr;

    memcpy(vir_addr, clut_lut->lut, HI_ISP_CLUT_LUT_LENGTH * sizeof(hi_u32));

    hi_ext_system_clut_lut_update_en_write(vi_pipe, HI_TRUE);

    HI_MPI_SYS_Munmap(clut_buf.vir_addr, HI_ISP_CLUT_LUT_LENGTH * sizeof(hi_u32));

    return HI_SUCCESS;
}

hi_s32 isp_get_clut_coeff(VI_PIPE vi_pipe, hi_isp_clut_lut *clut_lut)
{
    hi_u32         *vir_addr = HI_NULL;
    isp_mmz_buf_ex clut_buf;

    ISP_CHECK_PIPE(vi_pipe);
    ISP_CHECK_POINTER(clut_lut);
    ISP_CHECK_OPEN(vi_pipe);
    ISP_CHECK_MEM_INIT(vi_pipe);

    if (ioctl(g_as32IspFd[vi_pipe], ISP_CLUT_BUF_GET, &clut_buf.phy_addr) != HI_SUCCESS) {
        ISP_ERR_TRACE("get clut buffer err\n");
        return HI_ERR_ISP_NOMEM;
    }

    clut_buf.vir_addr = HI_MPI_SYS_Mmap(clut_buf.phy_addr, HI_ISP_CLUT_LUT_LENGTH * sizeof(hi_u32));

    if (clut_buf.vir_addr == HI_NULL) {
        return HI_ERR_ISP_NULL_PTR;
    }

    vir_addr = (hi_u32 *)clut_buf.vir_addr;

    memcpy(clut_lut->lut, vir_addr, HI_ISP_CLUT_LUT_LENGTH * sizeof(hi_u32));

    HI_MPI_SYS_Munmap(clut_buf.vir_addr, HI_ISP_CLUT_LUT_LENGTH * sizeof(hi_u32));

    return HI_SUCCESS;
}

hi_s32 isp_set_clut_attr(VI_PIPE vi_pipe, const hi_isp_clut_attr *clut_attr)
{
    ISP_CHECK_PIPE(vi_pipe);
    ISP_CHECK_POINTER(clut_attr);
    ISP_CHECK_BOOL(clut_attr->enable);
    ISP_CHECK_OPEN(vi_pipe);
    ISP_CHECK_MEM_INIT(vi_pipe);

    if (clut_attr->gain_r > 4095) {
        ISP_ERR_TRACE("Invalid gain_r!\n");
        return HI_ERR_ISP_ILLEGAL_PARAM;
    }
    if (clut_attr->gain_g > 4095) {
        ISP_ERR_TRACE("Invalid gain_g!\n");
        return HI_ERR_ISP_ILLEGAL_PARAM;
    }
    if (clut_attr->gain_b > 4095) {
        ISP_ERR_TRACE("Invalid gain_b!\n");
        return HI_ERR_ISP_ILLEGAL_PARAM;
    }

    hi_ext_system_clut_en_write(vi_pipe, clut_attr->enable);
    hi_ext_system_clut_gainR_write(vi_pipe, clut_attr->gain_r);
    hi_ext_system_clut_gainG_write(vi_pipe, clut_attr->gain_g);
    hi_ext_system_clut_gainB_write(vi_pipe, clut_attr->gain_b);
    hi_ext_system_clut_ctrl_update_en_write(vi_pipe, HI_TRUE);

    return HI_SUCCESS;
}
hi_s32 isp_get_clut_attr(VI_PIPE vi_pipe, hi_isp_clut_attr *clut_attr)
{
    ISP_CHECK_PIPE(vi_pipe);
    ISP_CHECK_POINTER(clut_attr);
    ISP_CHECK_OPEN(vi_pipe);
    ISP_CHECK_MEM_INIT(vi_pipe);

    clut_attr->enable = hi_ext_system_clut_en_read(vi_pipe);
    clut_attr->gain_r = hi_ext_system_clut_gainR_read(vi_pipe);
    clut_attr->gain_g = hi_ext_system_clut_gainG_read(vi_pipe);
    clut_attr->gain_b = hi_ext_system_clut_gainB_read(vi_pipe);

    return HI_SUCCESS;
}

hi_s32 isp_set_raw_pos(VI_PIPE vi_pipe, const hi_isp_raw_pos_attr *raw_pos_attr)
{
    return HI_ERR_ISP_NOT_SUPPORT;
}

hi_s32 isp_get_raw_pos(VI_PIPE vi_pipe, hi_isp_raw_pos_attr *raw_pos_attr)
{
    return HI_ERR_ISP_NOT_SUPPORT;
}

hi_s32 isp_calc_flicker_type(VI_PIPE ViPipe, ISP_CALCFLICKER_INPUT_S *pstInputParam,ISP_CALCFLICKER_OUTPUT_S *pstOutputParam, VIDEO_FRAME_INFO_S stFrame[], HI_U32 u32ArraySize)
{
    HI_U32 i = 0;

    if (u32ArraySize != 3){
        ISP_ERR_TRACE("Frame Number is not 3 Frame\n");
        return HI_ERR_ISP_ILLEGAL_PARAM;
    }

    ISP_CHECK_PIPE(ViPipe);
    ISP_CHECK_OPEN(ViPipe);
    ISP_CHECK_MEM_INIT(ViPipe);

    ISP_CHECK_POINTER(pstInputParam);
    ISP_CHECK_POINTER(pstOutputParam);


    if (pstInputParam->u32LinesPerSecond < 500){
        ISP_ERR_TRACE("LinePerSecond is out of range\n");
        return HI_ERR_ISP_ILLEGAL_PARAM;
    }

    for (i = 0; i < u32ArraySize; i++){
        ISP_CHECK_POINTER(stFrame+i);

        if (0 == stFrame[i].stVFrame.u64PhyAddr[0]){
            ISP_ERR_TRACE("The Phy Address Error!!!\n");
            return HI_ERR_ISP_ILLEGAL_PARAM;
        }

        if ((stFrame[i].stVFrame.u32Width < RES_WIDTH_MIN) || (stFrame[i].stVFrame.u32Width > RES_WIDTH_MAX(ViPipe))){
            ISP_ERR_TRACE("The Image width is out of range!!!\n");
            return HI_ERR_ISP_ILLEGAL_PARAM;
        }

        if ((stFrame[i].stVFrame.u32Height < RES_HEIGHT_MIN) || (stFrame[i].stVFrame.u32Height > RES_HEIGHT_MAX(ViPipe))){
            ISP_ERR_TRACE("The Image height is out of range!!!\n");
            return HI_ERR_ISP_ILLEGAL_PARAM;
        }

        if (i != 0){
            if ((stFrame[i].stVFrame.u32TimeRef - stFrame[i-1].stVFrame.u32TimeRef) != 2){
                ISP_ERR_TRACE("The Frames is not continuity!!!\n");
                return HI_ERR_ISP_ILLEGAL_PARAM;
            }
        }
    }

    return ISP_CalcFlickerType(pstInputParam, pstOutputParam, stFrame, u32ArraySize);
}

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* End of #ifdef __cplusplus */
