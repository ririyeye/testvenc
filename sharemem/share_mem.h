#ifndef __SHARE_MEM_H__
#define __SHARE_MEM_H__

#if __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "hi_comm_venc.h"

struct DATA_BLOCK_HANDLE {
	void *pdat;
	int maxcnt;
	int pointcnt;
};

enum DAT_BLOCK_FLG{
	DAT_BLOCK_CLEAN = 1,
};


int get_buff_handle(struct DATA_BLOCK_HANDLE *hdl, enum DAT_BLOCK_FLG flg);

int put_stream_buff(struct DATA_BLOCK_HANDLE *hdl, VENC_STREAM_S *pstStream);

#if __cplusplus
}
#endif

#endif
