#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <errno.h>
#include "share_mem.h"
#include <string.h>
#include <stdio.h>

#define MAX_PACK_CNT 256
struct DATA_BLOCK {
	union {
		unsigned char header[64];

		struct {
			unsigned char first_buffer_pos;
			unsigned char all_buffer_cnt;
			unsigned char packcnt;
			uint32_t len[12];
		};
	};
	char dat[1024 * 16 - sizeof(header)];
};

static inline int PER_PACK_DAT_LEN(void)
{
	return sizeof(DATA_BLOCK::dat);
}

struct DATA_BLOCK_MEM{
    struct DATA_BLOCK db[MAX_PACK_CNT];
};




key_t getKey(const char *path, int num)
{
	key_t key;
	if (-1 == (key = ftok(path, num))) {
		return -1;
	}
	return key;
}

void *getSHM(key_t key, int sz)
{
	int sid;
	if (-1 == (sid = shmget(key, sz, IPC_CREAT | IPC_EXCL | 0666))) {
		if (errno == EEXIST) {
			if (0 == (sid = shmget(key, sz, 0666))) {
				goto succ;
			}
		}
		return nullptr;
	}
succ:
	void *p = shmat(sid, nullptr, 0);

	if ((long int)p != -1) {
		return p;
	}
	return nullptr;
}

int find_last_dat_index(struct DATA_BLOCK_MEM *blcok)
{
	if (!blcok)
		return 0;

	int maxindex = 0;
	DATA_BLOCK *pdb = blcok->db;
	for (size_t i = 0; i < MAX_PACK_CNT; i++) {
		if (pdb[i].first_buffer_pos != i) {
			continue;
		}
		#warning "包头数据没有验证"
		if (pdb[i].packcnt > 0) {
			maxindex = i;
		}
	}
	return maxindex;
}

int get_buff_handle(struct DATA_BLOCK_HANDLE *hdl, enum DAT_BLOCK_FLG flg)
{
	if (!hdl) {
		return -1;
	}

	key_t key = getKey("/", 123456);

	DATA_BLOCK_MEM *blcok = (DATA_BLOCK_MEM *)getSHM(key, sizeof(DATA_BLOCK_MEM));

	if (!blcok) {
		return -2;
	}

	if (flg | DAT_BLOCK_CLEAN) {
		for (size_t i = 0; i < MAX_PACK_CNT; i++) {
			memset(blcok->db[i].header, 0, sizeof(blcok->db[i].header));
		}
		hdl->pointcnt = 0;
	} else {
		hdl->pointcnt = find_last_dat_index(blcok);
	}

	hdl->pdat = blcok;
	hdl->maxcnt = MAX_PACK_CNT;
	return 0;
}

int put_stream_buff(struct DATA_BLOCK_HANDLE *hdl, VENC_STREAM_S *pstStream)
{
	if(!hdl || ! pstStream){
		return -1;
	}

	DATA_BLOCK_MEM *blcok = (DATA_BLOCK_MEM *)hdl->pdat;

	struct DATA_BLOCK * pdb = blcok->db;
	

	int all_dat_len = 0;
	for (size_t i = 0; i < pstStream->u32PackCount; i++) {
		all_dat_len += pstStream->pstPack[i].u32Len - pstStream->pstPack[i].u32Offset;
	}

	int all_buffer_cnt = all_dat_len / PER_PACK_DAT_LEN() + (all_dat_len % PER_PACK_DAT_LEN()) ? 1 : 0;

	//写入头节点
	pdb[hdl->pointcnt].first_buffer_pos = hdl->pointcnt;
	pdb[hdl->pointcnt].all_buffer_cnt = all_buffer_cnt;
	//写入包数量
	pdb[hdl->pointcnt].packcnt = pstStream->u32PackCount;
	//写入包数据
	size_t wr_pos = 0;
	size_t wr_pack_index = hdl->pointcnt;
	for (size_t i = 0; i < pstStream->u32PackCount; i++) {
		//计算stream包
		const size_t pack_tatal_len = pstStream->pstPack[i].u32Len - pstStream->pstPack[i].u32Offset;		
		unsigned char *pack_dat_start = pstStream->pstPack[i].pu8Addr + pstStream->pstPack[i].u32Offset;

		size_t pack_left_len = pack_tatal_len;

		while (pack_left_len) {
			//写入当前stream数据
			size_t can_wr_sz = wr_pos % PER_PACK_DAT_LEN();

			int share_mempos = wr_pos % PER_PACK_DAT_LEN();
			int pack_mempos = pack_tatal_len - pack_left_len;

			if (can_wr_sz > pack_left_len) {
				memcpy(pdb[wr_pack_index].dat + share_mempos, pack_dat_start + pack_mempos, pack_left_len);
				wr_pos += pack_left_len;
				break;
			} else {
				memcpy(pdb[wr_pack_index].dat + share_mempos, pack_dat_start + pack_mempos, can_wr_sz);
				pack_left_len -= can_wr_sz;
				wr_pack_index++;
				wr_pack_index = wr_pack_index % MAX_PACK_CNT;
				wr_pos += can_wr_sz;
			}
		}
	}
	//hdl->pointcnt = wr_pos
	return 0;
}
