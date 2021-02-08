#ifndef __SHARE_MEM_H__
#define __SHARE_MEM_H__

#if __cplusplus
extern "C" {
#endif

#include <stdint.h>

struct DATA_BLOCK {
	union {
		unsigned char header[64];

		struct {
			unsigned char pos;
			uint16_t packcnt;
			uint16_t len[30];
		};
	};

	char dat[1024 * 16 - 16];
};

struct DATA_BLOCK_MEM{
    struct DATA_BLOCK db[256];
};

struct DATA_BLOCK_MEM *get_DATA_BLOCK_MEM(void);


#if __cplusplus
}
#endif

#endif
