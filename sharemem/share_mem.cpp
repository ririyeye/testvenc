#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <errno.h>
#include "share_mem.h"
#include <string.h>
#include <stdio.h>
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
	int firstflg = 0;
	printf("get shm \n");
	if (-1 == (sid = shmget(key, sz, IPC_CREAT | IPC_EXCL | 0666))) {
		if (errno == EEXIST) {
			if (0 == (sid = shmget(key, sz, 0666))) {
				goto succ;
			}
		} else {
			//第一次获取
			//清空数据
			firstflg = 1;
		}
		return nullptr;
	}
succ:
	void *p = shmat(sid, nullptr, 0);

	if ((long int)p != -1) {
		printf("get mem\n");
		if (firstflg) {
            printf("get init mem\n");
			DATA_BLOCK_MEM *blcok = (DATA_BLOCK_MEM *)p;
            
            for (size_t i = 0; i < 256; i++)
            {
		        memset(blcok->db[i].header, 0, sizeof(blcok->db[i].header));
	        }
		}
		return p;
	}
    printf("get null mem\n");
	return nullptr;
}

DATA_BLOCK_MEM * get_DATA_BLOCK_MEM(void)
{
	key_t key = getKey("/", 123456);

	return (DATA_BLOCK_MEM *)getSHM(key, sizeof(DATA_BLOCK_MEM));
}










