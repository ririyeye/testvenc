#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "hi_common.h"
//#include "sample_vio.h"
#include "mpi_sys.h"
int main(int argc, char **argv)
{
	HI_S32 s32Ret = HI_FAILURE;
	HI_S32 s32Index;
	HI_U32 u32VoIntfType = 0;
	HI_U32 u32ChipId;
	HI_MPI_SYS_GetChipId(&u32ChipId);

	if (HI3516C_V500 == u32ChipId) {
		u32VoIntfType = 1;
	} else {
		u32VoIntfType = 0;
	}
	return 0;
}