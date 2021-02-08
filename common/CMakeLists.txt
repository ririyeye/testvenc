
add_library(common STATIC
loadbmp.c
#sample_comm_audio.c
sample_comm_isp.c
#sample_comm_region.c
sample_comm_sys.c
#sample_comm_vdec.c
sample_comm_venc.c
sample_comm_vi.c
sample_comm_vo.c
sample_comm_vpss.c
)

set(main common)

target_link_directories(${main} PUBLIC ${PROJECT_SOURCE_DIR}/mpp/lib)

target_link_libraries(${main} PRIVATE lib_hiae.a)
target_link_libraries(${main} PRIVATE libisp.a)
target_link_libraries(${main} PRIVATE lib_hidehaze.a)
target_link_libraries(${main} PRIVATE lib_hidrc.a)
target_link_libraries(${main} PRIVATE lib_hildci.a)
target_link_libraries(${main} PRIVATE lib_hicalcflicker.a)
target_link_libraries(${main} PRIVATE lib_hiawb.a)
target_link_libraries(${main} PRIVATE libsns_imx327.a)
target_link_libraries(${main} PRIVATE libsns_imx327_2l.a)
target_link_libraries(${main} PRIVATE libsns_imx307.a)
target_link_libraries(${main} PRIVATE libsns_imx307_2l.a)
target_link_libraries(${main} PRIVATE libsns_imx458.a)
target_link_libraries(${main} PRIVATE libsns_mn34220.a)
target_link_libraries(${main} PRIVATE libsns_os05a.a)
target_link_libraries(${main} PRIVATE libsns_os08a10.a)
target_link_libraries(${main} PRIVATE libsns_gc2053.a)
target_link_libraries(${main} PRIVATE libsns_sc4210.a)
target_link_libraries(${main} PRIVATE libsns_ov12870.a)
target_link_libraries(${main} PRIVATE libsns_os04b10.a)
target_link_libraries(${main} PRIVATE libsns_imx415.a)


#tools
target_link_libraries(${main} PRIVATE libisp.a)
target_link_libraries(${main} PRIVATE libmpi.a)
target_link_libraries(${main} PRIVATE lib_hidehaze.a)
target_link_libraries(${main} PRIVATE lib_hidrc.a)
target_link_libraries(${main} PRIVATE lib_hildci.a)
target_link_libraries(${main} PRIVATE libhifisheyecalibrate.a)
target_link_libraries(${main} PRIVATE libsecurec.a)
target_link_libraries(${main} PRIVATE lib_hicalcflicker.a)

#audio
target_link_libraries(${main} PRIVATE libVoiceEngine.a)
target_link_libraries(${main} PRIVATE libupvqe.a)
target_link_libraries(${main} PRIVATE libdnvqe.a)
#target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libaacenc.a)
#target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libaacdec.a)

#hdmi
target_link_libraries(${main} PRIVATE libhdmi.a)


#imx335
target_link_libraries(${main} PRIVATE libsns_imx335_forcar.a)