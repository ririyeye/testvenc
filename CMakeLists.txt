cmake_minimum_required(VERSION 3.5)


PROJECT(PROJECT_ONE)

set(SENSOR0_TYPE HT_BT656_2M_30FPS_8BIT)
set(SENSOR1_TYPE HT_BT656_2M_30FPS_8BIT)
add_definitions(-DSENSOR0_TYPE=HT_BT656_2M_30FPS_8BIT)
add_definitions(-DSENSOR1_TYPE=HT_BT656_2M_30FPS_8BIT)



set(main aim)
include_directories(${PROJECT_SOURCE_DIR}/mpp/include)
include_directories(${PROJECT_SOURCE_DIR}/common)

#add common
add_subdirectory(common)

add_executable(${main} main.cpp sample_vio.c)

link_directories(${PROJECT_SOURCE_DIR}/mpp/lib)

target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/lib_hiae.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libisp.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/lib_hidehaze.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/lib_hidrc.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/lib_hildci.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/lib_hicalcflicker.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/lib_hiawb.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libsns_imx327.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libsns_imx327_2l.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libsns_imx307.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libsns_imx307_2l.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libsns_imx458.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libsns_mn34220.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libsns_os05a.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libsns_os08a10.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libsns_gc2053.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libsns_sc4210.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libsns_ov12870.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libsns_os04b10.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libsns_imx415.a)


#tools
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libisp.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libmpi.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/lib_hidehaze.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/lib_hidrc.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/lib_hildci.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libhifisheyecalibrate.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libsecurec.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/lib_hicalcflicker.a)

#audio
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libVoiceEngine.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libupvqe.a)
target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libdnvqe.a)
#target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libaacenc.a)
#target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/mpp/lib/libaacdec.a)


target_link_libraries(${main} pthread)
target_link_libraries(${main} dl)



target_link_libraries(${main} ${PROJECT_SOURCE_DIR}/common/common)

INSTALL(TARGETS ${main}
       RUNTIME DESTINATION ./
)

