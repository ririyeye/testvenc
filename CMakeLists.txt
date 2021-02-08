cmake_minimum_required(VERSION 3.5)


PROJECT(PROJECT_ONE)

set(main aim)
add_executable(${main} main.cpp)

include_directories(${PROJECT_SOURCE_DIR}/mpp/include)
link_directories(${PROJECT_SOURCE_DIR}/mpp/lib)

set(REL_LIB ${PROJECT_SOURCE_DIR}/mpp/lib)

target_link_libraries(${main} $(REL_LIB)/lib_hiae.a)


target_link_libraries(${main} $(REL_LIB)/lib_hiae.a)
target_link_libraries(${main} $(REL_LIB)/libisp.a)
target_link_libraries(${main} $(REL_LIB)/lib_hidehaze.a)
target_link_libraries(${main} $(REL_LIB)/lib_hidrc.a)
target_link_libraries(${main} $(REL_LIB)/lib_hildci.a)
target_link_libraries(${main} $(REL_LIB)/lib_hicalcflicker.a)
target_link_libraries(${main} $(REL_LIB)/lib_hiawb.a)
target_link_libraries(${main} $(REL_LIB)/libsns_imx327.a)
target_link_libraries(${main} $(REL_LIB)/libsns_imx327_2l.a)
target_link_libraries(${main} $(REL_LIB)/libsns_imx307.a)
target_link_libraries(${main} $(REL_LIB)/libsns_imx307_2l.a)
target_link_libraries(${main} $(REL_LIB)/libsns_imx458.a)
target_link_libraries(${main} $(REL_LIB)/libsns_mn34220.a)
target_link_libraries(${main} $(REL_LIB)/libsns_os05a.a)
target_link_libraries(${main} $(REL_LIB)/libsns_os08a10.a)
target_link_libraries(${main} $(REL_LIB)/libsns_gc2053.a)
target_link_libraries(${main} $(REL_LIB)/libsns_sc4210.a)
target_link_libraries(${main} $(REL_LIB)/libsns_ov12870.a)
target_link_libraries(${main} $(REL_LIB)/libsns_os04b10.a)
target_link_libraries(${main} $(REL_LIB)/libsns_imx415.a)





INSTALL(TARGETS ${main}
       RUNTIME DESTINATION ./
)

