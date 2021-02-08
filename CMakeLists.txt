cmake_minimum_required(VERSION 3.5)


PROJECT(PROJECT_ONE)

set(SENSOR0_TYPE HT_BT656_2M_30FPS_8BIT)
set(SENSOR1_TYPE HT_BT656_2M_30FPS_8BIT)
add_definitions(-DSENSOR0_TYPE=HT_BT656_2M_30FPS_8BIT)
add_definitions(-DSENSOR1_TYPE=HT_BT656_2M_30FPS_8BIT)



set(main aim)
include_directories(${PROJECT_SOURCE_DIR}/mpp/include)
include_directories(${PROJECT_SOURCE_DIR}/common)
include_directories(${PROJECT_SOURCE_DIR}/sharemem)

#add common
add_subdirectory(common)
add_subdirectory(sharemem)

add_executable(${main} main.cpp sample_vio.c)


target_link_libraries(${main} PRIVATE pthread)
target_link_libraries(${main} PRIVATE dl)


target_link_directories(${main} PUBLIC ${PROJECT_SOURCE_DIR}/common)
target_link_libraries(${main} PRIVATE common)

target_link_directories(${main} PUBLIC ${PROJECT_SOURCE_DIR}/sharemem)
target_link_libraries(${main} PRIVATE sharemem)

INSTALL(TARGETS ${main}
       RUNTIME DESTINATION ./
)

