SET(BUILDROOT_PATH /opt/hisi-linux/x86-arm/arm-himix200-linux/)
#SET(MYSYSROOT ${BUILDROOT_PATH}/arm-buildroot-linux-gnueabihf/sysroot)
#SET(CMAKE_SYSROOT ${MYSYSROOT})
SET(toolpathprefix ${BUILDROOT_PATH}/bin/arm-himix200-linux-)

SET(CMAKE_C_COMPILER ${toolpathprefix}gcc)
SET(CMAKE_CXX_COMPILER ${toolpathprefix}g++)
SET(CMAKE_STRIP  ${toolpathprefix}strip)
SET(CMAKE_AR ${toolpathprefix}ar)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
add_definitions(-mcpu=cortex-a7 -mapcs -rdynamic)
add_definitions(-mfpu=neon-vfpv4)
add_definitions("-O2")
add_definitions("-g3")
add_definitions(-Wall)

