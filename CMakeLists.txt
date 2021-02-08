cmake_minimum_required(VERSION 3.0)


PROJECT(PROJECT_ONE)

set(main aim)
add_executable(${main} main.cpp)


INSTALL(TARGETS ${main}
       RUNTIME DESTINATION ./
)

