cmake_minimum_required(VERSION 3.0)


PROJECT(PROJECT_ONE)


add_executable(aim main.cpp)


INSTALL(TARGETS ${main}
       RUNTIME DESTINATION bin
)

