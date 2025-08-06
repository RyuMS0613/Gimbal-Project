cmake_minimum_required(VERSION 3.30)
project(PROJECT_GIMBAL)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

file(GLOB_RECURSE SOURCE src/*.c)
file(GLOB_RECURSE HEADER include/*.h)

add_library(${PROJECT_NAME} STATIC
        ${SOURCE}
        ${HEADER}
)

target_include_directories(${PROJECT_NAME} PUBLIC
        include
)

add_executable(Gimbal_test main.c)

target_include_directories(Gimbal_test PUBLIC include)

target_link_libraries(Gimbal_test PUBLIC
        PROJECT_GIMBAL
        "../libs/NIDAQmx"
)