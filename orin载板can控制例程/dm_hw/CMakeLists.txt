cmake_minimum_required(VERSION 3.16)
project(dm_main)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)



# 添加库
add_library(mylib STATIC
    src/hardware_interface/socketcan.cpp
    src/hardware_interface/damiao.cpp
    src/hardware_interface/DmHW.cpp  
)

# 包含目录
target_include_directories(mylib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(mylib PRIVATE Threads::Threads)

# 添加可执行文件
add_executable(${PROJECT_NAME}
    src/control_loop.cpp
    src/main.cpp
)

# 链接库到可执行文件
target_link_libraries(${PROJECT_NAME} PRIVATE
    mylib 
)


