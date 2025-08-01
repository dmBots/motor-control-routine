cmake_minimum_required(VERSION 3.0.2)
project(dmbot_serial)

## 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## 查找 catkin 依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## 查找 libusb（使用 pkg-config）
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)



## 包含头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIRS}
)


## 声明 catkin 包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES motor
  CATKIN_DEPENDS roscpp
)
## 编译 motor 库
add_library(motor 
  src/protocol/damiao.cpp
)

## motor 库链接其他依赖库
target_link_libraries(motor
  ${CMAKE_CURRENT_SOURCE_DIR}/lib/libu2canfd.a
  ${LIBUSB_LIBRARIES}
  Threads::Threads
)



## 编译主程序
add_executable(test_motor
  src/test.cpp
)

target_link_libraries(test_motor
  ${catkin_LIBRARIES}
  motor
)

add_executable(dev_sn
    src/dev_sn.cpp
)
# 链接库到可执行文件
target_link_libraries(dev_sn PRIVATE
    ${LIBUSB_LIBRARIES}
)