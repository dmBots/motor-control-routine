cmake_minimum_required(VERSION 3.5)
project(dmbot_serial LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

add_library(motor
  src/protocol/damiao.cpp
  src/protocol/socketcan.cpp
)
target_include_directories(motor
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(motor PUBLIC Threads::Threads)

add_executable(test_motor src/test.cpp)
target_link_libraries(test_motor PRIVATE motor)

install(TARGETS motor test_motor
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
