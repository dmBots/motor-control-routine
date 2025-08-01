cmake_minimum_required(VERSION 3.0.2)
project(dm_hw)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dmbot_serial
  controller_manager
  dm_common
  urdf
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp 
  std_msgs
  dmbot_serial
  controller_manager
  dm_common
  
  
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
        src/DmHW.cpp
        src/DmHWLoop.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES} 
        
        )

