cmake_minimum_required(VERSION 2.8.3)
project(savePic)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES savePic
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

FIND_PACKAGE( OpenCV REQUIRED )

ADD_EXECUTABLE(savePic src/savePic.cpp  )
target_link_libraries(savePic ${catkin_LIBRARIES}) 
target_link_libraries(savePic ${OpenCV_LIBRARIES} )
