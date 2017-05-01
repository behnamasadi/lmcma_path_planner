cmake_minimum_required(VERSION 2.8.3)
project(lmcma_path_planner)
set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
find_package(catkin REQUIRED COMPONENTS octomap_ros cmake_modules)

catkin_package()


find_package(Eigen REQUIRED)



# It defines the following variables
#  DYNAMICEDT3D_INCLUDE_DIRS - include directories for dynamicEDT3D
#  DYNAMICEDT3D_LIBRARY_DIRS - library directories for dynamicEDT3D (normally not used!)
#  DYNAMICEDT3D_LIBRARIES    - libraries to link against


FIND_PACKAGE(dynamicEDT3D REQUIRED )
INCLUDE_DIRECTORIES(${DYNAMICEDT3D_INCLUDE_DIRS})
link_directories(${DYNAMICEDT3D_LIBRARY_DIRS})

include_directories(include ${catkin_INCLUDE_DIRS}  ${SDFormat_INCLUDE_DIRS})

MESSAGE("DYNAMICEDT3D_INCLUDE_DIRS: " ${DYNAMICEDT3D_INCLUDE_DIRS})
MESSAGE("DYNAMICEDT3D_LIBRARIES: " ${DYNAMICEDT3D_LIBRARIES})
MESSAGE("DYNAMICEDT3D_LIBRARY_DIRS: " ${DYNAMICEDT3D_LIBRARY_DIRS})



MESSAGE( "catkin_LIBRARIES : " ${catkin_LIBRARIES} )




add_executable(example_lmcma  src/example_lmcma.cpp src/lmcma.cpp)
TARGET_LINK_LIBRARIES(example_lmcma ${catkin_LIBRARIES} ${Boost_LIBRARIES}  )


