cmake_minimum_required(VERSION 2.8.3)
project(sample_based_optimisation_based_path_planner)
set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")
find_package(catkin REQUIRED COMPONENTS octomap_ros cmake_modules)

catkin_package()
find_package(ompl REQUIRED)

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

MESSAGE( "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")

MESSAGE( "catkin_LIBRARIES : " ${catkin_LIBRARIES} )
MESSAGE( "OMPL_LIBRARIES : " ${OMPL_LIBRARIES})




find_package(PCL 1.7 REQUIRED COMPONENTS
common
octree
io
kdtree
search
sample_consensus
filters
2d
features
registration
geometry
visualization
outofcore
surface
keypoints
ml
segmentation
recognition
people
tracking
stereo
)

FIND_PACKAGE( OpenCV REQUIRED )

MESSAGE("OpenCV_DIR: " ${OpenCV_DIR})
MESSAGE("OpenCV_VERSION: " ${OpenCV_VERSION})
MESSAGE("OpenCV_INCLUDE_DIRS: " ${OpenCV_INCLUDE_DIRS})
MESSAGE("OpenCV_LIBS: " ${OpenCV_LIBS})


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


MESSAGE("######################################################################PCL#####################################################################")
MESSAGE("PCL_FOUND:" ${PCL_FOUND})
MESSAGE("PCL_INCLUDE_DIRS:" ${PCL_LIBRARY_DIRS})
MESSAGE("PCL_LIBRARIES:" ${PCL_LIBRARIES})
MESSAGE("PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
MESSAGE("PCL_VERSION:" ${PCL_VERSION})
MESSAGE("PCL_COMPONENTS:" ${PCL_COMPONENTS})
MESSAGE("PCL_DEFINITIONS:" ${PCL_DEFINITIONS})
pkg_check_modules(OpenNI REQUIRED libopenni)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})



add_executable(sample_based_optimisation_based_path_planner  src/sample_based_optimisation_based_path_planner.cpp)
TARGET_LINK_LIBRARIES(sample_based_optimisation_based_path_planner
${DYNAMICEDT3D_LIBRARIES}  ${catkin_LIBRARIES} ${Boost_LIBRARIES}  ${OMPL_LIBRARIES} SDL ${OpenCV_LIBS})



add_executable(binvox2bt src/binvox2bt.cpp)
TARGET_LINK_LIBRARIES(binvox2bt ${DYNAMICEDT3D_LIBRARIES}  ${catkin_LIBRARIES} ${Boost_LIBRARIES}  )


#add_executable(test_kuka src/test_kuka.cpp)
#TARGET_LINK_LIBRARIES(test_kuka ${DYNAMICEDT3D_LIBRARIES}  ${catkin_LIBRARIES} ${Boost_LIBRARIES}  )



