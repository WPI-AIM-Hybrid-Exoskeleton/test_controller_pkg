cmake_minimum_required(VERSION 3.0.2)
project(test_controller_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS  
  roscpp
  # rbdl_model
  # rbdl_server
  # controller_modules
  controller_modules
  ambf_msgs
  ambf_client
  # sensor_msgs
)

include_directories( include ${AMBF_CLIENT_INCLUDE_DIRS}  ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})


get_filename_component(PARENT_CMAKE ../cmake ABSOLUTE)
set (CMAKE_MODULE_PATH ${PARENT_CMAKE} )
FIND_PACKAGE (ambf_client REQUIRED)
include_directories(${AMBF_CLIENT_INCLUDE_DIR})
MESSAGE(STATUS "Found AMBsd;lf;lasfklasjf;ladsjf;lsadjf;lsdjfF: ${AMBF_CLIENT_INCLUDE_DIR}")
find_package (Boost COMPONENTS program_options filesystem REQUIRED)






include_directories(${catkin_INCLUDE_DIRS})
set (CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};${CMAKE_CURRENT_SOURCE_DIR}/cmake")
#get_filename_component(PARENT_CMAKE ../cmake ABSOLUTE)
#set (CMAKE_MODULE_PATH ${PARENT_CMAKE} )

find_package (Boost COMPONENTS program_options filesystem REQUIRED)

MESSAGE(STATUS "${PROJECT_SOURCE_DIR}/cmake")
# static library external dependencies source files
# file (GLOB_RECURSE source_eigen     RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/external/Eigen/Eigen/*)
file (GLOB_RECURSE source_ambf  RELATIVE ${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/cmake/*)
MESSAGE(STATUS " ${source_ambf}")
# group source files (MSVC likes this)
foreach (FILE ${source} ${source_eigen} ${source_ambf})
  get_filename_component (PARENT_DIR "${FILE}" PATH)
  string (REGEX REPLACE "(\\./)?(src|include)/?" "" GROUP "${PARENT_DIR}")
  string (REPLACE "/" "\\" GROUP "${GROUP}")
  source_group ("${GROUP}" FILES "${FILE}")
endforeach ()


set (ambf_msgs_DIR "${PROJECT_SOURCE_DIR}/cmake")
find_package(catkin REQUIRED COMPONENTS
   ambf_msgs
   geometry_msgs
   message_generation
   tf
)



set (ambf_msgs_DIR "/${PROJECT_SOURCE_DIR}/cmake")
find_package(catkin REQUIRED COMPONENTS
   ambf_msgs
   geometry_msgs
   message_generation
   tf
)

set (AMBF_CLIENT_LIBRARY "$ENV{HOME}/ambf/build/devel/lib/libambf_client_cpp.so")

FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)
#find_package(GTest REQUIRED)

FIND_PACKAGE (ambf_client REQUIRED)
include_directories(${AMBF_CLIENT_INCLUDE_DIR})
MESSAGE(STATUS "Found AMBF: ${AMBF_CLIENT_INCLUDE_DIR}")
find_package (Boost COMPONENTS program_options filesystem REQUIRED)

catkin_package(INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS ambf_msgs
#  DEPENDS system_lib
)

include_directories( include ${AMBF_CLIENT_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})


#find_package (ambf_client REQUIRED)
find_package (GLFW REQUIRED)
find_package (Boost COMPONENTS program_options filesystem REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable (main_test src/main_test.cpp)
add_dependencies (main_test ${catkin_EXPORTED_TARGETS})
# target_link_libraries(kuka_control_node ${GLFW_LIBRARIES} ${catkin_LIBRARIES} ${Eigen_LIBRARIES} pthread)
target_link_libraries (main_test ${AMBF_CLIENT_LIBRARY} ${RBDL_SERVER_LIBRARIES} ${CONTROLLER_MODULES_LIBRARIES} ${GLFW_LIBRARIES} ${RBDL_SERVER_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} pthread controller_modules rbdl_server rbdl_model ${RBDL_SERVER_LIBRARIES} ${RBDL_MODEL_LIBRARIES})

