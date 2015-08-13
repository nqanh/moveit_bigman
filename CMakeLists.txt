cmake_minimum_required(VERSION 2.8.3)
project(moveit_bigman)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS moveit_core moveit_ros_planning moveit_ros_planning_interface pluginlib cmake_modules)

find_package(Boost REQUIRED system filesystem date_time thread)



find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 3.0.0 REQUIRED)
find_package(Eigen3 REQUIRED)

## Set compiler optimization flags
set(CMAKE_CXX_FLAGS "-DNDEBUG  -O0 -Wall -fopenmp -Wno-deprecated -Wenum-compare -D WITH_OPENNI=ON -D WITH_OPENNI2=ON -D WITH_V4L=ON")


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

include_directories(EIGEN3_INCLUDE_DIR)








catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pr2_moveit_tutorials
  CATKIN_DEPENDS 
    moveit_core
    moveit_ros_planning_interface
    interactive_markers
)

#find_package(Eigen REQUIRED)

#################################
## Build
#################################

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(main_planning src/main_planning.cpp)
target_link_libraries(main_planning ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS main_planning DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#################################
## for lauch files
#################################

#add_executable(main_detection src/detection/main_detection.cpp   src/detection/CsvReader.cpp   src/detection/CsvWriter.cpp  src/detection/ModelRegistration.cpp   src/detection/Mesh.cpp   src/detection/Model.cpp   src/detection/PnPProblem.cpp   src/detection/Utils.cpp  src/detection/RobustMatcher.cpp src/detection/RootFrame.cpp)
#target_link_libraries(main_detection   ${OpenCV_LIBS} ${Boost_LIBRARIES} ${catkin_LIBRARIES} )
#install(TARGETS main_detection DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(main src/detection/main.cpp src/detection/Detection.cpp  src/detection/CsvReader.cpp   src/detection/CsvWriter.cpp  src/detection/ModelRegistration.cpp   src/detection/Mesh.cpp   src/detection/Model.cpp   src/detection/PnPProblem.cpp   src/detection/Utils.cpp  src/detection/RobustMatcher.cpp src/detection/RootFrame.cpp)
target_link_libraries(main ${OpenCV_LIBS} ${Boost_LIBRARIES} ${catkin_LIBRARIES} )
install(TARGETS main DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
