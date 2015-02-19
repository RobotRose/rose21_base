cmake_minimum_required(VERSION 2.8.3)
project(odometry)

find_package(catkin REQUIRED
	COMPONENTS
		opteq_wheelunits_01
		rose_conversions
	)

set(CMAKE_CXX_FLAGS "-std=c++0x -fPIC -g")

catkin_package(
    INCLUDE_DIRS 
		include
    LIBRARIES 
		odometry 
	CATKIN_DEPENDS
		opteq_wheelunits_01
		rose_conversions
)

include_directories(
	${catkin_INCLUDE_DIRS}
	include 
)

add_library(odometry 
	SHARED
	src/odometry.cpp
)

target_link_libraries(odometry ${catkin_LIBRARIES})
add_dependencies(odometry ${PROJECT_NAME}_generate_messages_cpp)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
