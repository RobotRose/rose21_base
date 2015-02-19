cmake_minimum_required(VERSION 2.8.3)
project(rose21_drive_controller)

find_package(catkin REQUIRED COMPONENTS
		contact_sensor_msgs
		geometry_msgs
		operator_messaging
		opteq_wheelunits_01
		roscomm
		roscpp
		rose_base_msgs
		rose_common
		rose_conversions
		rose_footprint_collision_checker
		rose_geometry
		rose_transformations
		sensor_msgs
		server_multiple_client
		rose_shared_variables
	)

catkin_package(
 INCLUDE_DIRS 
 	include
 LIBRARIES 

 CATKIN_DEPENDS 
		contact_sensor_msgs
		geometry_msgs
		operator_messaging
		opteq_wheelunits_01
		roscomm
		roscpp
		rose_base_msgs
		rose_common
		rose_conversions
		rose_footprint_collision_checker
		rose_geometry
		rose_transformations
		sensor_msgs
		server_multiple_client
		rose_shared_variables
 DEPENDS 

)

set(CMAKE_CXX_FLAGS "-std=c++0x -g") #! @todo OH [IMPR]: Remove -g flag in release?

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable( rose21_drive_controller 	
					src/drive_controller.cpp
					src/drive_controller_node.cpp
)
						

add_dependencies( rose21_drive_controller ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries( rose21_drive_controller ${catkin_LIBRARIES} )
