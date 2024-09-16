cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(jump_rgc)
set (GZ_DISTRIBUTION "Harmonic")

#============================================================================
# Configure the project
#============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#============================================================================
# Find packages
#============================================================================
# If you get an error at this line, you need to install gz-cmake
find_package(gz-transport13 REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-launch7 REQUIRED)

# find_package(jump_msgs-msgs REQUIRED)


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME}
    SHARED
		src/load_rgc.cpp
		src/rgc_controller.cpp
		src/pred_control.cpp
		src/op_wrapper.cpp	
		src/opt_problem.cpp
		src/opt_problem1.cpp
		src/opt_problem2.cpp
		src/opt_problem3.cpp
		src/opt_problem4.cpp
		src/opt_problem5.cpp
		src/opt_problem6.cpp
		src/jump_robot_model.cpp
		
)

target_link_libraries(${PROJECT_NAME}
	PUBLIC
		${GZ-SIM_LIBRARIES}
		${GZ-MATH_LIBRARIES}
		${GZ-LAUNCH_LIBRARIES}
		gz-plugin2::core
		OsqpEigen::OsqpEigen
		jump_cmsgs-msgs
)	


install(
	TARGETS             ${PROJECT_NAME}
    LIBRARY DESTINATION /usr/lib/gazebo/plugins
	COMPONENT           jump_rgc
)