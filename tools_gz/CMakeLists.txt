cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(tools_gz)


#============================================================================
# Configure the project
#============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#============================================================================
# Find packages
#============================================================================
find_package(OsqpEigen REQUIRED)
find_package(gz-msgs10 REQUIRED)


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library(${PROJECT_NAME}
    SHARED
		src/tools_gz.cpp
)

#set(MSGS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../go2_msgs/build/proto) 

#  is the command to apoint the directory where the libraries used in the project are locaded 
#target_link_directories(${PROJECT_NAME}
#   PUBLIC
#	${MSGS_DIR}
#)

# libries used in the project
target_link_libraries(${PROJECT_NAME}
	PUBLIC
		${GZ-MSGS_LIBRARIES}
		OsqpEigen::OsqpEigen
		jump_cmsgs-msgs
)	


install(
	TARGETS             ${PROJECT_NAME}
    LIBRARY DESTINATION /usr/local/lib
)

install(
	DIRECTORY             ${CMAKE_SOURCE_DIR}/include/
    DESTINATION /usr/local/include
	FILES_MATCHING PATTERN "*.h"
)