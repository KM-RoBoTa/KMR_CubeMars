cmake_minimum_required(VERSION 3.20)
project(KMR_CubeMars)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)

# Source files
add_library(${PROJECT_NAME} 
            source/writer.cpp
            source/listener.cpp
            source/motor_handler.cpp
            source/utils.cpp
            )

# Directories containing header files
target_include_directories(${PROJECT_NAME} PUBLIC include)

# Locations of the used libraries
#target_link_directories(${PROJECT_NAME} PUBLIC /usr/local/lib)

# Generate Docs
option(BUILD_DOCS "Generate Docs" ON)
if(BUILD_DOCS)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/docs)  # path to docs' cmakelists
endif()

# Generate example executables
add_executable(ex1_impedance examples/ex1_impedance.cpp)
target_link_libraries(ex1_impedance ${PROJECT_NAME})
add_executable(ex2_position examples/ex2_position.cpp)
target_link_libraries(ex2_position ${PROJECT_NAME})
add_executable(ex3_speed examples/ex3_speed.cpp)
target_link_libraries(ex3_speed ${PROJECT_NAME})
add_executable(ex4_torque examples/ex4_torque.cpp)
target_link_libraries(ex4_torque ${PROJECT_NAME})
add_executable(singleThread examples/singleThread.cpp)
target_link_libraries(singleThread ${PROJECT_NAME})
add_executable(PID_tuning examples/PID_tuning.cpp)
target_link_libraries(PID_tuning ${PROJECT_NAME})


add_executable(ping examples/ping.cpp)
target_link_libraries(ping ${PROJECT_NAME})