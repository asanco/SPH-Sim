cmake_minimum_required(VERSION 3.12)

project(SPH-Sim C CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(thirdparty)
add_subdirectory(src)

configure_file(${PROJECT_SOURCE_DIR}//res//font.ttf ${PROJECT_BINARY_DIR}//res//font.ttf COPYONLY)
configure_file(${PROJECT_SOURCE_DIR}//src//renderer//shaders//basicShader.frag ${PROJECT_BINARY_DIR}//res//basicShader.frag COPYONLY)
configure_file(${PROJECT_SOURCE_DIR}//src//renderer//shaders//basicShader.vert ${PROJECT_BINARY_DIR}//res//basicShader.vert COPYONLY)

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")