file(GLOB_RECURSE SOURCES
    ./*.hpp
    ./*.cpp
    )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/renderer/include)

add_executable(SPHSim ${SOURCES})

target_link_libraries(SPHSim PRIVATE sfml-graphics PRIVATE sfml-window PRIVATE sfml-system PRIVATE glad)

target_include_directories(SPHSim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
