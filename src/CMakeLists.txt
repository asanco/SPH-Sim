file(GLOB_RECURSE SOURCES
    ./*.hpp
    ./*.cpp
    )

add_executable(SPHSim ${SOURCES})

target_link_libraries(SPHSim PRIVATE sfml-graphics PRIVATE sfml-window PRIVATE sfml-system)

target_include_directories(SPHSim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
