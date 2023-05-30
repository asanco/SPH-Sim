# Collect the source files
file(GLOB GLAD_SOURCES "src/*.c")

# Create the glad library target
add_library(glad ${GLAD_SOURCES})

# Set the include directories for glad
target_include_directories(glad PUBLIC include)