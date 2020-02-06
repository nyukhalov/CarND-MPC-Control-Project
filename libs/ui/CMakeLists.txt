set(TARGET libui)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("include/ui")

add_library(${TARGET} STATIC
    src/visualizer.cpp
    src/window.cpp
)

target_include_directories(${TARGET} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include/"
)

target_link_libraries(${TARGET} PUBLIC
    libmpc
    ${OpenCV_LIBS}
)
