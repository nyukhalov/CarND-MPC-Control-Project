set(TARGET libmpc)

add_library(${TARGET} STATIC
    src/FG_eval.cpp
    src/FG_eval.h
    src/models.cpp
    src/mpc_control.cpp
    src/config.cpp
    src/helpers.cpp
)

include_directories("../Eigen-3.3")
include_directories("include/mpc")

target_include_directories(${TARGET} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(${TARGET}
    ipopt
)
