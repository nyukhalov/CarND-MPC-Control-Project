include_directories("../libs/Eigen-3.3")

set(MPC_SOURCES
    main.cpp
    helpers.cpp
)

set(MPC_DEBUG_SOURCES
    mpc_debug.cpp
    helpers.cpp
)

add_executable(mpc ${MPC_SOURCES})
add_executable(mpc_debug ${MPC_DEBUG_SOURCES})

target_link_libraries(mpc libmpc libui z ssl uv uWS)
target_link_libraries(mpc_debug libmpc libui z)

