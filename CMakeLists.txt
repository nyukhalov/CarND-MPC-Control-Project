project(MPC)

cmake_minimum_required (VERSION 3.5)

find_package(OpenCV REQUIRED)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O3)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(LIB_MPC_SRC
  src/mpc/MPC.cpp
  src/mpc/MPCConfig.cpp
  src/mpc/FG_eval.cpp
  src/mpc/models.cpp
)

set(LIB_UI_SRC
  src/ui/window.cpp
)

set(MPC_SOURCES
  src/main.cpp
  src/helpers.cpp
)

set(MPC_DEBUG_SOURCES
  src/mpc_debug.cpp
  src/helpers.cpp
)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(src/Eigen-3.3)
include_directories(${OpenCV_INCLUDE_DIRS})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_library(libmpc STATIC
  ${LIB_MPC_SRC}
)
add_library(libui STATIC
  ${LIB_UI_SRC}
)

add_executable(mpc ${MPC_SOURCES})
add_executable(mpc_debug ${MPC_DEBUG_SOURCES})

target_link_libraries(mpc libmpc libui ipopt z ssl uv uWS ${OpenCV_LIBS})
target_link_libraries(mpc_debug libmpc libui ipopt z ${OpenCV_LIBS})

