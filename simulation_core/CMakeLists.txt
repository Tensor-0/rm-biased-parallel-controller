cmake_minimum_required(VERSION 3.14)
project(wheel_legged_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# pybind11
find_package(pybind11 REQUIRED)

# 源文件
set(SOURCES
    wheel_legged_dynamics.cpp
    bindings.cpp
)

# 编译为 Python 模块
pybind11_add_module(wheel_legged_sim ${SOURCES})

# 头文件路径
target_include_directories(wheel_legged_sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# 安装路径
install(TARGETS wheel_legged_sim
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}
)
