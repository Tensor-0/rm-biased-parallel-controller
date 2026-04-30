cmake_minimum_required(VERSION 3.14)
project(wheel_legged_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# pybind11 自动检测 (先尝试 python3，再尝试 /home/zhan/anaconda3/bin/python)
set(PYTHON_EXECUTABLE "python3")
execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -c "import pybind11; print(pybind11.get_cmake_dir())"
    OUTPUT_VARIABLE pybind11_cmake_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE pybind11_found
)
if(NOT pybind11_found EQUAL 0)
    set(PYTHON_EXECUTABLE "/home/zhan/anaconda3/bin/python")
    execute_process(
        COMMAND ${PYTHON_EXECUTABLE} -c "import pybind11; print(pybind11.get_cmake_dir())"
        OUTPUT_VARIABLE pybind11_cmake_dir
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif()
message(STATUS "Using Python: ${PYTHON_EXECUTABLE}")
message(STATUS "pybind11 cmake dir: ${pybind11_cmake_dir}")
list(APPEND CMAKE_PREFIX_PATH ${pybind11_cmake_dir})
find_package(pybind11 REQUIRED)

# 源文件
set(SOURCES
    wheel_legged_dynamics.cpp
    bindings.cpp
)

# 编译为 Python 模块
pybind11_add_module(wheel_legged_sim ${SOURCES})
target_include_directories(wheel_legged_sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(wheel_legged_sim PRIVATE -O2 -Wall)

# 安装
install(TARGETS wheel_legged_sim
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}
)
