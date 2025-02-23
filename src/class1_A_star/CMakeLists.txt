# 最小版本
cmake_minimum_required(VERSION 3.0)

# 工程名
project(Astar)

# 将所有相关文件都添加进来，用于编译
add_executable(A_star main.cc A_star.cc)

# 找python包
find_package(Python3 COMPONENTS Development NumPy)

# 指定目标包含的头文件路径
target_include_directories(A_star PRIVATE 
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 指定目标链接的库
target_link_libraries(A_star Python3::Python Python3::NumPy)






