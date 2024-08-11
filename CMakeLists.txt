cmake_minimum_required(VERSION 3.10)
project(myproject)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)

# 查找PCL包
find_package(PCL 1.10 REQUIRED)

# 添加源文件
add_executable(${PROJECT_NAME} src/main.cpp
				src/DBSCAN.cpp
				src/filter.cpp
				src/read_write.cpp)

# 链接PCL库
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

# 包含项目的头文件目录
include_directories(include)

# 包含PCL头文件
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
