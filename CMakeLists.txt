cmake_minimum_required(VERSION 3.10)
project(base LANGUAGES CXX)

# C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 自动查找所有子目录中的 CMakeLists.txt (除了 cmake 目录)
file(GLOB_RECURSE CMAKE_FILES 
    LIST_DIRECTORIES false 
    RELATIVE ${CMAKE_SOURCE_DIR}
    "*/CMakeLists.txt"
)

# 遍历找到的 CMakeLists.txt 文件
foreach(CMAKE_FILE ${CMAKE_FILES})
    # 获取 CMakeLists.txt 所在的目录路径
    get_filename_component(DIR ${CMAKE_FILE} DIRECTORY)
    
    # 排除 cmake 目径及其子目录
    if(NOT DIR MATCHES "^cmake.*")
        # 添加子目录
        add_subdirectory(${DIR})
    endif()
endforeach()

# 添加可执行文件
add_executable(${PROJECT_NAME}
    ${CMAKE_CURRENT_SOURCE_DIR}/apps/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/apps/calibration/calibrate_camera.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/apps/calibration/calibrate_handeye.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/apps/calibration/calibrate_latency.cpp
)

# 链接子模块库
target_link_libraries(${PROJECT_NAME} PRIVATE
    rm_utils
    hikcamera
    serial
    usbcamera
    hardware_common
    threads
    auto_aim
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/apps/calibration
)

# 创建软链接到源代码中的config文件夹
execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_CURRENT_SOURCE_DIR}/config
    ${CMAKE_CURRENT_BINARY_DIR}/config
)