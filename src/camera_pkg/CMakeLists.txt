cmake_minimum_required(VERSION 3.8)
project(camera_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package (ament_cmake_auto REQUIRED) #寻找依赖项
ament_auto_find_build_dependencies ()

include_directories(include) #头文件目录
 
ament_auto_add_executable(camera_split_node  #添加可执行文件
  src/camera_split.cpp
)
 
ament_auto_add_library (camera_split SHARED  #制作动态链接库，将可执行文件和自定义的库文件链接
  src/camera_split.cpp
)

ament_auto_package (  #安装package.xml，安装配置文件
  INSTALL_TO_SHARE
  launch
  config
)
