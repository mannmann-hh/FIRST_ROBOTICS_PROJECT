cmake_minimum_required(VERSION 3.16)
project(first_project)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 寻找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bunker_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) # 必须有，用于生成你的 TfErrorMsg

# 2. 生成自定义接口 (TfErrorMsg.msg)
# 这一步会生成 C++ 头文件，供后面的代码调用
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TfErrorMsg.msg"
  DEPENDENCIES std_msgs
)

# 3. 添加里程计节点 (odometer)
add_executable(odometer src/odometer.cpp)

# 链接外部依赖库
ament_target_dependencies(odometer
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  bunker_msgs
  std_srvs
)

# 关键：由于你在同一个包里定义了消息，odometer 必须链接到这个接口目标
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(odometer "${cpp_typesupport_target}")

# 4. 安装路径 (避免运行脚本找不到节点)
install(TARGETS odometer
  DESTINATION lib/${PROJECT_NAME})

# 安装 launch 和 config 文件夹 (作业要求，后续一键启动用)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()