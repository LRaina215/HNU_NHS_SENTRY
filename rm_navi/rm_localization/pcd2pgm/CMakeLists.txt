cmake_minimum_required(VERSION 3.5)
project(pcd2pgm)

# 1. 查找基础依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters) # 明确指定组件是个好习惯
find_package(OpenCV REQUIRED)

# 【新增】查找 libusb-1.0 (解决 PCL io 报错的关键)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB libusb-1.0 REQUIRED)

# 2. 包含头文件
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIRS}  # 包含 libusb 头文件
)

# 3. 添加可执行文件
add_executable(pcd2pgm src/pcd2pgm.cpp)

# 4. 指定 ROS 依赖
ament_target_dependencies(pcd2pgm
  rclcpp 
  # PCL 和 OpenCV 通常建议在 target_link_libraries 处理，而不是这里，但保留也没事
)

# 5. 链接库文件
target_link_libraries(pcd2pgm
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${LIBUSB_LIBRARIES}   # 【关键修改】必须显式链接 libusb库
)

# Install targets
install(TARGETS pcd2pgm
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config map
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
