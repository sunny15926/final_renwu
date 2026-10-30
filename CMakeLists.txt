cmake_minimum_required(VERSION 3.5)
project(armor_track_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 自定义消息：云台下发指令
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GimbalCmd.msg"
  "msg/GimbalFeedback.msg"
)

# ---- 闭环自瞄解算库（PnP + 坐标变换 + 一阶低通滤波 + 限幅 + 6维EKF）----
add_library(auto_aim
  src/closed_loop_solver.cpp
  src/ekf.cpp
)
target_include_directories(auto_aim PUBLIC include)
target_link_libraries(auto_aim
  PUBLIC Eigen3::Eigen ${OpenCV_LIBS} yaml-cpp
)

# ---- 主节点（传统装甲识别 + 自瞄解算）----
add_executable(armor_track_node
  src/auto_aim_node.cpp
  src/traditional_detector.cpp
)
ament_target_dependencies(armor_track_node rclcpp sensor_msgs cv_bridge)
target_include_directories(armor_track_node PRIVATE
  include
)
target_link_libraries(armor_track_node auto_aim ${OpenCV_LIBS} yaml-cpp)

# ---- 串口节点（订阅 gimbal_cmd -> 串口下发）----
add_executable(serial_node
  src/serial_node.cpp
  src/serial_driver.cpp
)
ament_target_dependencies(serial_node rclcpp)
target_include_directories(serial_node PRIVATE include)
target_link_libraries(serial_node yaml-cpp)

# ---- 大恒 Galaxy 相机驱动（发布 image_raw + camera_info）----
# 需要 libgxiapi.so（官方 Galaxy 驱动已装到 /usr/lib）与 ros-humble-camera-info-manager。
# 二者缺任一时跳过该节点，不影响其余节点构建。
find_package(camera_info_manager QUIET)
find_library(GXIAPI_LIB gxiapi)

if(camera_info_manager_FOUND AND GXIAPI_LIB)
  add_executable(daheng_camera_node
    src/daheng_camera_node.cpp
    src/daheng_camera.cpp
  )
  ament_target_dependencies(daheng_camera_node rclcpp sensor_msgs cv_bridge camera_info_manager)
  target_include_directories(daheng_camera_node PRIVATE include)
  target_link_libraries(daheng_camera_node ${GXIAPI_LIB} yaml-cpp)
  install(TARGETS daheng_camera_node DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING
    "跳过 daheng_camera_node：缺少 camera_info_manager 或 libgxiapi.so。"
    " 安装相机依赖: sudo apt install ros-humble-camera-info-manager")
endif()

# 链接生成的接口 typesupport（提供 armor_track_ros2/msg/gimbal_cmd.hpp）
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(armor_track_node "${cpp_typesupport_target}")
target_link_libraries(serial_node "${cpp_typesupport_target}")

# ---- 安装 ----
# 共享库放 lib/（setup.bash 会把 lib/ 加进 LD_LIBRARY_PATH），
# 可执行文件放 lib/${PROJECT_NAME}/（ROS2 标准布局）。混放会导致库找不到。
install(TARGETS auto_aim
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS armor_track_node serial_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch msg DESTINATION share/${PROJECT_NAME})

ament_package()
