cmake_minimum_required(VERSION 3.0.2)
project(fusion_estimator)

## 编译选项：根据需要自行调整
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")

## 查找 catkin 依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  tf
  tf2
  tf2_geometry_msgs
  urdf
  message_generation
)

## 如果你有自定义消息(.msg 文件)，在这定义:
## 注意：如果你的包内还有 LowState.msg, RobotState.msg, JointState.msg 等，也要在这里列出
add_message_files(
  FILES
  FusionEstimatorTest.msg
  LowState.msg
  JointState.msg
)

## 如果消息依赖其他消息/ROS包(例如 std_msgs, geometry_msgs, sensor_msgs 等)，在这里声明:
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

## catkin_package 用于导出依赖和包含路径
catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    nav_msgs
    tf
    tf2
    tf2_geometry_msgs
    urdf
    message_runtime
)

## 搜索源文件，或者手动列出
file(GLOB_RECURSE ESTIMATOR_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)

## 包含头文件路径
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

## 创建可执行文件
add_executable(${PROJECT_NAME}_node
  ${ESTIMATOR_SOURCES}
)

## 链接库
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

## 如果你的节点需要用到生成的消息头文件，需要加上依赖消息生成
add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

## 安装可执行文件到 devel / install 路径
install(TARGETS
  ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)