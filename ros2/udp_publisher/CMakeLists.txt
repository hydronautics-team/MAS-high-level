cmake_minimum_required(VERSION 3.5)
project(udp_publisher)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost COMPONENTS system REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/FromBort.msg"
  "msg/ToBort.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(publisher
  src/publisher.cpp
)

ament_target_dependencies(publisher rclcpp Boost)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(publisher "${cpp_typesupport_target}")

target_include_directories(publisher
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
    $<INSTALL_INTERFACE:include>
)

add_executable(control
  src/control.cpp
)
ament_target_dependencies(control rclcpp)
target_link_libraries(control "${cpp_typesupport_target}")

install(TARGETS
  publisher
  control
  DESTINATION lib/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
