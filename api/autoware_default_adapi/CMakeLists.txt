cmake_minimum_required(VERSION 3.14)
project(autoware_default_adapi)

find_package(autoware_cmake REQUIRED)
find_package(autoware_agnocast_wrapper REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/interface.cpp
  src/localization.cpp
  src/routing.cpp
  src/utils/localization_conversion.cpp
  src/utils/route_conversion.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
  ${autoware_agnocast_wrapper_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  autoware_agnocast_wrapper
)

autoware_agnocast_wrapper_setup(${PROJECT_NAME})

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::InterfaceNode"
  EXECUTABLE interface_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR SingleThreadedAgnocastExecutor)
autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::LocalizationNode"
  EXECUTABLE localization_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR SingleThreadedAgnocastExecutor)
autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::RoutingNode"
  EXECUTABLE routing_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR SingleThreadedAgnocastExecutor)

if(BUILD_TESTING AND (NOT "$ENV{ENABLE_AGNOCAST}" STREQUAL "1"))
  add_launch_test(test/main.test.py)

  ament_auto_add_gtest(test_${PROJECT_NAME}
    test/test_conversion.cpp
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE src)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
endif()

autoware_ament_auto_package(INSTALL_TO_SHARE config launch test)
