# Copyright 2026 Phillip Pitschi
cmake_minimum_required(VERSION 3.12)

# CONFIGURE PKG HERE
# ==========================================

set(PROJECT_NAME stability_controller_node_cpp)
set(NODE_LIBRARY_PATHS src/stability_controller_node.cpp)
set(COMPONENT_PATH components/stability_controller_node.cpp)
set(COMPONENT_NAME ${PROJECT_NAME}::StabilityControllerNode)
set(EXECUTABLE_NAME stability_controller_node)


set(NODE_DEPENDENCIES
  rclcpp
  nav_msgs
  geometry_msgs
  autoware_auto_vehicle_msgs
  autoware_auto_control_msgs
  tum_msgs
  tum_ros_helpers_cpp
  tum_type_conversions_ros_cpp
  param_management_ros2_integration_cpp
  param_management_cpp
  tsl_logger_cpp
  tsl_ros2_publisher_cpp
  ros2_watchdog_cpp
  vehicle_handler_cpp
  stability_control_tam_cpp
)
# ============================================
# END OF CONFIGURATION


project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
foreach(pkg_name ${NODE_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

# section: BUILD LIBRARY TARGET TO IMPORT NODE CLASS WITHIN OTHER PKGS
# ====================================================================

add_library(${PROJECT_NAME} SHARED ${NODE_LIBRARY_PATHS})

target_include_directories(${PROJECT_NAME}
    PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_target_dependencies(${PROJECT_NAME} ${NODE_DEPENDENCIES})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

add_library(${PROJECT_NAME}_components SHARED ${COMPONENT_PATH})
find_package(rclcpp_components REQUIRED)

ament_target_dependencies(${PROJECT_NAME}_components rclcpp_components)
target_link_libraries(${PROJECT_NAME}_components ${PROJECT_NAME})

rclcpp_components_register_node(
  ${PROJECT_NAME}_components
  PLUGIN ${COMPONENT_NAME}
  EXECUTABLE ${EXECUTABLE_NAME}
  )


ament_export_targets(export_${PROJECT_NAME}_components) # so client can also call target_link_libraries(template_lib_cpp::template_lib_cpp)

install(TARGETS ${PROJECT_NAME}_components
        EXPORT export_${PROJECT_NAME}_components
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_dependencies(rclcpp_components ${NODE_DEPENDENCIES}) # other dependencies so client does not need to call find_package on them
ament_package()
