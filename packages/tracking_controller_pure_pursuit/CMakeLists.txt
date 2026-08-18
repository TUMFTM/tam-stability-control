cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME tracking_controller_pure_pursuit)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


set(PACKAGE_DEPENDENCIES
param_management_cpp
tum_types_cpp
tum_helpers_cpp
controller_helpers_cpp
limit_handler_cpp
tsl_logger_cpp
vehicle_handler_cpp
)

foreach(pkg_name ${PACKAGE_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

## Build wrapper library
add_library(${PROJECT_NAME} SHARED src/tracking_controller_pure_pursuit.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})

# Install library
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME} # Here always project name.. above library name
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export ament targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries(tod_core::tod_core)

ament_export_dependencies(${PACKAGE_DEPENDENCIES}) # other dependencies so client does not need to call find_package on them

ament_export_include_directories(include) # superfluous: handeled by INCLUDES DESTINATION

# if launchfiles
#install(
#    DIRECTORY launch
#    DESTINATION share/${PROJECT_NAME}
#)
#
#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  ament_lint_auto_find_test_dependencies()
#
#  ament_add_gtest(unit_tests test/unit_test.cpp) # unit tests
#  target_link_libraries(unit_tests ${PROJECT_NAME})
#endif()

ament_package()
