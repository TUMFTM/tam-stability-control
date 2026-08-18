cmake_minimum_required(VERSION 3.12)

# CONFIGURE PKG HERE
# ==========================================

set(PROJECT_NAME stability_control_tam_cpp)

# # Collect all source files in the src directory
# file(GLOB_RECURSE PACKAGE_LIBRARY_PATH src/*.cpp)

set(PACKAGE_DEPENDENCIES 
param_management_cpp
tum_types_cpp
tum_helpers_cpp
tsl_logger_cpp
controller_helpers_cpp
vehicle_handler_cpp
)
# ============================================
# END OF CONFIGURATION

project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set compile options but exclude pedantic for Abseil
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
  
  # Add pedantic warning only to our target, not to dependencies
  set(PROJECT_COMPILE_OPTIONS -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
foreach(pkg_name ${PACKAGE_DEPENDENCIES})
  find_package(${pkg_name} REQUIRED)
endforeach()

## Build wrapper library
add_library(${PROJECT_NAME} SHARED
  src/esc.cpp
  src/slip_control.cpp
  src/slip_calculation.cpp
  src/abs.cpp
  src/tc.cpp
  src/countersteer.cpp
)

target_compile_options(${PROJECT_NAME} PRIVATE ${PROJECT_COMPILE_OPTIONS})

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

target_include_directories(${PROJECT_NAME}
PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)

# Install library
install(
  DIRECTORY include/
  DESTINATION include
)
# TODO: Add colcon ignore for mod_control

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME} # Here always project name.. above library name
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})
# Export ament targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries(tod_core::tod_core)
ament_export_dependencies(${PACKAGE_DEPENDENCIES}) # other dependencies so client does not need to call find_package on them

ament_package()
