cmake_minimum_required(VERSION 3.8)
project(nvsam)

add_definitions(-std=c++17)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ZED 4 REQUIRED)
find_package(CUDA REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(zed_components REQUIRED)

# include
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${ZED_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})

add_executable(nvsam src/nvsam.cpp)

target_include_directories(nvsam PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

set_target_properties(nvsam PROPERTIES
  INSTALL_RPATH "$ORIGIN/"
)

target_link_libraries(nvsam ${CMAKE_CURRENT_SOURCE_DIR}/lib/libnvsam_shared.so class_loader::class_loader)

ament_target_dependencies(nvsam rclcpp rclcpp_components zed_components)

add_definitions(-O2 -pthread)

target_compile_features(nvsam PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS nvsam
  DESTINATION lib/${PROJECT_NAME})

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libnvsam_shared.so
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  lib
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  cache
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
