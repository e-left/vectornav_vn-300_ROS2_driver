cmake_minimum_required(VERSION 3.8)
project(turtle_vn)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-w)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtle_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

# vn library
add_subdirectory(include/vn libvncxx)
# file(GLOB_RECURSE vn_sources ${CMAKE_CURRENT_SOURCE_DIR}/include/vn/src/*.cpp)
# add_library(libvncxx ${vn_sources})
# target_include_directories(libvncxx PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/vn/include>
#   $<INSTALL_INTERFACE:include/vn/include>)

# publisher library
add_library(VNPub src/vn_publisher.cpp)
target_include_directories(VNPub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(VNPub libvncxx pthread)
ament_target_dependencies(VNPub rclcpp tf2_ros turtle_interfaces std_msgs std_srvs nav_msgs geometry_msgs) 

# final executable
add_executable(turtle_vn src/turtle_vn.cpp)
target_include_directories(turtle_vn PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(turtle_vn VNPub)
ament_target_dependencies(turtle_vn rclcpp)

install(TARGETS turtle_vn
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/launch
    DESTINATION share/${PROJECT_NAME}
    USE_SOURCE_PERMISSIONS)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
