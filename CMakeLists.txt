cmake_minimum_required(VERSION 3.5)
project(particle_filter_localization)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED system)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(pf_localization_component SHARED
src/pf_localization_component.cpp
)

target_compile_definitions(pf_localization_component PRIVATE "PFL_BUILDING_DLL")
ament_target_dependencies(pf_localization_component
  rclcpp rclcpp_components nav_msgs sensor_msgs tf2 tf2_eigen tf2_geometry_msgs pcl_conversions)

add_executable(pf_localization_node
src/pf_localization_node.cpp
src/pf.cpp
)

target_link_libraries(pf_localization_node
pf_localization_component ${PCL_LIBRARIES})

ament_target_dependencies(pf_localization_node
  rclcpp rclcpp_components nav_msgs sensor_msgs tf2 tf2_eigen tf2_geometry_msgs pcl_conversions)

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
  )
  link_directories(
    ${PCL_LIBRARY_DIRS}
    ${Boost_LIBRARY_DIRS}
  )
  add_definitions(${PCL_DEFINITIONS})

rclcpp_components_register_nodes(pf_localization_component 
  "particle_filter_localization::PfLocalizationComponent")

# install executables/libs
install(
  DIRECTORY "include/"
  DESTINATION include
)

install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  pf_localization_node
  DESTINATION lib/particle_filter_localization
)

install(TARGETS
  pf_localization_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


# export
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(pcl_conversions)
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
