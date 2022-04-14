cmake_minimum_required(VERSION 3.8)
project(nav2_ceiling_perception_plugin)

set(lib_name ${PROJECT_NAME}_core)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_roi_srv REQUIRED)

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib
    sensor_msgs
    geometry_msgs
    std_msgs
    custom_roi_srv
    )


# === Build ===

add_library(${lib_name} SHARED
        src/ceiling_layer.cpp)
include_directories(include)

# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)
install(DIRECTORY include/
        DESTINATION include/
        )
# === Ament work ===

# pluginlib_export_plugin_description_file() installs ceiling_layer.xml
# file into "share" directory and sets ament indexes for it.
# This allows the plugin to be discovered as a plugin of required type.
ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_costmap_2d ceiling_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})

ament_package()
