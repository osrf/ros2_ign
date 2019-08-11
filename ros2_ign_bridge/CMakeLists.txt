cmake_minimum_required(VERSION 3.5)

project(ros2_ign_bridge)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(ignition-msgs4 QUIET REQUIRED)
set(IGN_MSGS_VER ${ignition-msgs4_VERSION_MAJOR})

find_package(ignition-transport7 QUIET REQUIRED)
set(IGN_TRANSPORT_VER ${ignition-transport7_VERSION_MAJOR})

include_directories(include)

set(common_sources
  src/convert_builtin_interfaces.cpp
  src/builtin_interfaces_factories.cpp
)

set(bridge_executables
  static_bridge
)

foreach(bridge ${bridge_executables})
  add_executable(${bridge}
    src/${bridge}.cpp
    ${common_sources}
  )
  target_link_libraries(${bridge}
    ignition-msgs${IGN_MSGS_VER}::core
    ignition-transport${IGN_TRANSPORT_VER}::core
  )
  ament_target_dependencies(${bridge}
    "rclcpp"
    "std_msgs"
  )
  install(TARGETS ${bridge}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
