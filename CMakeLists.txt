cmake_minimum_required(VERSION 3.5)
project(fusion_localizer)

add_subdirectory(backward-cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic )
endif()

find_package(Eigen3 REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


include_directories(include ${EIGEN3_INCLUDE_DIRS})

ament_auto_add_library(fusion_localizer SHARED
  src/fusion_localizer.cpp
)

rclcpp_components_register_node(fusion_localizer
  PLUGIN "FusionLocalizer"
  EXECUTABLE fusion_localizer_node
)

target_link_libraries(fusion_localizer ${EIGEN3_LIBRARIES})

add_backward(fusion_localizer)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)

install(FILES DESTINATION share/${PROJECT_NAME})
