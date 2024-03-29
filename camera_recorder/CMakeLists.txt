cmake_minimum_required(VERSION 3.22.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(camera_recorder)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV 4 REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  include/${PROJECT_NAME}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
  ${PROJECT_NAME}
  ${OpenCV_LIBS}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN helios_cv::CameraRecorder
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING) 
  find_package(ament_lint_auto REQUIRED)
  list(
    APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()