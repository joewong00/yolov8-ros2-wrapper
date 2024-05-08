cmake_minimum_required(VERSION 3.8)
project(yolov8_wrapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
)

# -------------- libtorch --------------
# If using x86 PC, uncomment the following two lines
# list(APPEND CMAKE_PREFIX_PATH "/home/nvidia/libtorch")
# set(Torch_DIR "/home/nvidia/libtorch/share/cmake/Torch")

# If using ARM based (Jetson), use this line and change the path
list(APPEND CMAKE_PREFIX_PATH "/home/nvidia/.local/lib/python3.8/site-packages/torch")

find_package(Torch REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${TORCH_INCLUDE_DIRS})

add_executable(image_detector src/detect_node.cpp src/detect.cpp)

target_link_libraries(image_detector ${TORCH_LIBRARIES} ${OpenCV_LIBS})

ament_target_dependencies(image_detector
    rclcpp
    sensor_msgs
    cv_bridge
  )

install(TARGETS image_detector
    DESTINATION lib/${PROJECT_NAME})  

install(DIRECTORY
    launch
    weights
    DESTINATION share/${PROJECT_NAME}/
  )

install(
    DIRECTORY src/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.hpp"
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
