cmake_minimum_required(VERSION 3.8)
project(px4_exec)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gz-transport12 REQUIRED)
find_package(gz-msgs9 REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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


# Build the messages:
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AprilTagDetection.msg"  
  DEPENDENCIES "std_msgs" "geometry_msgs"
)


# Build the ROS2 model pose publisher:
add_executable(model_pose_pub src/model_pose_pub.cpp)
# Add the dependencies:
ament_target_dependencies(model_pose_pub
  "rclcpp" 
  "geometry_msgs"
)
# Linkj the gz libraries:
target_link_libraries(model_pose_pub
  "gz-transport12::gz-transport12"
  "gz-msgs9::gz-msgs9"
)
# Intall the executable:
install(TARGETS model_pose_pub
  DESTINATION lib/${PROJECT_NAME}
)



# Build the MAVSDK code to start the server where the FOLLOW ME system is working:
# Define the MAVSDK local install preffix:
set(MAVSDK_PREFIX "$ENV{HOME}/.local/mavsdk")
set(MAVSDK_INCLUDE_DIR "${MAVSDK_PREFIX}/include")
set(MAVSDK_INCLUDE_DIR2 "${MAVSDK_PREFIX}/include/mavsdk")
set(MAVSDK_LIB_DIR "${MAVSDK_PREFIX}/lib")
# Define the location fo teh code:
add_executable(mavsdk_follow_agent src/mavsdk_follow_agent.cpp)
# Compiler options:
target_compile_options(mavsdk_follow_agent PRIVATE
  -Wno-pedantic
  -Wno-address-of-packed-member
)
# Indlude the directories of teh include directoreis:
target_include_directories(mavsdk_follow_agent PRIVATE
  ${MAVSDK_INCLUDE_DIR}
  ${MAVSDK_INCLUDE_DIR2}
)
# Include MAVSDK libraries:
target_link_directories(mavsdk_follow_agent PRIVATE
  ${MAVSDK_LIB_DIR}
)
# Link the libraries tot eh code:
target_link_libraries(mavsdk_follow_agent
  "mavsdk"
  "pthread"
)
# Set the directies of libmavsdk.so at runtime:
set_target_properties(mavsdk_follow_agent PROPERTIES
  BUILD_RPATH "${MAVSDK_LIB_DIR}"
  INSTALL_RPATH "${MAVSDK_LIB_DIR}"
)
# Intall the executable:
install(TARGETS mavsdk_follow_agent
  DESTINATION lib/${PROJECT_NAME}
)


# Add the ROS2 messages:
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
# Build the landing strategy that uses PX4:
add_executable(landing_strategy src/landing_strategy.cpp)
# Target it with the linked libarires:
target_link_libraries(landing_strategy
  ${cpp_typesupport_target}
)
# Add the dependencies:
ament_target_dependencies(landing_strategy
  "tf2"
  "rclcpp" 
  "geometry_msgs"
  "sensor_msgs"
  "px4_msgs"
)
# Intall the executable:
install(TARGETS landing_strategy
  DESTINATION lib/${PROJECT_NAME}
)


# Find the paths for the AprilTag library:
find_path(APRILTAG_INCLUDE_DIR apriltag.h PATH_SUFFIXES apriltag)
find_library(APRILTAG_LIBRARY NAMES apriltag)
# Include the directories:
include_directories(
  include
  ${APRILTAG_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
# Add the ROS2 messages:
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
# Build the AprilTag detector Node:
add_executable(apriltag_detector src/apriltag_detector.cpp)
# Target it with the linked libarires:
target_link_libraries(apriltag_detector
  ${APRILTAG_LIBRARY}
  ${OpenCV_LIBRARIES}
  ${cpp_typesupport_target}
)
# Add the dependencies:
ament_target_dependencies(apriltag_detector
  "rclcpp" 
  "sensor_msgs"
  "cv_bridge"
)
# Intall the executable:
install(TARGETS apriltag_detector
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
