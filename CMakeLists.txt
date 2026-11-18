cmake_minimum_required(VERSION 3.8)
project(shm_ros)

# Builds under both ROS2 (ament) and ROS1 (catkin), the same way astribot_camera
# does. The ROS1 build exists so the Noetic camera_image_fixer can publish
# ShmImage and ros1_bridge can generate its conversion factory. Only the message
# is built under catkin; the C++/Python libraries are ROS2-only.
find_package(ament_cmake QUIET)
find_package(catkin QUIET)

if(ament_cmake_FOUND)  # ROS2
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  find_package(ament_cmake_python REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/ShmImage.msg"
    DEPENDENCIES std_msgs
  )

  # Header-only core: consumers include it, nothing links.
  add_library(${PROJECT_NAME}_core INTERFACE)
  target_include_directories(${PROJECT_NAME}_core INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${PROJECT_NAME}_core INTERFACE cxx_std_17)

  install(DIRECTORY include/ DESTINATION include)
  install(TARGETS ${PROJECT_NAME}_core EXPORT export_${PROJECT_NAME})

  # shm_to_image: republishes ShmImage streams as sensor_msgs/Image for stock tools.
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
  add_library(shm_to_image_lib src/shm_to_image.cpp)
  target_include_directories(shm_to_image_lib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
  target_link_libraries(shm_to_image_lib ${PROJECT_NAME}_core "${cpp_typesupport_target}")
  ament_target_dependencies(shm_to_image_lib rclcpp sensor_msgs)
  add_executable(shm_to_image src/shm_to_image_main.cpp)
  target_link_libraries(shm_to_image shm_to_image_lib)
  install(TARGETS shm_to_image_lib shm_to_image
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

  # rosidl already generates a Python package named shm_ros (holding shm_ros.msg),
  # so ament_python_install_package would collide on the same target. Install our
  # module INTO that package instead: shm_ros.segment sits beside shm_ros.msg.
  # We deliberately ship no __init__.py — the generated one owns it.
  install(FILES ${PROJECT_NAME}/segment.py
    DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME})

  # ros1_bridge mapping rule (see package.xml export) — forces the ROS1<->ROS2
  # package pair, since the name does not end in _msgs.
  install(FILES mapping_rules.yaml DESTINATION share/${PROJECT_NAME})

  if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_layout_parity test/test_layout_parity.py)
    ament_add_pytest_test(test_segment test/test_segment.py)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_shm_to_image test/test_shm_to_image.cpp)
    target_link_libraries(test_shm_to_image shm_to_image_lib)
  endif()

  ament_export_include_directories(include)
  ament_export_targets(export_${PROJECT_NAME})
  ament_export_dependencies(rosidl_default_runtime std_msgs)
  ament_package()

elseif(catkin_FOUND)  # ROS1
  find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
  add_message_files(FILES
    ShmImage.msg
  )
  generate_messages(DEPENDENCIES std_msgs)
  catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

else()
  message(FATAL_ERROR "shm_ros: source a ROS env first (no ament_cmake or catkin found).")
endif()
