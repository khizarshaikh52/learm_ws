cmake_minimum_required(VERSION 3.5)
project(learm_vision_lite)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Install your executable script
install(PROGRAMS
  scripts/crack_tuner_ros.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the helper module (so it sits next to the script at runtime)
install(FILES
  scripts/crack_tuner.py
  DESTINATION lib/${PROJECT_NAME}
)


# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

