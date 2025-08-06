cmake_minimum_required(VERSION 3.5)
project(learm_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)


# Generate only your AddTwoInts service—no extra dependencies needed
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddTwoInts.srv"
  "srv/EulerToQuaternion.srv"
  "srv/QuaternionToEuler.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
