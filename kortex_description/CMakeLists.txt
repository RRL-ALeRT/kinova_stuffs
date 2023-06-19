cmake_minimum_required(VERSION 3.5)
project(kortex_description)

# Find packages
find_package(ament_cmake REQUIRED)

# Install
install(DIRECTORY arms grippers robots launch
  DESTINATION share/${PROJECT_NAME}
)

# Create ROS 2 package
ament_package()
