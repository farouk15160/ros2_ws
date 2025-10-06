cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(gazebo_ros REQUIRED)

# ===================================================================
# === THE FINAL FIX: INSTALL ALL ASSET DIRECTORIES ===
# These commands ensure that your launch, urdf, mesh, config, and world
# files are all copied to the install directory where ROS2 can find them.
# ===================================================================

# Install the launch directory
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the URDF directory
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install the meshes directory
install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install the config directory
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install the worlds directory
install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)
cmake_minimum_required(VERSION 3.8)
project(my_robot_description) # <-- This must be present and near the top!

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ... rest of your CMakeLists.txt
ament_package()
