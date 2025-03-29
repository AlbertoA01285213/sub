cmake_minimum_required(VERSION 3.8)
project(sub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)  # Añadir esta línea

#add_subdirectory(libs/uuv_libs)

# ============= Para nodos python ==========================

ament_python_install_package(${PROJECT_NAME}) # Esta comentado pq cuando esta activado no compila jeje

install(PROGRAMS
  src/pid.py
  src/posicion.py
  DESTINATION lib/${PROJECT_NAME})

# ============= Para nodos c++ ========================

# add_executable(posicion src/posicion.cpp)
# ament_target_dependencies(posicion rclcpp std_msgs geometry_msgs)
# install(TARGETS
#   posicion
#   DESTINATION lib/${PROJECT_NAME})

add_executable(objetivo src/objetivo.cpp)
ament_target_dependencies(objetivo rclcpp std_msgs geometry_msgs)
install(TARGETS
  objetivo
  DESTINATION lib/${PROJECT_NAME})

# ============== Lunchfiles ===============================
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
