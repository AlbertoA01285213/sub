cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0009 NEW)
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

#add_subdirectory(libs/src) # No se como se ponga bien
add_library(uuv_model_lib
  libs/src/models/uuv_dynamic_model.cpp)

target_include_directories(uuv_model_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/src
  ${EIGEN3_INCLUDE_DIRS})

# ============= Para nodos python ==========================

ament_python_install_package(${PROJECT_NAME}) # Esta comentado pq cuando esta activado no compila jeje

install(PROGRAMS
  src/pid.py
  src/posicion.py
  src/visualizador.py
  src/objetivo.py
  src/joystick.py

  src/maquina_estados_1.py

  src/misiones/cuadrado/obj_cuadrado_1.py
  src/misiones/cuadrado/obj_cuadrado_2.py
  src/misiones/cuadrado/obj_cuadrado_3.py
  src/misiones/cuadrado/obj_cuadrado_4.py
  DESTINATION lib/${PROJECT_NAME})

# ============= Para nodos c++ ========================

# add_executable(posicion src/posicion.cpp)
# ament_target_dependencies(posicion rclcpp std_msgs geometry_msgs)
# install(TARGETS
#   posicion
#   DESTINATION lib/${PROJECT_NAME})

add_executable(dynamic_model_uuv src/dynamic_model_uuv.cpp)
target_include_directories(dynamic_model_uuv PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/uuv_libs
  ${CMAKE_CURRENT_SOURCE_DIR}/libs/src
  )
target_compile_features(dynamic_model_uuv PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(dynamic_model_uuv ${DEPENDENCIES})
# target_link_libraries(dynamic_model_uuv uuv_libs)
ament_target_dependencies(dynamic_model_uuv
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  sensor_msgs
  Eigen3
)
target_link_libraries(dynamic_model_uuv uuv_model_lib)

#add_executable(objetivo src/objetivo.cpp)
#ament_target_dependencies(objetivo rclcpp std_msgs geometry_msgs)
#install(TARGETS
#  objetivo
#  dynamic_model_uuv
#  DESTINATION lib/${PROJECT_NAME})

# ============== Launchfiles ===============================
install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY 
  models
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
