cmake_minimum_required(VERSION 3.16)
project(dt_viz)

# Define o padrão da linguagem C++.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Ativa avisos úteis durante a compilação.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# Dependências do ROS 2
# ============================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(dt_msgs REQUIRED)

# ============================================================
# Dependências da interface gráfica
# ============================================================

find_package(Qt5 REQUIRED COMPONENTS Widgets)

# ============================================================
# Executável da interface
# ============================================================

add_executable(dt_visualizer_node
  src/main.cpp
  src/main_window.cpp
  src/viz_node.cpp
)

# Disponibiliza os arquivos do diretório include.
target_include_directories(dt_visualizer_node
  PRIVATE
    include
)

# Dependências ROS 2 utilizadas pelo executável.
ament_target_dependencies(dt_visualizer_node
  rclcpp
  nav_msgs
  dt_msgs
)

# Vincula a aplicação ao módulo Qt Widgets.
target_link_libraries(dt_visualizer_node
  Qt5::Widgets
)

# ============================================================
# Instalação
# ============================================================

install(
  TARGETS dt_visualizer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

ament_package()