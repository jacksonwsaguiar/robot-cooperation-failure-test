cmake_minimum_required(VERSION 3.8)
project(turtlebot3_description)

find_package(ament_cmake REQUIRED)

# Instala o package.xml
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Instala todos os diretórios de recursos
install(
  DIRECTORY urdf meshes rviz world
  DESTINATION share/${PROJECT_NAME}
)

ament_package()