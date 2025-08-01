# Define a versão mínima do CMake (ROS 2 exige uma versão mais nova)
cmake_minimum_required(VERSION 3.8)

# Nome do projeto
project(map_launcher)

# Encontra as dependências necessárias do ament_cmake
find_package(ament_cmake REQUIRED)
find_package(ament_lint_auto REQUIRED)
find_package(ament_lint_common REQUIRED)

# Adiciona um executável (exemplo comentado, caso você adicione código C++)
# add_executable(map_launcher_node src/map_launcher_node.cpp)
# ament_target_dependencies(map_launcher_node rclcpp)

#################
## INSTALL     ##
#################
# Em ROS 2, é MANDATÓRIO instalar todos os arquivos que serão usados em tempo de execução.

# Instala o package.xml
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Instala o diretório de launch files
# Assumindo que você tem uma pasta "launch" no seu pacote
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Instala o diretório de mapas (exemplo)
# Assumindo que você tem uma pasta "maps"
# install(
#   DIRECTORY maps
#   DESTINATION share/${PROJECT_NAME}/
# )

# Instala o diretório de configuração (exemplo)
# Assumindo que você tem uma pasta "config" ou "params"
# install(
#   DIRECTORY config
#   DESTINATION share/${PROJECT_NAME}/
# )

# Marca o pacote para ser encontrado pelo ROS 2
ament_index_register_resource(
  "ros_distro_dependent" "true"
)

#################
## TESTING     ##
#################
# Adiciona testes de linting (boas práticas em ROS 2)
if(BUILD_TESTING)
  ament_lint_auto_find_test_dependencies()
endif()

# Finaliza o pacote. Deve ser a última função chamada.
ament_package()