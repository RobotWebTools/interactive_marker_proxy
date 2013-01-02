cmake_minimum_required(VERSION 2.8.3)
project(interactive_marker_proxy)

find_package(catkin REQUIRED COMPONENTS message_generation interactive_markers)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(${PROJECT_NAME} src/proxy.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}
  PROPERTIES OUTPUT_NAME proxy )

add_service_files(FILES srv/GetInit.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES visualization_msgs std_msgs geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS interactive_markers
)

