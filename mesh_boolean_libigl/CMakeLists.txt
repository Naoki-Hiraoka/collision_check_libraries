cmake_minimum_required(VERSION 3.16)
project(mesh_boolean_libigl)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  qhulleigen)

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Libigl
find_package(CGAL REQUIRED COMPONENTS Core) # CGALは既にインストールされているものを用いる. そうしないとdevel以下にboostがインストールされてしまう.
include(libigl)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mesh_boolean_libigl
  )

# Enable the target igl_copyleft::cgal
igl_include(copyleft cgal)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME}
  src/mesh_boolean_libigl.cpp
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  igl_copyleft::cgal
  )

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )

############
## Sample ##
############

add_executable(sample src/sample.cpp)
target_link_libraries(sample
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
install(TARGETS sample
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
