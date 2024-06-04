cmake_minimum_required(VERSION 3.0.2)
project(convex_polyhedron_intersection)

set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  cddeigen)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES convex_polyhedron_intersection
  CATKIN_DEPENDS cddeigen
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(${PROJECT_NAME}
  src/convex_polyhedron_intersection.cpp
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
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
