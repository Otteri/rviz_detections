cmake_minimum_required(VERSION 3.10)
project(rviz_detections CXX)

## Find catkin libraries
set(${PROJECT_NAME}_CATKIN_DEPS
  roscpp
  rosmsg
  std_msgs
  geometry_msgs
  message_generation
)

## Find dependencies
find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS}
             )

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/msg)

## Generate added messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

set(SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/bbox.cpp
)

set(PROJECT_LIBS
    ${catkin_LIBRARIES}
    )

add_executable(${PROJECT_NAME} ${SRC})

target_link_libraries(${PROJECT_NAME} ${PROJECT_LIBS})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
