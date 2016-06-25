cmake_minimum_required(VERSION 2.8.3)

## enable c++11 mode
set(CMAKE_CXX_COMPILER_ARG1 -std=c++11)

project(path_array_rviz_plugin)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  nav_msgs
  roscpp
  rviz
  std_msgs
)

# this setting causes Qt's "MOC" generation to happen automatically
set(CMAKE_AUTOMOC ON)

## this plugin includes Qt widgets, so we must include Qt.
## we'll use the version that rviz used so they are compatible
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

## for the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here
add_definitions(-DQT_NO_KEYWORDS)

## add message, serviece and action files from msg, srv and action directories

add_message_files(DIRECTORY msg
  FILES
    Path.msg
    PathArray.msg
)

# add_service_files(
#   FILES
#     Service1.srv
# )

# add_action_files(
#   FILES
#     Action1.action
# )

# generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    nav_msgs
    std_msgs
)

## add dynamic reconfigure config files from cfg directory
# generate_dynamic_reconfigure_options(
#   DynReconf1.cfg
# )

# declare catkin package
catkin_package(
  # INCLUDE_DIRS
  #   include
  LIBRARIES
    path_array_rviz_plugin
  CATKIN_DEPENDS
    message_runtime
    nav_msgs
    std_msgs
  # DEPENDS
  #   system_lib
)



## build ##

# include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

## declare a c++ library
add_library(${PROJECT_NAME}
  src/path_array_display.cpp
)

## cmake target dependencies of the c++ library
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## libraries to link the target c++ library against
link_directories(${catkin_LIBRARY_DIRS})
target_link_libraries(${PROJECT_NAME}
  ${QT_LIBRARIES} ${catkin_LIBRARIES}
  ${catkin_LIBRARIES}
)

## declare a c++ executable
# add_executable(${PROJECT_NAME}_node
#   src/path_array_rviz_plugin_node.cpp
# )

## rename c++ executable without prefix, for shorter version
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## cmake target dependencies of the c++ executable
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## libraries to link the target c++ executable against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )



## install ##

## executable scripts (python etc.) for installation
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## cpp-header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## other files for installation (launch, data, bag files)
# foreach(dir launch data bags)
#    install(DIRECTORY ${dir}/
#       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

## other files for installation
install(
  FILES
    plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



## testing ##

## add catch based cpp test target
# catkin_add_catchtest(${PROJECT_NAME}-test test/test_path_array_rviz_plugin.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## add nosetest file folders
# catkin_add_nosetests(test)
