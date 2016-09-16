cmake_minimum_required(VERSION 2.8.3)
project(rviz_ortho_view_controller)

# Package
set(CATKIN_DEPENDENCIES pluginlib roscpp rviz)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDENCIES})
find_package(Qt4 REQUIRED QtGui)

catkin_package(INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
               DEPENDS Qt4)

# Build
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${CMAKE_CURRENT_BINARY_DIR}
                    ${QT_INCLUDE_DIR})

qt4_wrap_cpp(MOC_FILES include/${PROJECT_NAME}/ortho_view_controller.h)
add_library(${PROJECT_NAME} src/ortho_view_controller.cpp ${MOC_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Qt4::QtGui)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-std=c++11")

# Install
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")

install(FILES plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
