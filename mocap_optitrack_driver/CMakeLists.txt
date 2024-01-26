cmake_minimum_required(VERSION 3.5)

# project(mocap_optitrack_driver)
project(optitrack_serial)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(LIBNATNET_SDK_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/NatNetSDK/lib/libNatNet.so)
#set(NATNET_INCLUDE_DIR
#  ${CMAKE_CURRENT_SOURCE_DIR}/NatNetSDK/include
#)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mocap4r2_control_msgs REQUIRED)
find_package(mocap4r2_control REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_ros
  mocap4r2_msgs
  mocap4r2_control
  mocap4r2_control_msgs
  geometry_msgs
)

include_directories(
  include
#  ${NATNET_INCLUDE_DIR}
)

#execute_process(
#  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/install_sdk.py
#  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#)

#add_library(${PROJECT_NAME}
#  src/mocap_optitrack_driver/mocap_optitrack_driver.cpp)

#ament_target_dependencies(${PROJECT_NAME} ${dependencies})
#target_compile_definitions(${PROJECT_NAME}
#  PRIVATE "OPTITRACK_BUILDING_LIBRARY")

#add_executable(mocap_optitrack_driver_main
#  src/mocap_optitrack_driver_main.cpp
#)
#ament_target_dependencies(mocap_optitrack_driver_main ${dependencies})
#target_link_libraries(mocap_optitrack_driver_main
#  ${PROJECT_NAME}
#  ${LIBNATNET_SDK_LIBRARY}
#)

add_executable(rf_receiver
  src/rf_receiver.cpp
)
ament_target_dependencies(rf_receiver ${dependencies})
target_link_libraries(rf_receiver
  #${PROJECT_NAME} 
#  rclcpp geometry_msgs
#  ${LIBNATNET_SDK_LIBRARY}
)

# ament_target_dependencies(mocap_optitrack_driver ${dependencies})
# target_link_libraries(mocap_optitrack_driver ${LIBNATNET_SDK_LIBRARY})

#ament_target_dependencies(optitrack_serial ${dependencies})
#target_link_libraries(optitrack_serial ${LIBNATNET_SDK_LIBRARY})

# install(DIRECTORY include/${PROJECT_NAME}
#install(DIRECTORY include/mocap_optitrack_driver
#  DESTINATION include/
#)

install(DIRECTORY include/opti_serial
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
#  mocap_optitrack_driver_main
  rf_receiver
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#install(
#  TARGETS ${PROJECT_NAME}
#  EXPORT  ${PROJECT_NAME}Targets
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION bin
#  INCLUDES DESTINATION include
#)


if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(ament_cmake_uncrustify REQUIRED)


#  file(GLOB_RECURSE EXCLUDE_FILES NatNetSDK/*)

  ament_copyright(EXCLUDE ${EXCLUDE_FILES})
  ament_cppcheck(
    EXCLUDE ${EXCLUDE_FILES}
    LANGUAGE c++
  )
  ament_cpplint(EXCLUDE ${EXCLUDE_FILES})
  ament_lint_cmake()
  ament_uncrustify(
    EXCLUDE ${EXCLUDE_FILES}
    LANGUAGE c++
  )

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
