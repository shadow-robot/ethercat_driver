cmake_minimum_required(VERSION 2.8.3)
project(ethercat_hardware)

# find_package(catkin REQUIRED COMPONENTS
#     roscpp
#     realtime_tools
#     diagnostic_msgs
#     diagnostic_updater
#     pluginlib
#     hardware_interface
#     ros_ethercat_eml
# )

include_directories(include
)

add_library(${PROJECT_NAME} SHARED
    src/ethercat_com.cpp
    src/ethercat_device.cpp
    src/ethercat_hardware.cpp
    src/ethernet_interface_info.cpp
)
# add_dependencies(${PROJECT_NAME}
# )
target_link_libraries(${PROJECT_NAME}
)

#include(cmake/${PROJECT_NAME}-extras.cmake.em)
#ros_enable_rpath(${PROJECT_NAME})


install(TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}
   DESTINATION include
)
