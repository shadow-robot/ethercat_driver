cmake_minimum_required(VERSION 2.8.3)
project(ethercat_hardware)

find_package(Boost COMPONENTS system thread REQUIRED)

include_directories(include)

include_directories(${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
    src/ethercat_com.cpp
    src/ethercat_device.cpp
    src/ethercat_hardware.cpp
    src/ethernet_interface_info.cpp
)

target_link_libraries(${PROJECT_NAME} eml ${Boost_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}
   DESTINATION include
)
