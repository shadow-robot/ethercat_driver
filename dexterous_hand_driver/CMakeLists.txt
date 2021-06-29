cmake_minimum_required(VERSION 3.1)
project(dexterous_hand_driver)
set (CMAKE_CXX_STANDARD 11)

find_package(Boost COMPONENTS system REQUIRED)

include_directories(include)

include_directories(${Boost_INCLUDE_DIRS})

include_directories($ENV{HOME}/include)

add_library(${PROJECT_NAME} SHARED
    src/ethercat_hardware_hand_0220.cpp
    src/hand_driver_0220.cpp
    src/ethercat_bridge_driver.cpp
    src/calibration.cpp
)

target_link_libraries(${PROJECT_NAME} ethercat_hardware eml ${Boost_LIBRARIES}
  yaml)

add_library(ethercat_hardware STATIC IMPORTED)

set_property(TARGET ethercat_hardware PROPERTY IMPORTED_LOCATION
  $ENV{HOME}/lib/libethercat_hardware.so)

add_library(eml STATIC IMPORTED)

set_property(TARGET eml PROPERTY IMPORTED_LOCATION $ENV{HOME}/lib/libeml.so)

add_library(yaml STATIC IMPORTED)

set_property(TARGET yaml PROPERTY IMPORTED_LOCATION $ENV{HOME}/lib/libyaml.so)

add_library(${PROJECT_NAME}_interface SHARED
    src/hand_interface.cpp
)
target_link_libraries(${PROJECT_NAME}_interface ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_interface ${PROJECT_NAME})

add_executable(${PROJECT_NAME}_0220 src/main.cpp)
target_link_libraries(${PROJECT_NAME}_0220 ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_0220 ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_interface
   LIBRARY DESTINATION lib
)

install(TARGETS ${PROJECT_NAME}_0220 DESTINATION bin)

install(DIRECTORY include/${PROJECT_NAME}
   DESTINATION include
)
