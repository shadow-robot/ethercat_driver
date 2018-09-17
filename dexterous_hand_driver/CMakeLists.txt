cmake_minimum_required(VERSION 2.8.3)
project(dexterous_hand_driver)

find_package(Boost COMPONENTS system REQUIRED)

include_directories(include)

include_directories(${Boost_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED
    src/ethercat_hardware_hand_0220.cpp
    src/hand_driver_0220.cpp
    src/ethercat_bridge_driver.cpp
)

target_link_libraries(${PROJECT_NAME} ethercat_hardware eml ${Boost_LIBRARIES})

add_executable(${PROJECT_NAME}_0220 src/main.cpp)
target_link_libraries(${PROJECT_NAME}_0220 ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_0220 ${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION lib
)

install(TARGETS ${PROJECT_NAME}_0220 DESTINATION bin)

install(DIRECTORY include/${PROJECT_NAME}
   DESTINATION include
)
