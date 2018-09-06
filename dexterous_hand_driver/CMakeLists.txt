cmake_minimum_required(VERSION 2.8.3)
project(dexterous_hand_driver)

include_directories(include
)

add_library(${PROJECT_NAME} SHARED
    src/main.cpp
    src/ethercat_hardware_hand_0220.cpp
    src/hand_driver_0220.cpp
)
# add_dependencies(${PROJECT_NAME}
# )
target_link_libraries(${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
   LIBRARY DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}
   DESTINATION include
)
