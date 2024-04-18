cmake_minimum_required(VERSION 3.14)
project(loam_mapper)

add_compile_options(-Wall -Wextra -Wpedantic)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PcapPlusPlus REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED)

include_directories(include
        ${PCL_INCLUDE_DIRS}
        ${PcapPlusPlus_INCLUDE_DIRS})

set(LOAM_MAPPER_LIB_SRC
        src/utils.cpp
        src/points_provider.cpp
        src/transform_provider.cpp
        src/loam_mapper.cpp)

set(LOAM_MAPPER_LIB_HEADERS
        include/loam_mapper/utils.hpp
        include/loam_mapper/date.h
        include/loam_mapper/csv.hpp
        include/loam_mapper/point_types.hpp
        include/loam_mapper/Occtree.h
        include/loam_mapper/points_provider.hpp
        include/loam_mapper/transform_provider.hpp
        include/loam_mapper/loam_mapper.hpp)

add_executable(${PROJECT_NAME}
        ${LOAM_MAPPER_LIB_SRC}
        ${LOAM_MAPPER_LIB_HEADERS})
ament_target_dependencies(${PROJECT_NAME} rclcpp PcapPlusPlus PCL geometry_msgs)
target_link_libraries(${PROJECT_NAME}
        ${PCL_LIBRARIES}
        ${PcapPlusPlus_LIBRARIES})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif ()

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

 install(DIRECTORY
         launch
         DESTINATION share/${PROJECT_NAME}/
 )
 install(DIRECTORY
         config
         DESTINATION share/${PROJECT_NAME}/
 )

ament_auto_package()
