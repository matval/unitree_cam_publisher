cmake_minimum_required(VERSION 3.5)
project(unitree_cameras)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV 4 REQUIRED)

if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIR})
    message(STATUS "OpenCV ${OpenCV_VERSION} FOUND")
    message(STATUS ${OpenCV_LIBS})
    if(${OpenCV_VERSION_MAJOR} EQUAL 4)
        add_definitions(-DOPENCV_VERSION_4)
    elseif(${OpenCV_VERSION_MAJOR} LESS 4)
        add_definitions(-DOPENCV_VERSION_3)
    endif()
else()
    message(FATAL_ERROR "OpenCV Library Not Found")
endif()

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/UnitreecameraSDK/lib/amd64/)
  link_directories(${CMAKE_SOURCE_DIR}/UnitreecameraSDK/lib/amd64/)
else()
  set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/UnitreecameraSDK/lib/arm64/)
  link_directories(${CMAKE_SOURCE_DIR}/UnitreecameraSDK/lib/arm64/)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

include_directories(
    include
    ${CMAKE_SOURCE_DIR}/UnitreecameraSDK/include
)

set(SDKLIBS unitree_camera tstc_V4L2_xu_camera udev systemlog ${OpenCV_LIBS})

install(
    DIRECTORY ${CMAKE_SOURCE_DIR}/UnitreecameraSDK/lib/
    DESTINATION lib
)

link_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/UnitreecameraSDK/lib
)

add_executable(cameras_node src/cameras_node.cpp)
target_link_libraries(cameras_node ${SDKLIBS})
ament_target_dependencies(cameras_node rclcpp OpenCV)

install(TARGETS cameras_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
