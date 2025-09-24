PROJECT(pointcloud_to_laserscan)
CMAKE_MINIMUM_REQUIRED(VERSION 3.8)

MESSAGE(STATUS "Compilers found: ${CMAKE_CXX_COMPILER_LIST}")
MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

# Auto-detect ROS version from environment (amrl_msgs style)
if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
else()
  message(FATAL_ERROR "ROS_VERSION is not defined")
endif()

if(${ROS_VERSION} EQUAL "1")
  MESSAGE(STATUS "Compiling with ROS1 ROSBUILD")
  
  SET(CMAKE_CXX_FLAGS "-std=c++11 -march=native -Werror -Wall -g")

  IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
    MESSAGE(STATUS "Additional Flags for Release mode")
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O3 -DNDEBUG")
  ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
    MESSAGE(STATUS "Additional Flags for Debug mode")
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
  ENDIF()

  INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  ROSBUILD_INIT()
  SET(ROS_BUILD_STATIC_LIBS true)
  SET(ROS_BUILD_SHARED_LIBS false)

  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LUA REQUIRED lua5.1)

  MESSAGE(STATUS "ROS-Override Build Type: ${CMAKE_BUILD_TYPE}")
  MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")

  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})
  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/config_reader/include)

  SET(libs roslib roscpp glog gflags ${LUA_LIBRARIES})

  ROSBUILD_ADD_EXECUTABLE(pointcloud_to_laserscan
      pointcloud_to_laserscan_main.cc
  )
  TARGET_LINK_LIBRARIES(pointcloud_to_laserscan ${libs})
  TARGET_INCLUDE_DIRECTORIES(pointcloud_to_laserscan PRIVATE ${LUA_INCLUDE_DIRS})

elseif(${ROS_VERSION} EQUAL "2")
  MESSAGE(STATUS "Compiling with ROS2 COLCON")
  add_compile_definitions(ROS2)
  
  # ROS2 ament_cmake setup
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LUA REQUIRED lua5.1)
  
  SET(CMAKE_CXX_FLAGS "-std=c++17 -march=native -Werror -Wall -g")
  
  IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
    MESSAGE(STATUS "Additional Flags for Release mode")
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O3 -DNDEBUG")
  ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
    MESSAGE(STATUS "Additional Flags for Debug mode")
    SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
  ENDIF()

  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})
  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/config_reader/include)

  # ROS2 executable
  add_executable(pointcloud_to_laserscan
      pointcloud_to_laserscan_main.cc
  )
  
  # ROS2 dependencies
  ament_target_dependencies(pointcloud_to_laserscan
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
  )
  
  target_link_libraries(pointcloud_to_laserscan
      glog
      gflags
      ${LUA_LIBRARIES}
  )
  
  target_include_directories(pointcloud_to_laserscan PRIVATE ${LUA_INCLUDE_DIRS})

  # --- Add install steps ---
  install(TARGETS pointcloud_to_laserscan
    DESTINATION lib/${PROJECT_NAME}
  )
  
  install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
  )

  # Export runtime dependencies
  ament_export_dependencies(rclcpp std_msgs geometry_msgs sensor_msgs)

  ament_package()

else()
  message(FATAL_ERROR "Unknown ROS_VERSION ${ROS_VERSION}")
endif()
