cmake_minimum_required(VERSION 3.5)
project(server)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")
set(CMAKE_CXX_STANDARD 20)

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(BUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/build)

file(GLOB_RECURSE SOURCES ${SOURCE_DIR}/*.cpp)

include_directories(${INCLUDE_DIR})
link_directories(${LIB_DIR})
# link_directories(${CMAKE_SOURCE_DIR}/lib/UAV_common)

link_directories("/usr/lib/x86_64-linux-gnu")
link_directories("/usr/local/lib")
link_directories("/usr/local/include")

add_executable(server ${SOURCES})
set_property(TARGET server PROPERTY CXX_STANDARD 20)
target_include_directories(server PUBLIC include)
target_compile_features(server PUBLIC cxx_std_20)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
target_link_libraries(server Eigen3::Eigen)
find_package(cppzmq)
target_link_libraries(server cppzmq)
find_package(yaml-cpp)
target_link_libraries(server yaml-cpp)
find_package(Boost 1.71.0 REQUIRED COMPONENTS system)
target_link_libraries(server Boost::system)
target_link_libraries(server "/usr/lib/x86_64-linux-gnu/libmodbus.so")

add_custom_command(TARGET server POST_BUILD 
               COMMAND ${CMAKE_COMMAND} -E copy_if_different
               "${CMAKE_SOURCE_DIR}/lib/libconstraints.so"              
                $<TARGET_FILE_DIR:server>)
