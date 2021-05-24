cmake_minimum_required(VERSION 3.7.2)

project(kdl_parser)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(urdfdom REQUIRED)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

find_package(orocos_kdl REQUIRED)
find_package(TinyXML REQUIRED)
find_package(TinyXML2 REQUIRED)

find_package(urdf QUIET)
if(urdf_FOUND)
  add_definitions(-DHAS_URDF)
  include_directories(${urdf_INCLUDE_DIRS})
endif()

include_directories(include ${orocos_kdl_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS} ${TinyXML2_INCLUDE_DIRS})

add_library(${PROJECT_NAME} SHARED src/kdl_parser.cpp)
target_link_libraries(${PROJECT_NAME}
  ${TinyXML_LIBRARIES} ${TinyXML2_LIBRARIES} ${orocos_kdl_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME} ${urdfdom_LIBRARIES})

if(urdf_FOUND)
  target_link_libraries(${PROJECT_NAME} ${urdf_LIBRARIES})
endif()

add_executable(check_kdl_parser src/check_kdl_parser.cpp )
target_link_libraries(check_kdl_parser ${PROJECT_NAME})
