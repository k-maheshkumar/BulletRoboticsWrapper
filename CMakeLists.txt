cmake_minimum_required(VERSION 2.8)
project(BulletWrapper)
include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(Bullet REQUIRED)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

#add_definitions(-DEIGEN_DEFAULT_TO_ROW_MAJOR)
#include(cmake/FindEigen2.cmake)


include_directories(SYSTEM include
    ${EIGEN2_INCLUDE_DIR}
    ${BULLET_INCLUDE_DIRS}
    )

set(BULLET_LIBRARY_DIRS "/usr/local/lib")
set(BULLET_LIBRARY_DIRS
    ${BULLET_LIBRARY_DIRS}/libBulletDynamics.so
    ${BULLET_LIBRARY_DIRS}/libBulletCollision.so
    ${BULLET_LIBRARY_DIRS}/libLinearMath.so
   ${BULLET_LIBRARY_DIRS}/libBulletRobotics.so
    )
#set(BULLET_INCLUDE_DIR "" CACHE PATH "bullet include directory")
#set(BULLET_LIB_DIR "" CACHE PATH "bullet library directory")
#include("./cmake/FindBullet.cmake")



link_directories(${BULLET_LIBRARIES})

set(source_files src/main.cpp src/BulletWrapper.cpp)
add_executable(${PROJECT_NAME} ${source_files})
target_link_libraries(${PROJECT_NAME}
#     ${BULLET_LIBRARIES}
     ${BULLET_LIBRARY_DIRS}

#    ${BULLET_LIBRARY_DIRS1}
#    -lGL -lGLU
    )
