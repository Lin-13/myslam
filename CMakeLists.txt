cmake_minimum_required(VERSION 3.8)
project(myslam)
set(CMAKE_BUILD_TYPE "DEBUG")
set(CMAKE_CXX_STANDARD 17)
#package
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(G2O REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIR})
add_executable(test src/test.cpp)
add_library(${PROJECT_NAME} 
    STATIC 
    src/Frame.cpp
    src/Map.cpp
    src/MapPoint.cpp
    src/Camera.cpp
    src/Frontend.cpp
)
target_link_libraries(
    test 
    ${PROJECT_NAME}
    fmt
    ${OpenCV_LIBS}
)
install(TARGETS test
    DESTINATION
    ${CMAKE_SOURCE_DIR}/bin
)