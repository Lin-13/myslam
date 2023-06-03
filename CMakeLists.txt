cmake_minimum_required(VERSION 3.8)
project(myslam)
set(CMAKE_BUILD_TYPE "DEBUG")
set(CMAKE_CXX_STANDARD 17)
#package
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(G2O REQUIRED)
find_package(Pangolin 0.8 REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIR} ${Pangolin_INCLUDE_DIRS})
add_executable(test src/test.cpp src/Backend.cpp)
add_library(${PROJECT_NAME} 
    STATIC 
    src/Frame.cpp
    src/Map.cpp
    src/MapPoint.cpp
    src/Camera.cpp
    src/Frontend.cpp
    src/Viewer.cpp
    src/Backend.cpp)
target_link_libraries(
    ${PROJECT_NAME}
    pango_display
)
target_include_directories(${PROJECT_NAME}
    PRIVATE
    include ${OpenCV_INCLUDE_DIR} ${Pangolin_INCLUDE_DIRS}
)
target_link_libraries(
    test 
    ${PROJECT_NAME}
    fmt
    ${OpenCV_LIBS}
    opencv_sfm
)
install(TARGETS test
    DESTINATION
    ${CMAKE_SOURCE_DIR}/bin
)
install(TARGETS ${PROJECT_NAME}
    DESTINATION
    ${CMAKE_SOURCE_DIR}/bin
)