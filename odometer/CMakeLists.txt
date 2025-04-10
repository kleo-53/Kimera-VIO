cmake_minimum_required(VERSION 3.10)
project(GnssStereoVIO)

set(CMAKE_CXX_STANDARD 17)

include_directories(
    ${PROJECT_SOURCE_DIR}/../Kimera-VIO
    /usr/local/include
)

link_directories(
    /usr/local/lib
)

add_executable(gnss_stereo_vio GnssStereoVIO.cpp)

target_link_libraries(gnss_stereo_vio
    boost_system boost_filesystem
    glog gflags
    opencv_core opencv_imgproc opencv_highgui opencv_videoio
    tbb pthread
)
