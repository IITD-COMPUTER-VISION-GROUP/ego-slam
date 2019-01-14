cmake_minimum_required(VERSION 2.8.12)

project(sensitivity-analysis)

#########################################################
# FIND C++11 Support
#########################################################
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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(EIGEN_INCLUDE_DIR "N:/BROJO/ceres/eigen-eigen")
#########################################################
# FIND OPENCV
#########################################################
find_package(OpenCV REQUIRED)
#########################################################
# FIND GFLAGS
#########################################################
# find_package(gflags REQUIRED)
#########################################################
# FIND GLOG
#########################################################
# find_package(glog REQUIRED)
#########################################################
# FIND CERES
#########################################################
find_package(Ceres REQUIRED)
#########################################################
# Include Files
#########################################################
#find_package(PCL 1.2 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}
                    ${CERES_INCLUDE_DIRS}
                    include
                    ${PCL_INCLUDE_DIRS})

add_library(helpers-cc src/triangulation.cc
                       src/sequential_probability_ratio.cc
                       src/random.cc
                       src/pose_util.cc
                       src/fundamental_matrix_util.cc
                       src/five_point.cpp
                       src/estimate_uncalibrated_relative_pose.cc
                       src/estimate_twoview_info.cc
                       src/estimate_relative_pose.cpp
                       src/essential_matrix_utils.cc
                       src/eight_point_fundamental_matrix.cc
                       src/correspondance.cc
                       src/helpers.cc)

add_library (dense-helpers-cc src/triangulate.cc
                              src/densehelpers.cc)

add_library(sift-cc src/sift_processing.cc)

add_library(matrix src/matrix.cpp)

#add_library(pointcloud src/pointcloud.cc)

add_executable(slam scripts/slam.cpp)

#add_executable(main scripts/main.cpp)

#add_executable(place_side scripts/place_side.cpp)

#add_executable(epipolar scripts/epipolar_verification.cpp)

#add_executable(dense scripts/densify.cpp)

add_executable(corres scripts/corres_new.cpp)

add_executable(merge scripts/merge_pt_cloud.cpp)

add_executable(undistort scripts/omni2persp.cpp)

#add_executable(kitdev scripts/evaluate_odometry.cpp)

#add_executable(prior scripts/priors.cpp
#                     src/ceres_fit.cc)

#target_link_libraries(kitdev matrix)

#target_link_libraries(prior ${OpenCV_LIBS}
#                             gflags glog
#                             tiff
#                             ${PCL_LIBRARIES}
#                             ${CERES_LIBRARIES}
#                             pointcloud)

target_link_libraries(slam ${OpenCV_LIBS}
                           gflags
                           glog
                           tiff
                           ${CERES_LIBRARIES}
                           helpers-cc)

#target_link_libraries(main ${OpenCV_LIBS}
#                           gflags
#                           glog
#                           tiff
#                           ${CERES_LIBRARIES}
#                           helpers-cc)

#target_link_libraries(place_side ${OpenCV_LIBS}
#                              tiff)

#target_link_libraries(epipolar ${OpenCV_LIBS}
#                                tiff
#                                gflags)

#target_link_libraries(dense ${OpenCV_LIBS}
#                            gflags
#                            tiff
#                            dense-helpers-cc)

target_link_libraries(corres ${OpenCV_LIBS}
                              tiff)

target_link_libraries(merge ${OpenCV_LIBS}
                             gflags
                             tiff
                             dense-helpers-cc)

target_link_libraries(undistort ${OpenCV_LIBS}
                                tiff
                                gflags)
