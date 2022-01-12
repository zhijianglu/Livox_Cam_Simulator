cmake_minimum_required(VERSION 2.8.3)
project(livox_sim_tool)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")




find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV 3 REQUIRED)
#set(OpenCV_DIR /new_space/cpp_lib/opencv412/opencv412/build)
#find_package( OpenCV 4 REQUIRED )

find_package(Ceres REQUIRED)
find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

FIND_PACKAGE(OpenMP REQUIRED)
if (OPENMP_FOUND)
  message("OPENMP FOUND")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()


include_directories(${ZED_INCLUDE_DIRS})
link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

find_package(Boost COMPONENTS program_options REQUIRED )
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})

find_package(PCL REQUIRED)
message(STATUS "***** PCL version: ${PCL_VERSION} *****")
####


include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ./include
  )

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
)


#TODO 测试pcd存储python的npy,在生成ACSC的仿真文件的时候要用
#add_executable(test_pcd2npy src/pcd2npy.cpp)
#target_link_libraries(test_pcd2npy cnpy)

#TODO 生成ACSC的仿真文件，这里不包含分割，只有标定板的强度赋值，然后直接存储所有点云
add_executable(intensity_mapping_ACSC src/intensity_mapping_ACSC.cpp  src/global.cpp src/zed_cam.hpp include/params.h )
target_link_libraries(intensity_mapping_ACSC ${PCL_LIBRARIES} ${OpenCV_LIBS})

#TODO 生成ACSC的仿真文件，这里不包含分割，只有标定板的强度赋值，然后直接存储所有点云
add_executable(intensity_mapping_ILCC src/intensity_mapping_ILCC.cpp  src/global.cpp src/zed_cam.hpp include/params.h )
target_link_libraries(intensity_mapping_ILCC ${PCL_LIBRARIES} ${OpenCV_LIBS})


#TODO Gazebo运行时候录制仿真文件
add_executable(data_recorder src/data_recorder.cpp src/params.cpp src/global.cpp)
target_link_libraries(data_recorder ${catkin_LIBRARIES} ${ZED_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBS})

#TODO 点云与图像配准（未更新）
#add_executable(lc_align src/lc_align.cpp src/params.cpp src/zed_cam.hpp include/params.h )
#target_link_libraries(lc_align ${catkin_LIBRARIES} ${ZED_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBS})

#TODO 我们方法的强度值映射
add_executable(intensity_mapping_Ours src/intensity_mapping_Ours.cpp src/global.cpp src/zed_cam.hpp include/params.h )
target_link_libraries(intensity_mapping_Ours ${PCL_LIBRARIES} ${OpenCV_LIBS})


#TODO 代码测试
add_executable(codetest src/codetest.cpp)

#TODO 我们的方法的标定板的分割
add_executable(sim_board_ext src/sim_board_ext.cpp src/global.cpp src/zed_cam.hpp include/params.h )
target_link_libraries(sim_board_ext  ${PCL_LIBRARIES} ${OpenCV_LIBS})

#TODO 测试pnp
add_executable(test_pnp src/test_pnp.cpp src/global.cpp src/zed_cam.hpp include/params.h )
target_link_libraries(test_pnp ${PCL_LIBRARIES} ${OpenCV_LIBS})

#TODO 生成标准的图像标定板上的角点真值
add_executable(gen_gt_feature_corner src/gen_gt_feature_corner.cpp src/global.cpp src/zed_cam.hpp include/params.h )
target_link_libraries(gen_gt_feature_corner ${PCL_LIBRARIES} ${OpenCV_LIBS})

#add_executable(zed_sample src/zed_sample.cpp )
#target_link_libraries(zed_sample ${catkin_LIBRARIES} ${ZED_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBS})

#add_executable(pcd_viewer src/pcd_viewer.cpp )
#target_link_libraries(pcd_viewer ${catkin_LIBRARIES} ${ZED_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBS})



#add_executable(read_results src/detect_board.cpp src/params.cpp include/params.h src/DataGrab.cpp include/DataGrab.h )
#target_link_libraries(read_results ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
