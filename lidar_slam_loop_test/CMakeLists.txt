cmake_minimum_required(VERSION 3.5.1)
project(lidar_slam_loop_test)

find_package(Boost REQUIRED COMPONENTS thread)

set(CMAKE_BUILD_TYPE "Release")
set( CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(GTSAM_USE_SYSTEM_EIGEN ON)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  nav_msgs
  sensor_msgs
  message_generation
  message_runtime
  cv_bridge
  pcl_conversions
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(libnabo REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)


include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${libnabo_INCLUDE_DIRS})




catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)


# LiDAR-Iris
add_executable(iris_online src/iris_online.cpp 
  include/LidarIris/LidarIris.cpp
  include/LidarIris/fftm/fftm.cpp)
target_link_libraries(iris_online ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} libnabo::nabo)

# 可视化bow3d结果
add_executable(bow3d_online src/bow3d_online.cpp
      include/BoW3D/Frame.cpp  
      include/BoW3D/LinK3D_Extractor.cpp 
      include/BoW3D/BoW3D.cpp 
      )
target_link_libraries(bow3d_online ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${G2O_LIBS} ${Sophus_LIBRARIES})


add_executable(sc_online src/sc_online.cpp
              include/scancontext/Scancontext.cpp)
target_link_libraries(sc_online ${catkin_LIBRARIES}  ${PCL_LIBRARIES})


