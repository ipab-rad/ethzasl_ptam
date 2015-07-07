cmake_minimum_required(VERSION 2.8.3)
project(ptam)

add_custom_target(
  DEPS ALL
  COMMAND echo Building all dependencies
  COMMAND cd ${PROJECT_SOURCE_DIR}/thirdparty && make
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  std_msgs
  image_transport
  dynamic_reconfigure
  sensor_msgs
  ptam_com
  tf
  visualization_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)

generate_dynamic_reconfigure_options(cfg/PTAMVisualizerParams.cfg cfg/PtamParams.cfg)

##Needed for ros packages
catkin_package(CATKIN_DEPENDS roscpp dynamic_reconfigure std_msgs image_transport dynamic_reconfigure sensor_msgs ptam_com tf visualization_msgs cv_bridge)

set(PTAM_SOURCE_FILES         
	src/main.cc      
	src/ATANCamera.cc          
	src/Map.cc       
	src/MiniPatch.cc    
	src/ShiTomasi.cc         
	src/Tracker.cc
  src/Bundle.cc                 
  src/HomographyInit.cc  
  src/MapMaker.cc  
  src/PatchFinder.cc  
  src/SmallBlurryImage.cc                
  src/KeyFrame.cc        
  src/MapPoint.cc  
  src/Relocaliser.cc  
  src/System.cc     
  src/Params.cpp
  src/OctomapInterface.cc
)

set(PTAM_HEADER_FILES        
  include/ptam/CameraCalibrator.h  
  include/ptam/KeyFrame.h      
  include/ptam/MapViewer.h    
  include/ptam/Relocaliser.h       
  include/ptam/TrackerData.h
  include/ptam/ATANCamera.h         
  include/ptam/LevelHelpers.h  
  include/ptam/MEstimator.h   
  include/ptam/ShiTomasi.h         
  include/ptam/Tracker.h
  include/ptam/Bundle.h            
  include/ptam/Map.h           
  include/ptam/MiniPatch.h    
  include/ptam/SmallBlurryImage.h 
  include/ptam/CalibCornerPatch.h  
  include/ptam/MapMaker.h      
  include/ptam/SmallMatrixOpts.h
  include/ptam/CalibImage.h        
  include/ptam/HomographyInit.h    
  include/ptam/MapPoint.h      
  include/ptam/PatchFinder.h  
  include/ptam/System.h  
  include/ptam/Params.h
)
	
SET(CALIBRATOR_FILES     
  src/CameraCalibrator.cc 
  src/GLWindowMenu.cc 
  src/CalibCornerPatch.cc           
  src/CalibImage.cc            
  src/GLWindow2.cc	
  src/ATANCamera.cc 
  src/Params.cpp 
)	

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/thirdparty/TooN/include 
  ${PROJECT_SOURCE_DIR}/thirdparty/libcvd/include 
  ${PROJECT_SOURCE_DIR}/thirdparty/gvars3/include 
  ${PROJECT_SOURCE_DIR}/thirdparty/agast/include
  ${catkin_INCLUDE_DIRS}
)  

link_directories(
  ${PROJECT_SOURCE_DIR}/thirdparty/libcvd/lib 
  ${PROJECT_SOURCE_DIR}/thirdparty/gvars3/lib 
  ${PROJECT_SOURCE_DIR}/thirdparty/agast/lib
)

set(PTAM_LIBRARIES cvd GVars3_headless agast /home/nao/lapack-3.5.0/build/lib/libblas.so /home/nao/lapack-3.5.0/build/lib/liblapack.so)

add_definitions(-DCVD_HAVE_TOON)
add_definitions(-DKF_REPROJ)
set(CMAKE_BUILD_TYPE Release)

# PTAM executable
add_executable(ptam ${PTAM_SOURCE_FILES} ${PTAM_HEADER_FILES})
set_property(TARGET ptam APPEND_STRING PROPERTY COMPILE_FLAGS "-D_LINUX -D_REENTRANT -Wall -march=native ")
IF(CMAKE_BUILD_TYPE MATCHES Release)
 set_property(TARGET ptam APPEND_STRING PROPERTY COMPILE_FLAGS "-O3 ")
 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsee -funsafe-loop-optimizations -fno-signed-zeros -fno-math-errno -funroll-loops")
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)
add_dependencies(ptam DEPS ${${PROJECT_NAME}_EXPORTED_TARGETS}})
add_dependencies(ptam ${PROJECT_NAME}_gencfg)

target_link_libraries(ptam ${PTAM_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})


