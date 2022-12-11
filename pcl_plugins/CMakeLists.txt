cmake_minimum_required(VERSION 2.8)
project(pcl_registration)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(PCL 1.8 REQUIRED)

file(GLOB PCL_LIBRARIES /usr/lib/x86_64-linux-gnu/libpcl_*)
set(PCL_INCLUDE_DIRS /usr/include/pcl-1.8)
set(EIGEN3_INCLUDE_DIR /usr/local/include/eigen3)

include_directories(
${PROJECT_SOURCE_DIR}
# ${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${Pangolin_INCLUDE_DIRS}
${PCL_INCLUDE_DIRS}
/usr/include/vtk-6.3
)
add_definitions( ${PCL_DEFINITIONS} )
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions(-DBOOST_SYSTEM_NO_DEPRECATED)

# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

# add_library(${PROJECT_NAME} SHARED
# src/System.cc
# src/Tracking.cc
# src/LocalMapping.cc
# src/LoopClosing.cc
# src/ORBextractor.cc
# src/ORBmatcher.cc
# src/FrameDrawer.cc
# src/Converter.cc
# src/MapPoint.cc
# src/KeyFrame.cc
# src/Map.cc
# src/MapDrawer.cc
# src/Optimizer.cc
# src/PnPsolver.cc
# src/Frame.cc
# src/KeyFrameDatabase.cc
# src/Sim3Solver.cc
# src/Initializer.cc
# src/Viewer.cc
# )

target_link_libraries(
# ${OpenCV_LIBS}
# ${EIGEN3_LIBS}
# ${Pangolin_LIBRARIES}
${PCL_INCLUDE_DIRS}
# /usr/include/vtk-6.3
# ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
# ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)

# Build examples

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(reg  src/reg.cpp)
target_link_libraries(reg  ${PCL_LIBRARIES} 
/usr/lib/x86_64-linux-gnu/libboost_thread.so.1.65.1
/usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-6.3.so.6.3
/usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.3.so.6.3
/usr/lib/x86_64-linux-gnu/libvtkRenderingFreeTypeFontConfig-6.3.so.6.3
/usr/lib/x86_64-linux-gnu/libvtkRenderingMatplotlib-6.3.so.6.3
/usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL-6.3.so.6.3
/usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-6.3.so.6.3
/usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-6.3.so.6.3
/usr/lib/x86_64-linux-gnu/libvtkRenderingCore-6.3.so.6.3
/usr/lib/x86_64-linux-gnu/libvtkCommonMath-6.3.so.6.3
)

add_executable(convert  src/convert.cpp)
target_link_libraries(convert  ${PCL_LIBRARIES}   ${OpenCV_LIBS}
/usr/lib/x86_64-linux-gnu/libboost_thread.so.1.65.1
/usr/lib/x86_64-linux-gnu/libvtkCommonDataModel-6.3.so.6.3
/usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.3.so.6.3
/usr/lib/x86_64-linux-gnu/libvtkRenderingFreeTypeFontConfig-6.3.so.6.3
/usr/lib/x86_64-linux-gnu/libvtkRenderingMatplotlib-6.3.so.6.3
/usr/lib/x86_64-linux-gnu/libvtkRenderingOpenGL-6.3.so.6.3
/usr/lib/x86_64-linux-gnu/libvtkInteractionStyle-6.3.so.6.3
/usr/lib/x86_64-linux-gnu/libvtkRenderingFreeType-6.3.so.6.3
/usr/lib/x86_64-linux-gnu/libvtkRenderingCore-6.3.so.6.3
/usr/lib/x86_64-linux-gnu/libvtkCommonMath-6.3.so.6.3
)

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Stereo)

# add_executable(stereo_kitti
# Examples/Stereo/stereo_kitti.cc)
# target_link_libraries(stereo_kitti ${PROJECT_NAME})

# add_executable(stereo_euroc
# Examples/Stereo/stereo_euroc.cc)
# target_link_libraries(stereo_euroc ${PROJECT_NAME})


# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/Monocular)

# add_executable(mono_tum
# Examples/Monocular/mono_tum.cc)
# target_link_libraries(mono_tum ${PROJECT_NAME})

# add_executable(mono_kitti
# Examples/Monocular/mono_kitti.cc)
# target_link_libraries(mono_kitti ${PROJECT_NAME})

# add_executable(mono_euroc
# Examples/Monocular/mono_euroc.cc)
# target_link_libraries(mono_euroc ${PROJECT_NAME})

