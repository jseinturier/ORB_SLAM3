cmake_minimum_required(VERSION 3.12)
project(ORB_SLAM3)

# Enable find_package to search within environment variable_requires
# find_package(name) will use environment variable name_ROOT in order to find
# the package
cmake_policy(SET CMP0074 NEW)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

#######
# C++ Configuration
#######
# require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Debug extras settings
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUG")

# MSVC FIX FOR C++17 MACROS
if(MSVC)

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /W2 /Ox")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W2 /Ox")

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /Zc:__cplusplus -DUNICODE")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_HAS_STD_BYTE=0 /DNOMINMAX=1") # Avoid STL byte error on MSVC
else()

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -W2 -O3")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -W2 -O3")
  set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")
endif(MSVC)

if(WIN32) # WINDOWS DLL SYMBOLS
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
else() # Linux extra flags
  list(APPEND flags "-Wextra" "-Wpedantic")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()

# C++ Flags
list(APPEND flags "-fPIC")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.4)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 4.4 not found.")
   endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${PROJECT_SOURCE_DIR}/include/CameraModels
${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/System.cc
src/Tracking.cc
src/LocalMapping.cc
src/LoopClosing.cc
src/ORBextractor.cc
src/ORBmatcher.cc
src/FrameDrawer.cc
src/Converter.cc
src/MapPoint.cc
src/KeyFrame.cc
src/Atlas.cc
src/Map.cc
src/MapDrawer.cc
src/Optimizer.cc
src/Frame.cc
src/KeyFrameDatabase.cc
src/Sim3Solver.cc
src/ImuTypes.cc
src/G2oTypes.cc
src/CameraModels/Pinhole.cpp
src/CameraModels/KannalaBrandt8.cpp
src/OptimizableTypes.cpp
src/MLPnPsolver.cpp
src/GeometricTools.cc
src/TwoViewReconstruction.cc
src/Config.cc
src/Settings.cc
include/System.h
include/Tracking.h
include/LocalMapping.h
include/LoopClosing.h
include/ORBextractor.h
include/ORBmatcher.h
include/FrameDrawer.h
include/Converter.h
include/MapPoint.h
include/KeyFrame.h
include/Atlas.h
include/Map.h
include/MapDrawer.h
include/Optimizer.h
include/Frame.h
include/KeyFrameDatabase.h
include/Sim3Solver.h
include/ImuTypes.h
include/G2oTypes.h
include/CameraModels/GeometricCamera.h
include/CameraModels/Pinhole.h
include/CameraModels/KannalaBrandt8.h
include/OptimizableTypes.h
include/MLPnPsolver.h
include/GeometricTools.h
include/TwoViewReconstruction.h
include/SerializationUtils.h
include/Config.h
include/Settings.h
include/SLAMEventHandler.hpp)

# Dependecy to DBoW2
add_subdirectory(Thirdparty/DBoW2)
target_link_libraries(${PROJECT_NAME} DBoW2)

# Dependency to g2o
add_subdirectory(Thirdparty/g2o)
target_link_libraries(${PROJECT_NAME} g2o)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${EIGEN3_LIBS}
-lcrypto
)
