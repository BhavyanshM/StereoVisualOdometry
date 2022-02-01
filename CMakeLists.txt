cmake_minimum_required(VERSION 3.4)
project(map_sense)

# This line is required for specifying the "modules" directory in the project containing .cmake files
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/modules/" ${CMAKE_MODULE_PATH})

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

set(GTSAM_ALLOW_DEPRECATED_SINCE_V42 OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
#MATH(EXPR stack_size "2048000 * 1024 * 1024") # 16 Mb
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,stack-size=${stack_size}")

enable_testing()
find_package(GTest REQUIRED) # Find the google testing framework on your system
#find_package(Torch REQUIRED PATHS "${PROJECT_SOURCE_DIR}/Vendor/libtorch")

find_package(gflags REQUIRED)
find_package( OpenCV REQUIRED )
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTSAM 4.2.0 REQUIRED )

include_directories(${GTSAM_INCLUDE_DIR})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(VISUAL_ODOMETRY_LIB
		src/VisualOdometry/VisualOdometry.cpp
		src/headers/VisualOdometry.h
		src/VisualOdometry/BundleAdjustment.cpp
		src/headers/BundleAdjustment.h)


set(GEOMTOOLS_SRC
		src/Geometry/include/CameraParams.h
		src/Geometry/include/PointLandmark.h
		src/Geometry/src/Elements/CameraParams.cpp
		src/Geometry/src/Elements/PointLandmark.cpp)

include_directories(
		include
		src/headers
		src/Geometry/include
)

#set_target_properties(Mapsense_Launcher_UI PROPERTIES LINK_FLAGS -Wl,-z, stack-size, 1000000000000)

#[[
	Develop all libraries here.
]]
add_library(geom-tools STATIC ${GEOMTOOLS_SRC})
target_link_libraries(geom-tools
		Eigen3::Eigen
		${OpenCV_LIBS})

#add_library(visual_odometry STATIC ${VISUAL_ODOMETRY_LIB})
#target_link_libraries(visual_odometry
#		Eigen3::Eigen
#		${OpenCV_LIBS})

set(VIS_ODOM 1)
if(VIS_ODOM)
	add_executable(Vis_Odom ${VISUAL_ODOMETRY_LIB}
			src/ApplicationState.cpp
			src/DataManager.cpp
			src/ImageTools.cpp
			src/VisualOdometryLauncher.cpp
			src/FactorGraphHandler.cpp
			src/FactorGraphHandler.h
#			src/VisualISAM2Example.cpp
			src/AppUtils.cpp
			)
	target_link_libraries(Vis_Odom
			${OpenCV_LIBS}
			Eigen3::Eigen
			geom-tools gtsam)
endif()

add_executable(ISAM_Visual
		src/SFMdata.h
		src/VisualISAM2Example.cpp
		src/FactorGraphHandler.cpp
		src/FactorGraphHandler.h)
target_link_libraries(ISAM_Visual
		${OpenCV_LIBS}
		Eigen3::Eigen
		geom-tools gtsam)


## //////////////////////////////////////////////////////////////////////////////////////////////////////////////

