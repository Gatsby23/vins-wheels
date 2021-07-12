cmake_minimum_required(VERSION 3.5.1)
project(loop_fusion)

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    camera_models
    cv_bridge
    roslib
    )

find_package(OpenCV)


find_package(Ceres REQUIRED)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR})

catkin_package()

add_executable(loop_fusion_node
    src/pose_graph_node.cpp
    src/pose_graph.cpp
    src/keyframe.cpp
    src/utility/CameraPoseVisualization.cpp
    src/ThirdParty/DBoW/BowVector.cpp
    src/ThirdParty/DBoW/FBrief.cpp
    src/ThirdParty/DBoW/FeatureVector.cpp
    src/ThirdParty/DBoW/QueryResults.cpp
    src/ThirdParty/DBoW/ScoringObject.cpp
    src/ThirdParty/DUtils/Random.cpp
    src/ThirdParty/DUtils/Timestamp.cpp
    src/ThirdParty/DVision/BRIEF.cpp
    src/ThirdParty/VocabularyBinary.cpp
    )

target_link_libraries(loop_fusion_node ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${CERES_LIBRARIES})


add_executable(loop_fusion_node_uisee
        src/pose_graph_node_uisee.cpp
        src/pose_graph_uisee.cpp
        src/keyframe_uisee.cpp

        src/utility/CameraPoseVisualization.cpp
        src/ThirdParty/DBoW/BowVector.cpp
        src/ThirdParty/DBoW/FBrief.cpp
        src/ThirdParty/DBoW/FeatureVector.cpp
        src/ThirdParty/DBoW/QueryResults.cpp
        src/ThirdParty/DBoW/ScoringObject.cpp
        src/ThirdParty/DUtils/Random.cpp
        src/ThirdParty/DUtils/Timestamp.cpp
        src/ThirdParty/DVision/BRIEF.cpp
        src/ThirdParty/VocabularyBinary.cpp
#        src/parameters/parameters.cpp
        src/featureTracker/feature_tracker.cpp
        )
target_link_libraries(loop_fusion_node_uisee ${catkin_LIBRARIES}  ${OpenCV_LIBS} ${CERES_LIBRARIES})

