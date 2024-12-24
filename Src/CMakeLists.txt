cmake_minimum_required(VERSION 3.28)
project(Src)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add debug flags
set(CMAKE_CXX_FLAGS "-g")

# Find dependencies
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
find_package(PCL 1.14 REQUIRED)
find_package(realsense2 REQUIRED)


# Include directories
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${realsense2_INCLUDE_DIRS})

# Link directories
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${realsense2_LIBRARY_DIRS})

add_executable(Src main.cpp
        Log.cpp
        CaptureScene.cpp
        Recognition.cpp
        ObjectTransformation.cpp
        PoseEstimation.cpp
        ArmControl.cpp
        JsonHandler.cpp)

# Link libraries
target_link_libraries(${PROJECT_NAME} stdc++fs)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${CMAKE_DL_LIBS})
target_link_libraries(${PROJECT_NAME} ${realsense2_LIBRARY})
