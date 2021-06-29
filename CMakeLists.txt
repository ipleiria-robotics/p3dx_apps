cmake_minimum_required(VERSION 2.4.6)
project(p3dx_apps)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CMAKE_BUILD_TYPE Debug)

# Find and get all the information about the roscpp package
find_package(catkin REQUIRED COMPONENTS roscpp roslaunch)

# Declare the project as a catkin package
catkin_package()

# Check for possible syntax error in our launch files
roslaunch_add_file_check(launch)

# Add header folders
include_directories(include ${catkin_INCLUDE_DIRS})
 
# Define an executable target called tp1
add_executable(p3dx_demo src/main.cpp)

# Link the target against the libraries used by roscpp
target_link_libraries(p3dx_demo ${catkin_LIBRARIES})

