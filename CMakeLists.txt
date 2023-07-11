cmake_minimum_required(VERSION 3.0)
project(VIN_100)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)

# for vns
add_subdirectory(vnproglib-1.2.0.0/cpp)
include_directories(vnproglib-1.2.0.0/cpp/include)

# for reading XML files
find_package(Poco REQUIRED COMPONENTS Foundation XML)
include_directories(${Poco_INCLUDE_DIRS})

# set this config to enable assert
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -UNDEBUG")
# Create the executable
add_executable(vin_100 main.cpp)
target_link_libraries(vin_100 ${Poco_LIBRARIES})



## Specify libraries to link a library or executable target against
target_link_libraries(vin_100
  libvncxx
)
