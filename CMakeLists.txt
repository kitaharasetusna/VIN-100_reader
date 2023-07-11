cmake_minimum_required(VERSION 3.0)
project(VIN_100)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)


add_subdirectory(vnproglib-1.2.0.0/cpp)
include_directories(vnproglib-1.2.0.0/cpp/include)




# Create the executable
add_executable(vin_100 main.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(vin_100
  libvncxx
)
