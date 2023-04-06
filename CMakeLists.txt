cmake_minimum_required(VERSION 3.5)
project(franka_utils)

set(franka_utils_MAJOR_VERSION 0)
set(franka_utils_MINOR_VERSION 0)
set(franka_utils_PATCH_VERSION 0)
set(franka_utils_VERSION
  ${franka_utils_MAJOR_VERSION}.${franka_utils_MINOR_VERSION}.${franka_utils_PATCH_VERSION})

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(Franka REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(compute_lumped_inertia src/compute_lumped_inertia.cpp)
target_link_libraries(compute_lumped_inertia urdfdom_headers::urdfdom_headers Eigen3::Eigen)
