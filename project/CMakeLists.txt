cmake_minimum_required(VERSION 3.0)
project(spacecraft_sim)

set(CMAKE_BUILD_TYPE "Debug")

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=gnu++11 -O2 -DNDEBUG")
set( CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

find_package(Eigen3 REQUIRED)
include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    lib/geometry/include
)

# pybind11
add_subdirectory(include/pybind11)

pybind11_add_module(spacecraft_sim_interface
    src/spacecraft_sim.cpp
    src/dynamics.cpp
    src/spacecraft.cpp
)
# add_dependencies(spacecraft_sim_interface ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_include_directories(spacecraft_sim_interface PUBLIC lib/geometry/include)
set_target_properties(spacecraft_sim_interface PROPERTIES
  PREFIX "${PYTHON_MODULE_PREFIX}"
  SUFFIX "${PYTHON_MODULE_EXTENSION}"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/python/"
)

# target_link_libraries(spacecraft_sim_interface PUBLIC 
#   rosflight_holodeck_board 
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )