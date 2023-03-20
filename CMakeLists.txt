cmake_minimum_required(VERSION 3.4 FATAL_ERROR)

# set the project name
project(robotSimulator 
    VERSION 1.0
    LANGUAGES CXX)

# # specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# compile extras options
# add_compile_options(-Wall -Wextra -O3 )

# add the find packages for Eigen in the cmake folder
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

# find packages
find_package(VTK COMPONENTS)
find_package(Boost COMPONENTS thread REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)
find_package(PCL 1.10 REQUIRED)

# Install Variables
include(GNUInstallDirs)

if (NOT VTK_FOUND)
  message("Skipping robotSimulator: ${VTK_NOT_FOUND_MESSAGE}")
  return ()
endif()

message (STATUS "VTK_VERSION: ${VTK_VERSION}")

## Build options
option(STRICT "Treat warnings as errors" OFF)

if(STRICT)
  add_compile_options(-Werror)
endif()

# file to compile
set(EXECUTABLE_FILES
  src/simLink.cpp
  src/simJoint.cpp
  src/simServerSocket.cpp
  src/simClientSocket.cpp
  src/simvtkTimerCallback.cpp
  src/robotSimulator.cpp
  src/simInterface.cpp
  src/simPointCloud.cpp
  )

include(${VTK_USE_FILE})

# Set variables
set(LIB_NAME rkd_simulator)

# add Libraries
add_library(${LIB_NAME} SHARED "${EXECUTABLE_FILES}")

# Include Directories
target_include_directories(${LIB_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/ ${EIGEN3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}) 

# Target link libraries
target_link_libraries(${LIB_NAME} PRIVATE ${VTK_LIBRARIES} ${Boost_LIBRARIES} urdfdom_model Eigen3::Eigen3 nlohmann_json::nlohmann_json ${PCL_LIBRARIES}  Boost::thread)

# compile extras options
option(COMPILE_EXAMPLE "Compile the test function" ON)
option(DEBUG "Compile in debug mode" ON)
option(RELEASE "Compile in release mode" OFF)

if (DEBUG){
  add_compile_options(-Wall -Wextra -O0)
}
else {
  add_compile_options(-O3)
}


# Headers
set(HEADERS_SIM
  include/RKD/simLink.h
  include/RKD/simJoint.h
  include/RKD/simServerSocket.h
  include/RKD/simClientSocket.h
  include/RKD/simvtkTimerCallback.h
  include/RKD/robotSimulator.h
  include/RKD/simInterface.h
  include/RKD/simPointCloud.h
)

# ------------------------------------------------------- COMPILE THE SERVER

add_executable(${PROJECT_NAME}_srv MACOSX_BUNDLE src/server.cpp)

# Include Dirs
target_include_directories(${PROJECT_NAME}_srv PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/ ${Boost_INCLUDE_DIRS})

# Target link libraries
target_link_libraries(${PROJECT_NAME}_srv PRIVATE ${VTK_LIBRARIES} ${Boost_LIBRARIES} urdfdom_model Eigen3::Eigen3 nlohmann_json::nlohmann_json ${PCL_LIBRARIES} ${LIB_NAME}  Boost::thread)

# Tests
add_subdirectory(dep/gtest)
enable_testing()

add_executable(
  tests
  tests/test_client.cpp
  )

target_link_libraries(
  tests
  GTest::gtest_main
  Eigen3::Eigen3 
  ${LIB_NAME}
)

include(GoogleTest)
gtest_discover_tests(tests)

# ------------------------------------------------------- EXAMPLES CODE
if (COMPILE_EXAMPLE)
  # add the executable
  add_executable(${PROJECT_NAME}_cln MACOSX_BUNDLE src/client.cpp)

  # Include Dirs
  target_include_directories(${PROJECT_NAME}_cln PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/ ${Boost_INCLUDE_DIRS} )

  # Target link libraries
  target_link_libraries(${PROJECT_NAME}_cln PRIVATE ${VTK_LIBRARIES} ${Boost_LIBRARIES} urdfdom_model Eigen3::Eigen3 nlohmann_json::nlohmann_json ${PCL_LIBRARIES} ${LIB_NAME}  Boost::thread)
endif()
# ------------------------------------------------------- end EXAMPLES CODE


# ------------------------------------------------------- INSTALL

# Set Version properties
set_target_properties(${LIB_NAME} PROPERTIES VERSION ${PROJECT_VERSION} 
                PUBLIC_HEADER "${HEADERS_SIM}")

# Install target
install(TARGETS ${LIB_NAME}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/RKD/)
