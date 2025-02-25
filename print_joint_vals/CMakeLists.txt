cmake_minimum_required(VERSION 3.28)

project(PrintJoint)

## Define the base directory for external libraries
set(LIBRARIES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../Explicit-FRI/Libraries")

## Include directories (prefer target-based includes)
add_executable(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${LIBRARIES_DIR}/Explicit-cpp/include
  ${LIBRARIES_DIR}/myFRIClient/include
  ${LIBRARIES_DIR}/Explicit-cpp/Eigen
)

## Automatically add all .cpp files in the source directory
file(GLOB sourceFiles CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

## Add source files to the executable
target_sources(${PROJECT_NAME} PRIVATE ${sourceFiles})

## Define library paths with caching
set(EXPLICIT_CPP_LIBRARY "${LIBRARIES_DIR}/Explicit-cpp/lib/libExplicit-cpp.so" CACHE FILEPATH "Explicit-cpp shared library")
set(FRI_LIBRARY "${LIBRARIES_DIR}/myFRIClient/lib/libFRIClient.a" CACHE FILEPATH "FRI static library")

## Link libraries directly to the project
target_link_libraries(${PROJECT_NAME} PRIVATE ${EXPLICIT_CPP_LIBRARY} ${FRI_LIBRARY})
