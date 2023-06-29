CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

PROJECT (WhippingTask LANGUAGES CXX)

# ===================================================== #
# ============= SETTING UP THE CAMERA SYSTEM ========== #
# ===================================================== #

# Get the directory of the camera application
# For this, we recommend using the absolute directory
set( my_cam_app /home/baxterplayground/Documents/camera_applications )

# Adding the libraries to write the code

set(includeDir
  ${my_cam_app}/app
  ${my_cam_app}/my_cam_functions
  ${my_cam_app}/app/CRSDK )

include_directories( ${includeDir} )

# ================================================== #
# =========== SETTING UP APP DIRECTORY ============= #
# ================================================== #
set(app_dir ${my_cam_app}/app)

### Enumerate the sdk header(hdrs) and cpp(cpps) files
set(sdk_hdrs_cpps
    ${app_dir}/Text.h
    ${app_dir}/Text.cpp
    ${app_dir}/CameraDevice.h
    ${app_dir}/CameraDevice.cpp
    ${app_dir}/MessageDefine.h
    ${app_dir}/MessageDefine.cpp
    ${app_dir}/ConnectionInfo.h
    ${app_dir}/ConnectionInfo.cpp
    ${app_dir}/PropertyValueTable.h
    ${app_dir}/PropertyValueTable.cpp
)


# ================================================== #
# ======== SETTING UP THE PERSONAL FUNCS =========== #
# ================================================== #
set(my_app_dir ${my_cam_app}/my_cam_functions )

set( my_hdrs_cpps
    ${my_app_dir}/MyCameraApp.cpp
    ${my_app_dir}/MyCameraApp.h
)

# ================================================== #
# ======== SETTING UP APP/CRSDK DIRECTORY ========== #
# ================================================== #
## Script for enumerating CameraRemote SDK public header files
set(app_crsdk_dir ${my_cam_app}/app/CRSDK)

### Enumerate CameraRemote SDK public header files
set(crsdk_hdrs
    ${app_crsdk_dir}/CameraRemote_SDK.h
    ${app_crsdk_dir}/CrCommandData.h
    ${app_crsdk_dir}/CrDefines.h
    ${app_crsdk_dir}/CrDeviceProperty.h
    ${app_crsdk_dir}/CrError.h
    ${app_crsdk_dir}/CrImageDataBlock.h
    ${app_crsdk_dir}/CrTypes.h
    ${app_crsdk_dir}/ICrCameraObjectInfo.h
    ${app_crsdk_dir}/IDeviceCallback.h
)

# [2023.06.28] [Moses C. Nah]
# The following line is commented out since it seems unnecessary (although included in the original sdk)
# source_group("CRSDK" FILES ${crsdk_hdrs})


# ================================================== #
# ============= ADDING OpenCV Library ============== #
# ================================================== #
### The openCV must be downloaded separately:
### [REF] [For Linux] https://docs.opencv.org/4.7.0/d7/d9f/tutorial_linux_install.html

# [2023.06.28] [Moses C. Nah]
# If Necessary, use the following two lines of code and specify the path where opencv was downloaded/built
#set( opencv_dir /home/newman_lab/Documents/my_opencv/build )
#find_package( OpenCV REQUIRED PATHS "${opencv_dir}")

# [REF] {OPENCV_BUILD_DIRECTORY}/build/OpenCVConfig.cmake
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
#target_link_libraries(MY_TARGET_NAME ${OpenCV_LIBS})



# ===================================================== #
# ==================== KUKA Libraries ================= #
# ===================================================== #

## include directories
set(includeDir
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../Libraries/Explicit-cpp/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../Libraries/myFRIClient/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../Libraries/Explicit-cpp/Eigen
)
include_directories(${includeDir})

## make all source files visible in the IDE
set(sourceFiles "")
file(GLOB_RECURSE sourceFiles ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp )

## add executable
add_executable(${PROJECT_NAME}
    ${sourceFiles}
    ${headerFiles}
    ${sdk_hdrs_cpps}
    ${my_hdrs_cpps}
    ${crsdk_hdrs}
)


##### LINKING #####

## Look for MATH_LIBRARY, otherwise cmath doesn't compile!
find_library(MATH_LIBRARY m)
if(MATH_LIBRARY)
    target_link_libraries(${PROJECT_NAME} ${MATH_LIBRARY})
endif()

#set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(EXPLICIT_CPP_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/../../../Libraries/Explicit-cpp/lib/libExplicit-cpp.so" CACHE STRING "Explicit-cpp library")
set(FRI_LIBRARY "${CMAKE_CURRENT_SOURCE_DIR}/../../../Libraries/myFRIClient/lib/libFRIClient.a" CACHE STRING "FRI library")

set(linkLibraries
${EXPLICIT_CPP_LIBRARY}
${FRI_LIBRARY}
)


##### SOME PARTS OF THE CODE GO INTO AN ADDITIONAL LIBRARY #####

target_link_libraries(${PROJECT_NAME} ${linkLibraries})

# ================================================================== #
# The following if statements are given from the sony sdk

# Note that the "target_" prefix is for a specific target rather than the whole cmake file.
# Specify char is signed-char to fix mismatch with Raspbian
target_compile_options(${PROJECT_NAME} PRIVATE -fsigned-char )
target_include_directories(${PROJECT_NAME} PRIVATE ${app_crsdk_dir} )

# ================================================== #
# ============ LINKING THE SO LIBRARY ============== #
# ================================================== #
# Configure external library directories
set(lib_dir ${my_cam_app}/external/crsdk)

### Link CRSDK and OpenCV libraries
find_library(camera_remote Cr_Core HINTS ${lib_dir})

target_link_libraries(${PROJECT_NAME}
        ${camera_remote}
        ${OpenCV_LIBS}
)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")

    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8)
        # Must use std::experimental namespace if older than GCC8
        message("[${PROJECT_NAME}] GCC version less than 8. Using std::experimental namespace.")
        target_compile_definitions(${PROJECT_NAME} PRIVATE USE_EXPERIMENTAL_FS)
    endif()

    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
        # Must explicitly link separate std::filesystem if older than GCC9
        message("[${PROJECT_NAME}] GCC version less than 9. Explicitly linking separate std::filesystem library.")
        target_link_libraries(${PROJECT_NAME} PRIVATE stdc++fs)
    endif()

endif()

add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${lib_dir} $<TARGET_FILE_DIR:${PROJECT_NAME}>
)

# ================================================================== #
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")




## the output directory
set_target_properties(${PROJECT_NAME}
        PROPERTIES
    DEBUG_POSTFIX "${CMAKE_DEBUG_POSTFIX}"
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
    BUILD_RPATH "$ORIGIN"
    INSTALL_RPATH "$ORIGIN"
  RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_CURRENT_SOURCE_DIR}/../build/Debug"
  RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/../build/Release"
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ DESTINATION ${CMAKE_INSTALL_PREFIX}/include
  FILES_MATCHING PATTERN "*.h"
)
set(libOutputDir ${CMAKE_INSTALL_PREFIX}/lib)
INSTALL (TARGETS ${PROJECT_NAME}
                LIBRARY DESTINATION ${libOutputDir}
                ARCHIVE DESTINATION ${libOutputDir}
                RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/application
)
