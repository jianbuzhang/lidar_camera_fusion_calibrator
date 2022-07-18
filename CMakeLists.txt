cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME CalibratorTool)
project(${PROJECT_NAME})

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check http://doc.qt.io/qt-5/deployment-android.html for more information.
# They need to be set before the find_package(Qt5 ...) call.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()

find_package(Qt5 COMPONENTS Widgets REQUIRED)


find_package(PCL REQUIRED)
if(PCL_FOUND)
    # link_directories must be before add_executable
    include_directories(${PCL_INCLUDE_DIRS})
    #add_definitions(${PCL_DEFINITIONS})
    link_directories(${PCL_LIBRARY_DIRS})
    link_libraries(${PCL_LIBRARIES})
endif()

#动态库
#find_package(OpenCV REQUIRED)
#link_libraries(${OpenCV_LIBS})
#静态库
link_libraries(
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/opencv3.4.11/lib/libopencv_core.a
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/opencv3.4.11/lib/libopencv_highgui.a
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/opencv3.4.11/lib/libopencv_imgproc.a
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/opencv3.4.11/lib/libopencv_imgcodecs.a
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/opencv3.4.11/lib/jasper/lib/liblibjasper.a
)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/opencv3.4.11/include
)


set(LINK_LIB_SHARED_LIST pthread z m dl IlmImf webp)

#yaml-cpp源码编译
set(yamlcpp_root ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/yaml)
include_directories(${yamlcpp_root}/include)
file(GLOB yamlcpp_src_list ${yamlcpp_root}/src/*.cpp ${yamlcpp_root}/src/contrib/*.cpp)
add_library(yamlcpp STATIC ${yamlcpp_src_list})
list(APPEND  LINK_LIB_STATIC_LIST yamlcpp)

#easylogger源码编译
set(easylogger_root ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/easylogger/easylogger)
include_directories(${easylogger_root}/inc
                    ${easylogger_root}/plugins/file
)
file(GLOB easylogger_src_list ${easylogger_root}/src/*.c ${easylogger_root}/port/*.c ${easylogger_root}/plugins/file/*.c)
add_library(easylogger STATIC ${easylogger_src_list})
list(APPEND  LINK_LIB_STATIC_LIST easylogger)

if(ANDROID)
  add_library(${PROJECT_NAME} SHARED
    main.cpp
    fusion.cpp
    fusion.h
    fusion.ui
    imgpointshow.cpp
    imgpointshow.h
    imgpointshow.ui
  )
else()
  add_executable(${PROJECT_NAME}
    main.cpp
    fusion.cpp
    fusion.h
    fusion.ui
    imgpointshow.cpp
    imgpointshow.h
    imgpointshow.ui
  )
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE Qt5::Widgets ${LINK_LIB_SHARED_LIST} ${LINK_LIB_STATIC_LIST})
