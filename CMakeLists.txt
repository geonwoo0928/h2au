cmake_minimum_required(VERSION 3.14)
project(h2au_project CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

# ONNX Runtime는 공식 배포본에 CMake config가 없어(onnxruntimeConfig.cmake 미포함)
# find_package로 못 찾는다 - include/lib 경로를 직접 지정해야 한다.
# ONNXRUNTIME_ROOT 환경변수(또는 -DONNXRUNTIME_ROOT=<path>)로 SDK 압축 해제 경로를
# 알려줄 것. 공식 배포: https://github.com/microsoft/onnxruntime/releases
#   RPi4(Linux aarch64): onnxruntime-linux-aarch64-<ver>.tgz (LibTorch와 달리
#   공식 prebuilt가 있음 - 이게 LibTorch에서 ONNX Runtime으로 바꾼 이유)
if(DEFINED ENV{ONNXRUNTIME_ROOT} AND NOT DEFINED ONNXRUNTIME_ROOT)
    set(ONNXRUNTIME_ROOT $ENV{ONNXRUNTIME_ROOT})
endif()
if(NOT DEFINED ONNXRUNTIME_ROOT)
    message(FATAL_ERROR
        "ONNXRUNTIME_ROOT가 설정되지 않았습니다. ONNX Runtime C/C++ SDK를 받아서 "
        "-DONNXRUNTIME_ROOT=<path> 또는 환경변수 ONNXRUNTIME_ROOT로 압축 해제 경로를 지정하세요. "
        "(https://github.com/microsoft/onnxruntime/releases)")
endif()

find_library(ONNXRUNTIME_LIB
    NAMES onnxruntime
    PATHS ${ONNXRUNTIME_ROOT}/lib
    REQUIRED
)

add_executable(my_app
    main.cpp
    hardware/I2cDevice.cpp
    hardware/PwmController.cpp
    hardware/ServoController.cpp
    hardware/MotorController.cpp
    hardware/GpioOutput.cpp
    sensors/GpsController.cpp
    sensors/ImuController.cpp
    coord/SeoilCoordController.cpp
    coord/RcCarDataManager.cpp
    coord/PersonLocationCalculator.cpp
    detection/PersonDetector.cpp
    utils/Camera.cpp
    utils/TerminalInput.cpp
)

target_include_directories(my_app PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${ONNXRUNTIME_ROOT}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/hardware
    ${CMAKE_CURRENT_SOURCE_DIR}/sensors
    ${CMAKE_CURRENT_SOURCE_DIR}/coord
    ${CMAKE_CURRENT_SOURCE_DIR}/detection
    ${CMAKE_CURRENT_SOURCE_DIR}/utils
)

target_link_libraries(my_app PRIVATE
    ${OpenCV_LIBS}
    ${ONNXRUNTIME_LIB}
)

# 실행 시점에 onnxruntime.so를 찾도록 rpath에 심어둔다 - 안 하면 배포 시
# LD_LIBRARY_PATH를 매번 손으로 맞춰야 한다.
set_target_properties(my_app PROPERTIES
    BUILD_RPATH ${ONNXRUNTIME_ROOT}/lib
    INSTALL_RPATH ${ONNXRUNTIME_ROOT}/lib
)
