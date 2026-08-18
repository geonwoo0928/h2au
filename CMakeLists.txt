cmake_minimum_required(VERSION 3.14)
project(h2au_project CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(DEFINED ENV{LIBTORCH_PATH})
    list(APPEND CMAKE_PREFIX_PATH $ENV{LIBTORCH_PATH})
endif()

find_package(OpenCV REQUIRED)
find_package(Torch REQUIRED)

# main.cpp 외에 다른 .cpp 파일이 있다면 여기에 추가
add_executable(my_app
    main.cpp
    I2cDevice.cpp
    PwmController.cpp
    ServoController.cpp
    MotorController.cpp
    GpioOutput.cpp
    TerminalInput.cpp
    Camera.cpp
    GpsController.cpp
    SeoilCoordController.cpp
    RcCarDataManager.cpp
    PersonDetector.cpp
)

if(MSVC)
    target_compile_options(my_app PRIVATE /EHsc)
endif()

target_compile_options(my_app PRIVATE ${TORCH_CXX_FLAGS})

target_include_directories(my_app PRIVATE 
    ${OpenCV_INCLUDE_DIRS}
    ${TORCH_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(my_app PRIVATE 
    ${OpenCV_LIBS} 
    ${TORCH_LIBRARIES}
)
