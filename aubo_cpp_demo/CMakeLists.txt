CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(AUBO_DEMO)

SET(CMAKE_CXX_FLAGS "-std=c++11")
SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# robotSDK
set(auboSDK_INCLUDE ${PROJECT_SOURCE_DIR}/dependents/robotSDK/inc)
set(auboSDK_LIBS ${PROJECT_SOURCE_DIR}/dependents/robotSDK/lib/linux_x64)

# log4cplus
# in bash: EXPORT export LD_LIBRARY_PATH=~/App/log4cplus/linux_x64/lib:$LD_LIBRARY_PATH
set(log4cplus_DIR "~/App/log4cplus/linux_x64")
set(log4cplus_INCLUDE ${log4cplus_DIR}/inc)
set(log4cplus_LIBS ${log4cplus_DIR}/lib)

# include
INCLUDE_DIRECTORIES(
    include
    ${log4cplus_INCLUDE}
    ${auboSDK_INCLUDE}
)

# link libs
link_directories(
    ${log4cplus_LIBS}
    ${auboSDK_LIBS}
)

add_executable(aubo_demo src/main.cpp
    src/example_0.cpp    
)
target_link_libraries(aubo_demo auborobotcontroller log4cplus)