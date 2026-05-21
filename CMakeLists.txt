CMake
cmake_minimum_required(VERSION 3.16)

# Set CMakeLists.txt for AeroGrid project
project(AeroGrid VERSION 1.0 LANGUAGES CXX)

# Set C++ standard to 17 and require it for all targets
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable automatic processing of Qt's MOC, RCC, and UIC files
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# Find the required Qt6 components
find_package(Qt6 COMPONENTS Core Gui Widgets REQUIRED)

# Include directories for core and GUI components
include_directories(
    ${CMAKE_SOURCE_DIR}/src/core
    ${CMAKE_SOURCE_DIR}/src/gui
)

# Define the source files for the AeroGrid application
set(SOURCES
    src/main.cpp
    src/core/Drone.cpp
    src/core/SimulationEngine.cpp
    src/gui/MainWindow.cpp
    src/gui/TelemetryModel.cpp
)

# Add the executable target for the AeroGrid application
add_executable(${PROJECT_NAME} ${SOURCES})

# Link the Qt6 libraries to the AeroGrid target
target_link_libraries(${PROJECT_NAME} PRIVATE
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
)