cmake_minimum_required(VERSION 3.16)
project(AeroGrid LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable the Meta-Object Compiler (MOC)
# This is required for the SimulationEngine class because it uses the Q_OBJECT macro.
set(CMAKE_AUTOMOC ON)

# Find the required Qt components for the GUI application.
find_package(Qt6 REQUIRED COMPONENTS Core Widgets)

add_executable(AeroGrid
    src/main.cpp
    src/gui/MainWindow.cpp
    src/gui/TelemetryModel.cpp
    src/core/SimulationEngine.cpp
    src/core/Drone.cpp
)

target_link_libraries(AeroGrid PRIVATE Qt6::Core Qt6::Widgets)

# Add the source root so includes like "gui/MainWindow.h" are found correctly.
target_include_directories(AeroGrid PRIVATE src)