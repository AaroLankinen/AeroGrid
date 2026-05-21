cmake_minimum_required(VERSION 3.16)

project(AeroGrid LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable automatic handling of Qt's Meta-Object Compiler (MOC) and UI tools
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 COMPONENTS Core Gui Widgets Test REQUIRED)

# Define source groups for clarity as the project grows
set(CORE_SOURCES
    src/core/SimulationEngine.cpp
    src/core/SimulationEngine.h
    src/core/Drone.h
)

set(GUI_SOURCES
    src/gui/MainWindow.cpp
    src/gui/MainWindow.h
    src/gui/TelemetryModel.cpp
    src/gui/TelemetryModel.h
)

# Main Application Executable
add_executable(AeroGrid 
    src/main.cpp 
    ${CORE_SOURCES}
    ${GUI_SOURCES}
)
target_link_libraries(AeroGrid PRIVATE Qt6::Widgets)
target_include_directories(AeroGrid PRIVATE ${CMAKE_SOURCE_DIR}/src/core ${CMAKE_SOURCE_DIR}/src/gui)

# --- Unit Testing Setup ---
enable_testing()

add_executable(AeroGridTests 
    tests/TestAeroGrid.cpp
    src/core/SimulationEngine.cpp
    src/gui/TelemetryModel.cpp
)

target_link_libraries(AeroGridTests PRIVATE 
    Qt6::Core
    Qt6::Widgets
    Qt6::Test
)

target_include_directories(AeroGridTests PRIVATE ${CMAKE_SOURCE_DIR}/src/core ${CMAKE_SOURCE_DIR}/src/gui)
add_test(NAME AeroGridTests COMMAND AeroGridTests)