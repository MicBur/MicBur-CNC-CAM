cmake_minimum_required(VERSION 3.20)
project(GeminiCNC VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Export compile commands for IDE indexing
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Enable automatic MOC, UIC and RCC for Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Find Qt6 components
find_package(Qt6 REQUIRED COMPONENTS
    Core
    Gui
    Widgets
    OpenGL
    OpenGLWidgets
    Network
)

# Compiler warnings and optimization
if(MSVC)
    add_compile_options(/W4 /permissive- /utf-8)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Core Module
add_library(cnc_core STATIC
    src/core/Vector3D.h
    src/core/Vector3D.cpp
    src/core/BoundingBox.h
    src/core/BoundingBox.cpp
    src/core/ToolDefinition.h
    src/core/ToolDefinition.cpp
    src/core/MachineConfig.h
    src/core/MachineConfig.cpp
    src/core/MaterialDatabase.h
    src/core/MaterialDatabase.cpp
)
target_link_libraries(cnc_core PUBLIC
    Qt6::Core
    Qt6::Gui
)

# Geometry Module (STL, OBJ, DXF, Mesh)
add_library(cnc_geometry STATIC
    src/geometry/Mesh.h
    src/geometry/Mesh.cpp
    src/geometry/StlLoader.h
    src/geometry/StlLoader.cpp
    src/geometry/ObjLoader.h
    src/geometry/ObjLoader.cpp
    src/geometry/DxfLoader.h
    src/geometry/DxfLoader.cpp
    src/geometry/Contour.h
    src/geometry/Contour.cpp
    src/geometry/ContourSolver.h
    src/geometry/ContourSolver.cpp
    src/geometry/PolygonOffset.h
    src/geometry/PolygonOffset.cpp
)
target_link_libraries(cnc_geometry PUBLIC
    cnc_core
    Qt6::Core
    Qt6::Gui
)

# CAM Math & Collision Module
add_library(cnc_cam STATIC
    src/cam/Toolpath.h
    src/cam/Toolpath.cpp
    src/cam/ToolpathGenerator.h
    src/cam/ToolpathGenerator.cpp
    src/cam/CollisionDetector.h
    src/cam/CollisionDetector.cpp
    src/cam/ConversationalBlock.h
    src/cam/ConversationalBlock.cpp
    src/cam/ConversationalProgram.h
    src/cam/ConversationalProgram.cpp
    src/cam/PostProcessor.h
    src/cam/PostProcessor.cpp
    src/cam/PostProcessorFactory.cpp
    src/cam/HurcoPostProcessor.h
    src/cam/HurcoPostProcessor.cpp
    src/cam/HeidenhainPostProcessor.h
    src/cam/HeidenhainPostProcessor.cpp
    src/cam/MeshSlicer.h
    src/cam/MeshSlicer.cpp
    src/cam/FeatureRecognizer.h
    src/cam/FeatureRecognizer.cpp
    src/cam/AutoProgramGenerator.h
    src/cam/AutoProgramGenerator.cpp
)
target_link_libraries(cnc_cam PUBLIC
    cnc_core
    cnc_geometry
    Qt6::Core
    Qt6::Gui
)

# Simulation Engine Module
add_library(cnc_simulation STATIC
    src/simulation/SimulationEngine.h
    src/simulation/SimulationEngine.cpp
    src/simulation/StockModel.h
    src/simulation/StockModel.cpp
)
target_link_libraries(cnc_simulation PUBLIC
    cnc_core
    cnc_geometry
    cnc_cam
    Qt6::Core
)

# Hardware Interface Module (Klipper / Moonraker / TMC2209 / Probing)
add_library(cnc_hardware STATIC
    src/hardware/IKlipperClient.h
    src/hardware/MoonrakerClient.h
    src/hardware/MoonrakerClient.cpp
    src/hardware/JogController.h
    src/hardware/JogController.cpp
    src/hardware/ProbeController.h
    src/hardware/ProbeController.cpp
)
target_link_libraries(cnc_hardware PUBLIC
    cnc_core
    Qt6::Core
    Qt6::Network
)

# UI Module (3D Viewport & Dialogs)
add_library(cnc_ui STATIC
    src/ui/viewport/ShaderProgram.h
    src/ui/viewport/ShaderProgram.cpp
    src/ui/viewport/Viewport3D.h
    src/ui/viewport/Viewport3D.cpp
    src/ui/viewport/GPUStockModel.h
    src/ui/viewport/GPUStockModel.cpp
    src/ui/dialogs/DialogStack.h
    src/ui/dialogs/DialogStack.cpp
    src/ui/dialogs/SetupDialog.h
    src/ui/dialogs/SetupDialog.cpp
    src/ui/dialogs/ToolManagerDialog.h
    src/ui/dialogs/ToolManagerDialog.cpp
    src/ui/dialogs/ToolGraphicsWidget.h
    src/ui/dialogs/ToolGraphicsWidget.cpp
    src/ui/dialogs/ContourDialog.h
    src/ui/dialogs/ContourDialog.cpp
    src/ui/dialogs/ConversationalEditorDialog.h
    src/ui/dialogs/ConversationalEditorDialog.cpp
    src/ui/dialogs/ProbeDialog.h
    src/ui/dialogs/ProbeDialog.cpp
    src/ui/dialogs/SimulationDialog.h
    src/ui/dialogs/SimulationDialog.cpp
    src/ui/dialogs/JogDialog.h
    src/ui/dialogs/JogDialog.cpp
    src/ui/dialogs/HardwareDialog.h
    src/ui/dialogs/HardwareDialog.cpp
    src/ui/dialogs/ContourSegmentEditorDialog.h
    src/ui/dialogs/ContourSegmentEditorDialog.cpp
    src/ui/dialogs/FeaturePreviewDialog.h
    src/ui/dialogs/FeaturePreviewDialog.cpp
    src/ui/winmax/WinMaxHeaderBar.h
    src/ui/winmax/WinMaxHeaderBar.cpp
    src/ui/winmax/WinMaxSoftkeyBar.h
    src/ui/winmax/WinMaxSoftkeyBar.cpp
    src/ui/SimulationWindow.h
    src/ui/SimulationWindow.cpp
    src/ui/MainWindow.h
    src/ui/MainWindow.cpp
)
target_link_libraries(cnc_ui PUBLIC
    cnc_core
    cnc_geometry
    cnc_cam
    cnc_simulation
    cnc_hardware
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    Qt6::Network
)

# Main Executable (Programmsymbol: resources/icons/app_icon.ico, Fenstersymbol über Qt-Ressource)
set(GEMINICNC_APP_RESOURCES resources/app.qrc)
if(WIN32)
    enable_language(RC)
    list(APPEND GEMINICNC_APP_RESOURCES resources/app.rc)
endif()
add_executable(GeminiCNC WIN32
    src/main.cpp
    ${GEMINICNC_APP_RESOURCES}
)
target_link_libraries(GeminiCNC PRIVATE
    cnc_ui
)

# Test Suite
enable_testing()
add_subdirectory(tests)
