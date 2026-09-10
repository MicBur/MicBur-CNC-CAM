# Test Geometry (STL, DXF, Mesh, Contour)
add_executable(TestGeometry TestGeometry.cpp)
target_link_libraries(TestGeometry PRIVATE
    cnc_core
    cnc_geometry
    Qt6::Core
)
add_test(NAME TestGeometry COMMAND TestGeometry)

# Test CAM Engine (Toolpath, Generator, Collision Detector)
add_executable(TestCamEngine TestCamEngine.cpp)
target_link_libraries(TestCamEngine PRIVATE
    cnc_core
    cnc_geometry
    cnc_cam
    Qt6::Core
)
add_test(NAME TestCamEngine COMMAND TestCamEngine)

# Test Hardware (MoonrakerClient, Mock, JogController, TMC2209)
add_executable(TestHardware TestHardware.cpp)
target_link_libraries(TestHardware PRIVATE
    cnc_core
    cnc_hardware
    Qt6::Core
    Qt6::Network
)
add_test(NAME TestHardware COMMAND TestHardware)

# Test WinMax (ObjLoader, MaterialDatabase, Conversational, 3D Finishing, Probing, Softkeys)
add_executable(TestWinMax TestWinMax.cpp)
target_link_libraries(TestWinMax PRIVATE
    cnc_core
    cnc_geometry
    cnc_cam
    cnc_hardware
    cnc_ui
    Qt6::Core
    Qt6::Network
    Qt6::Widgets
)
add_test(NAME TestWinMax COMMAND TestWinMax)
