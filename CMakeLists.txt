cmake_minimum_required (VERSION 3.8)

project ("robocad-cpp" VERSION 1.3.0)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED )

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("include")
include_directories("include/internal")
include_directories("include/internal/common")

add_library (robocad-cpp SHARED
    "src/shufflecad.cpp"
    "src/studica.cpp"
    "src/algaritm.cpp"
    "src/common.cpp"
    "src/internal/studica_internal.cpp"
    "src/internal/algaritm_internal.cpp"
    "src/internal/common_internal.cpp"
    "src/internal/common/connection_base.cpp"
    "src/internal/common/connection_real.cpp" 
    "src/internal/common/connection_sim.cpp" 
    "src/internal/common/lidar.cpp"
    "src/internal/common/n10_lidar.cpp"
    "src/internal/common/robot.cpp" 
    "src/internal/common/shared.cpp" 
    "src/internal/common/updaters.cpp" 
)

if (MSVC)
  target_link_options(robocad-cpp PRIVATE "LINKER:/NODEFAULTLIB:library")
endif()

# On Windows a SHARED lib emits an import .lib only if symbols are exported.
# The sources have no __declspec(dllexport), so auto-export everything - this
# lets RobocadDemo/ShufflecadDemo link against robocad-cpp.
if (WIN32)
  set_target_properties(robocad-cpp PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()
target_link_libraries(robocad-cpp PRIVATE ${OpenCV_LIBS})
target_link_libraries(robocad-cpp PRIVATE Threads::Threads)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET robocad-cpp PROPERTY CXX_STANDARD 20)
endif()

# Demo executable (analog of robocad-cs RobocadDemo). Uploaded to the robot and
# launched by Shufflecad. Disable with -DBUILD_ROBOCAD_DEMO=OFF.
option(BUILD_ROBOCAD_DEMO "Build the RobocadDemo executable" ON)
if (BUILD_ROBOCAD_DEMO)
  add_executable(RobocadDemo "RobocadDemo/main.cpp")
  target_link_libraries(RobocadDemo PRIVATE robocad-cpp)
  target_link_libraries(RobocadDemo PRIVATE ${OpenCV_LIBS})
  target_link_libraries(RobocadDemo PRIVATE Threads::Threads)
  if (CMAKE_VERSION VERSION_GREATER 3.12)
    set_property(TARGET RobocadDemo PROPERTY CXX_STANDARD 20)
  endif()
  # On Windows put the OpenCV runtime DLL next to the exe so it runs without
  # OpenCV on PATH.
  if (WIN32)
    add_custom_command(TARGET RobocadDemo POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:opencv_world> $<TARGET_FILE_DIR:RobocadDemo>)
  endif()
endif()

# Simulator demo (analog of robocad-cs ShufflecadDemo). Runs locally and streams
# synthetic variables to the Shufflecad app. Disable with -DBUILD_SHUFFLECAD_DEMO=OFF.
option(BUILD_SHUFFLECAD_DEMO "Build the ShufflecadDemo executable" ON)
if (BUILD_SHUFFLECAD_DEMO)
  add_executable(ShufflecadDemo "ShufflecadDemo/main.cpp")
  target_link_libraries(ShufflecadDemo PRIVATE robocad-cpp)
  target_link_libraries(ShufflecadDemo PRIVATE ${OpenCV_LIBS})
  target_link_libraries(ShufflecadDemo PRIVATE Threads::Threads)
  if (CMAKE_VERSION VERSION_GREATER 3.12)
    set_property(TARGET ShufflecadDemo PROPERTY CXX_STANDARD 20)
  endif()
  if (WIN32)
    add_custom_command(TARGET ShufflecadDemo POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:opencv_world> $<TARGET_FILE_DIR:ShufflecadDemo>)
  endif()
endif()

