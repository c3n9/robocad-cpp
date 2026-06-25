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
    "src/internal/common/robot.cpp" 
    "src/internal/common/shared.cpp" 
    "src/internal/common/updaters.cpp" 
)

target_link_options(robocad-cpp PRIVATE "LINKER:/NODEFAULTLIB:library")
target_link_libraries(robocad-cpp PRIVATE ${OpenCV_LIBS})
target_link_libraries(robocad-cpp PRIVATE Threads::Threads)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET robocad-cpp PROPERTY CXX_STANDARD 20)
endif()

