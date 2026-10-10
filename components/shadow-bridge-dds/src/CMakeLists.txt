# Zero-copy DDS media plane (rc::media) — the shared lib in active_inference/common/media_transport.
# This component is NOT a DSR agent, so FastDDS is linked standalone.
set(MEDIA_TRANSPORT_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../../active_inference/common/media_transport)
include(${MEDIA_TRANSPORT_DIR}/idl_sources.cmake)   # AI_COMMON_IDL_SOURCES

# Sources set
LIST(APPEND SOURCES
    ../src/specificworker.cpp
    ../src/media_plane.cpp
    # LiDAR robot-body self-filter, copied VERBATIM from robocomp-robolab lidar3d_dds (2026-10-10) so
    # the cloud is filtered exactly as the driver filtered it.
    ../src/mesh_filter.cpp
    ../src/urdf_mesh_loader.cpp
    ../src/webots_proto_loader.cpp
    ${MEDIA_TRANSPORT_DIR}/media_transport.cpp
    ${AI_COMMON_IDL_SOURCES}
)

# Headers set
LIST(APPEND HEADERS
    ../src/specificworker.h
    ../src/media_plane.h
    ../src/media_descriptor_servant.h
    ../src/mesh_filter.h
    ../src/urdf_mesh_loader.h
    ../src/webots_proto_loader.h
    fixedsizedeque.h
)

LIST(APPEND LIBS ${LIBS})

# NO -march=native: the media plane is shared with the DSR fleet, and active_inference/CLAUDE.md bans
# it there (it changes Eigen's alignment and so sizeof of shared types). webots-bridge used it.
add_compile_options(
    -O3
    -fno-plt
    -fomit-frame-pointer
    -funroll-loops
)
add_definitions(-fmax-errors=1 -Wno-unknown-pragmas)

INCLUDE( $ENV{ROBOCOMP}/cmake/modules/opencv4.cmake )

include_directories(${MEDIA_TRANSPORT_DIR})
find_package(fastcdr REQUIRED)
find_package(fastdds REQUIRED)
find_package(OpenMP REQUIRED)
LIST(APPEND LIBS ${LIBS} fastdds fastcdr embree4 assimp tinyxml2 OpenMP::OpenMP_CXX)

# Webots controller libraries
set(WEBOTS_HOME "/usr/local/webots")
link_directories(${WEBOTS_HOME}/lib/controller)
SET (LIBS ${LIBS} -L/usr/local/webots/lib/controller Controller CppController)
include_directories(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)
