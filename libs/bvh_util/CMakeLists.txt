# Lista de archivos de código fuente
set(SOURCES
    bvh_util.c
    quaternion.c
    vector3.c
)

# Genera la librería compartida
add_library(bvh_util SHARED ${SOURCES})

# Linking Webots libraries
set(WEBOTS_HOME "/usr/local/webots")
link_directories(${WEBOTS_HOME}/lib/controller)
SET (LIBS ${LIBS} -L/usr/local/webots/lib/controller Controller CppController)
include_directories(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)

# Enlaza la librería con Webots
target_link_libraries(bvh_util 
    /usr/local/webots/lib/controller/libController.so
    /usr/local/webots/lib/controller/libCppController.so
)  # Ajusta el nombre de la biblioteca según sea necesario

# Instala la librería compartida en /usr/local/libs
install(TARGETS bvh_util LIBRARY DESTINATION /usr/local/lib)