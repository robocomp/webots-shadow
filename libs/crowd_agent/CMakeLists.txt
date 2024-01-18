# Lista de archivos de código fuente
set(SOURCES
    CrowdAgent.cpp
)

# Genera la librería compartida
add_library(crowd_agent SHARED ${SOURCES})

# Linking Webots libraries
set(WEBOTS_HOME "/usr/local/webots")
link_directories(${WEBOTS_HOME}/lib/controller)
SET (LIBS ${LIBS} -L/usr/local/webots/lib/controller Controller CppController)
include_directories(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)

# Enlaza la librería con Webots
target_link_libraries(crowd_agent 
    /usr/local/webots/lib/controller/libController.so
    /usr/local/webots/lib/controller/libCppController.so
)  # Ajusta el nombre de la biblioteca según sea necesario

# Instala la librería compartida en /usr/local/libs
install(TARGETS crowd_agent LIBRARY DESTINATION /usr/local/lib)