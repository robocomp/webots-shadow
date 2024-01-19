# Lista de módulos de librería
set(LIB_DIRS
    animation_skin
    crowd_agent
)

# Definir el destino de instalación para las librerías compartidas
install(DIRECTORY DESTINATION /usr/local/lib)

include_directories(/usr/local/webots/include)

# Agrega cada módulo de librería como un subdirectorio
foreach(LIB_DIR ${LIB_DIRS})
    add_subdirectory(${LIB_DIR})
endforeach()