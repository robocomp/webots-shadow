# Makefile principal

# Directorios
LIBS_DIR := libs
CONTROLLERS_DIR := controllers

# Objetivo por defecto
all: libs controllers

# Compilar todas las carpetas en 'libs'
libs:
	@for dir in $(wildcard $(LIBS_DIR)/*); do \
        if [ -f $$dir/Makefile ]; then \
            echo "Compiling $$dir..."; \
            $(MAKE) -C $$dir; \
			echo ""; \
        fi; \
    done

# Compilar todas las carpetas en 'controllers'
controllers:
	@for dir in $(wildcard $(CONTROLLERS_DIR)/*); do \
        if [ -f $$dir/Makefile ]; then \
            echo "Compiling $$dir..."; \
            $(MAKE) -C $$dir; \
			echo ""; \
        fi; \
    done

# Limpiar todos los directorios
clean:
	@for dir in $(wildcard $(LIBS_DIR)/*); do \
        if [ -f $$dir/Makefile ]; then \
            echo "Cleaning $$dir..."; \
            $(MAKE) -C $$dir clean; \
			echo ""; \
        fi; \
    done
	@for dir in $(wildcard $(CONTROLLERS_DIR)/*); do \
        if [ -f $$dir/Makefile ]; then \
            echo "Cleaning $$dir..."; \
            $(MAKE) -C $$dir clean; \
			echo ""; \
        fi; \
    done

.PHONY: all libs controllers clean
