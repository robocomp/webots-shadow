# Installation of Shadow workspace on Webots Simulator

This document provides detailed instructions on how to install and use the "webots-shadow" workspace. This workspace  

## Prerequisites

Make sure you have the following prerequisites installed before proceeding:

- [Webots](https://cyberbotics.com/)

## Installation Steps

1. **Clone the Repository:**

    Clone the library repository from GitHub:

    ```bash
    git clone https://github.com/robocomp/webots-shadow
    ```

2. **Compile the Library**

    Navigate to the library directory and compile the source code:

    ```bash
    cd webots-shadow
    mkdir build
    cd build
    cmake ..
    make
    ```

3. **Install the Library**

    Install the library on the system

    ```bash
    sudo make install
    sudo ldconfig
    ```

## Using the Library

Now that the library is installed, you can use it in your webots controller in this workspace.

Have fun!
