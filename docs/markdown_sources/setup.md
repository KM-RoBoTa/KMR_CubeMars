# How to setup {#setup}
[TOC]

## Dependencies
This library is dependent on the following tools that need to be installed first:
- CMake
- (Doxygen and Graphviz if you wish to regenerate the documentation locally)

```bash
sudo pacman -S cmake doxygen graphviz
```

## Installation
This library does not need to be installed per se. It has to be built into a static library (.a) in a "build" folder: 
```bash
cd <your-path>
git clone https://github.com/KM-RoBoTa/KMR_CubeMars
cd KMR_CubeMars
mkdir build && cd build
cmake ../
make
```
This will generate the static library ```libKMR_dxl.a``` in the "build" folder, as well as several executable examples. If doxygen and Graphviz are installed, it will also generate local doxygen documentation in ```KMR_CubeMars/docs/generated_docs/html/index.html```.


## Include in a project
It is very straightforward to include this library in a project. \n
All the headers are located in "KMR_CubeMars/include", and, as already mentioned, the static libraries in "KMR_CubeMars/build".

In the root CMakeLists.txt of the project, add:

```
# Path to KMR_CubeMars's CMakeLists
add_subdirectory(path-to-KMR_CubeMars)

# Path to KMR_CubeMars's header files
target_include_directories(project PUBLIC path-to-KMR_CubeMars/include)

# Path to the static (.a) library
target_link_directories(project PRIVATE path-to-KMR_CubeMars/build)

# Link the library
target_link_libraries(project KMR_CubeMars)
```

In the source code, only one header needs to be included:

```cpp
#include "KMR_CubeMars.hpp"
```

## Add a model

To add a motor model to the library, follow those steps:

1. Make sure its protocol is compatible with the other models
2. In ```config/motor_models.hpp```, create a new structure, inspired from the already existing ones. Enter the min and max speed and torque of the model. 
3. Add the model to the ```Model``` enumeration in ```config/structures.hpp```
4. Still in ```config/structures.hpp```, in the ```Motor``` structure, edit the switch in its constructor to add the case for the new model.

Next: how to [use](#how-to-use)