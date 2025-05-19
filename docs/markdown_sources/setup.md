# How to setup {#setup}
[TOC]

## Operating system 
The library uses two threads, one of them being the Listener thread that listens to the CAN bus. \n 
When using a kernel configuration that does not support sufficient kernel preemption, there are delays due to the scheduling of the Listener thread. During the first tests, using an armv7l architecture with an SMP kernel, those delays happened in around 2% of loops, resulting in those loops being much longer than what they're supposed to be (10-20 ms instead of about 2 ms), throwing off the control loop.

Using the official image of Archlinux-RPI, using the aarch64 architecture (PREEMPT_DYNAMIC kernel configuration) instantly solved this problem.

To check if the operating system supports sufficient preemption, it is recommended to run the examples (see more in the [use section](#how-to-use)). At the end of the program, there should be no overtimes reported.

## Dependencies
This library is dependent on the following tools that need to be installed first:
- CMake
- (Doxygen and Graphviz if you wish to regenerate the documentation locally, with Mathjax to have pretty equations)

```bash
sudo pacman -S cmake doxygen graphviz mathjax
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


## Set the CAN bus up
This step needs to be done on every start-up of your computer/raspberry PI/... (assuming your bus is called "can0"):

```bash
sudo ip link set can0 up type can bitrate 1000000
```


Next: how to [use](#how-to-use)