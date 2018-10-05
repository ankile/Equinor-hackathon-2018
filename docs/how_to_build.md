# Build System Overview

In ROS, code is seperated into packages to simplify development. A package is a collection of (related) executables compiled together. All packages are located in `catkin_ws/src` as seperate folders. Each package should be loosely coupled, and independent of other packages.

## Each package contains:

### CMakelists.txt

This file describes how to build the package. If you want to seperate your code into multiple source files, you can add the new source files to an existing `add_executable` in this file

### package.xml

This file contains all build and run requirements for your package. There shouldn't be a need to edit this file. 

### src/
The src folder should contain all C++ source files (`*.cpp/`)

### scripts/
The scripts folder contains all python scripts

### include/
The include folder should contain all header files (`*.h/*.hpp`)

## How to build a package:
All packages are built by the catkin build system. To build all packages run, go to the `catkin_ws` folder and run:
```
catkin_make
```

All packages have to build successfully for catkin to complete successfully. 

## Potential issues

### I get some weird build errors I can't explain!
1. Run `catkin_make clean` to clear build files and run `catkin_make` (from `catkin_ws`)
2. If 1. didn't work, delete devel/ and build/ by running `rm -rf devel build` from `catkin_ws` and run `catkin_make`
