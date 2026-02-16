## FCL -- The Flexible Collision Library 

Linux / OS X [![Build Status](https://travis-ci.org/flexible-collision-library/fcl.svg?branch=master)](https://travis-ci.org/flexible-collision-library/fcl)
Windows [![Build status](https://ci.appveyor.com/api/projects/status/do1k727uu6e8uemf/branch/master?svg=true)](https://ci.appveyor.com/project/flexible-collision-library/fcl)
Coverage [![Coverage Status](https://coveralls.io/repos/github/flexible-collision-library/fcl/badge.svg?branch=master)](https://coveralls.io/github/flexible-collision-library/fcl?branch=master)

FCL is a library for performing three types of proximity queries on a pair of
geometric models composed of triangles.
 - Collision detection: detecting whether the two models overlap, and
   optionally, all of the triangles that overlap.
 - Distance computation: computing the minimum distance between a pair of
   models, i.e., the distance between the closest pair of points.
 - Tolerance verification: determining whether two models are closer or farther
   than a tolerance distance.
 - Continuous collision detection: detecting whether the two moving models
   overlap during the movement, and optionally, the time of contact.
 - Contact information: for collision detection and continuous collision
   detection, the contact information (including contact normals and contact
   points) can be returned optionally.

FCL has the following features
 - C++ interface
 - Compilable for either linux or win32 (both makefiles and Microsoft Visual
   projects can be generated using cmake)
 - No special topological constraints or adjacency information required for
   input models – all that is necessary is a list of the model's triangles
 - Supported different object shapes:
  + box
  + sphere
  + ellipsoid
  + capsule
  + cone
  + cylinder
  + convex
  + half-space
  + plane
  + mesh
  + octree (optional, octrees are represented using the octomap library
    http://octomap.github.com)


## Installation

### Prerequisites
Before compiling FCL, ensure you have the necessary dependencies installed on your system (Ubuntu/Debian).

1. Install System Dependencies
Open a terminal and run:

``` bash
sudo apt-get update
sudo apt-get install cmake build-essential
sudo apt-get install liboctomap-dev libccd-dev
sudo apt-get install freeglut3-dev libgl1-mesa-dev libglu1-mesa-dev
```

2. Install Eigen (Math Library)
FCL relies heavily on Eigen.

```bash
sudo apt-get install libeigen-eigen-dev
```

CMakeLists.txt is used to generate makefiles in Linux or Visual studio projects
in windows. In command line, run

```bash
# Clone the repository
git clone https://github.com/Ignitarium-AI/fcl_fork.git
cd fcl_fork

# Create build directory
mkdir build && cd build

# Configure and install
cmake ..
make -j4
sudo make install
```

## Running the sample code

1. Navigate to the project directory:

```bash 
cd sample
```
2. Create a build directory:

```bash
mkdir build
cd build
```
3. Configure the project with CMake:

```bash 
cmake ..
```

4. Compile the code:

```bash
make -j4
```

5. Running the Visualizer
  

```bash
./fcl_visualizer
```

### More examples from the FCL Library

For more examples, please refer to the test folder:
- test_fcl_collision.cpp: provide examples for collision test
- test_fcl_distance.cpp: provide examples for distance test
- test_fcl_broadphase.cpp: provide examples for broadphase collision/distance
  test
- test_fcl_frontlist.cpp: provide examples for frontlist collision acceleration
- test_fcl_octomap.cpp: provide examples for collision/distance computation
  between octomap data and other data types.
