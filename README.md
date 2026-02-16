## FCL -- The Flexible Collision Library 

Official readme [link](https://github.com/flexible-collision-library/fcl/blob/master/README.md)


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

### Explanation of Results

**Blue Cube**: Represents obj1.

**Red Cube**: Represents obj2.

**Yellow Sphere**: They mark the specific coordinates $(x, y, z)$ where the two shapes (e.g., the blue box and the red box) are actually touching or penetrating each other.
If the objects are overlapping significantly, FCL calculates the "deepest" point of overlap or a set of points that best describe the contact patch.

If you see multiple yellow spheres instead of just one, it means the collision surface is complex or flat-against-flat (like two boxes resting on each other).

**Single Point**: Typical for Sphere-Sphere or Sphere-Box collision.

**Multiple Points (Manifold)**: Typical for Box-Box collision where a whole face or edge is intersecting. FCL generates multiple contact points to stabilize the collision response (preventing the boxes from wobbling).

You should visualize the following image:

![Collision of two cubes](assets/image.png)

### More examples from the FCL Library

For more examples, please refer to the test folder:
- test_fcl_collision.cpp: provide examples for collision test
- test_fcl_distance.cpp: provide examples for distance test
- test_fcl_broadphase.cpp: provide examples for broadphase collision/distance
  test
- test_fcl_frontlist.cpp: provide examples for frontlist collision acceleration
- test_fcl_octomap.cpp: provide examples for collision/distance computation
  between octomap data and other data types.
