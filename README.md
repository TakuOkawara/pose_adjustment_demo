# Overview
* This repository has some examples of pose adjustment using a pose graph.
* All pose graphs are implemented as a factor graph defined by GTSAM.

# Preparation to execute sample codes
* Please execute the following commands before executing sample codes.
* All sample codes assume that you are in the build directory made by the following command.
```commandline
git clone
cd pose_adjustment_demo
mkdir build
cd build
```

# Example files
## small_pose_adjustment_2d.cpp
* This example is a small pose graph optimization using a pose graph.
* We made this example referring to [official gtsam tutorial](https://github.com/borglab/gtsam/blob/develop/examples/Pose2SLAMExample.cpp)
* This code can be executed by the following commands
```commandline
cd build
cmake ..
make
./small_pose_adjustment_2d 
```

## pose_adjustment_2d.cpp
* This example is **2D pose graph optimization using a pose graph**.
* This code uses the Manhattan dataset.
* This code can be executed by the following commands
```commandline
cd build
cmake ..
make
./pose_adjustment_2d 
```

## pose_adjustment_3d.cpp
* This example is **3D pose graph optimization using a pose graph**.
* This code uses the Sphere dataset.
* This code can be executed by the following commands
```commandline
cd build
cmake ..
make
./pose_adjustment_3d 
```

## pose_adjustment_3d_with_customfactor.cpp
* This example is 3D pose graph optimization using a pose graph implemented by a custom factor.
* The custom factor is created by inheriting the NoiseModelFactor; thus **this code is a tutorial to make a new factor using GTSAM**.
* This code also uses the Sphere dataset; thus this result is the same as pose_adjustment_3d.cpp.
* This code can be executed by the following commands
```commandline
cd build
cmake ..
make
./pose_adjustment_3d_with_customfactor 
```
