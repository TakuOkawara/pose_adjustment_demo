# Overview
* This repository has some examples of pose adjustment using a pose graph.
* All pose graphs are implemented as a factor graph defined by GTSAM.
* This repository was used for demonstration of [this seminar](https://www.j-techno.co.jp/seminar/seminar-63357/).

# Dependencies
* [gtsam](https://github.com/borglab/gtsam)
  * Library for factor graph optimization
* [iridescence](https://github.com/koide3/iridescence)
  * Library for visualization

We tested all codes with Ubuntu 22.04.

# Preparation to execute sample codes
* Please execute the following commands before executing sample codes.
* All sample codes assume that you are in the build directory made by the following command.
```commandline
git clone https://github.com/TakuOkawara/pose_adjustment_demo.git
cd pose_adjustment_demo
mkdir build
cd build
```

# How to execute each sample code
## [small_pose_adjustment_2d.cpp](https://github.com/TakuOkawara/pose_adjustment_demo/blob/main/src/small_pose_adjustment_2d.cpp)
* This example is a small pose graph optimization using a pose graph.
* We made this example referring to [official gtsam tutorial](https://github.com/borglab/gtsam/blob/develop/examples/Pose2SLAMExample.cpp)
* This code can be executed by the following commands
```commandline
cd build
cmake ..
make
./small_pose_adjustment_2d 
```

## [pose_adjustment_2d.cpp](https://github.com/TakuOkawara/pose_adjustment_demo/blob/main/src/pose_adjustment_2d.cpp)
* This example is **2D pose graph optimization using a pose graph**.
* This code uses the [Manhattan dataset](https://github.com/OpenSLAM-org/openslam_vertigo).
* This code can be executed by the following commands
```commandline
cd build
cmake ..
make
./pose_adjustment_2d 
```
<img src="https://github.com/user-attachments/assets/aa1a0767-bb21-4f58-828b-c700c19d14f0" alt="image" width="50%">

## [pose_adjustment_3d.cpp](https://github.com/TakuOkawara/pose_adjustment_demo/blob/main/src/pose_adjustment_3d.cpp)
* This example is **3D pose graph optimization using a pose graph**.
* This code uses the [Sphere dataset](https://github.com/OpenSLAM-org/openslam_vertigo).
* This code can be executed by the following commands
```commandline
cd build
cmake ..
make
./pose_adjustment_3d 
```
<img src="https://github.com/user-attachments/assets/70f02f60-711d-4d36-87e7-1e4fe8c626d4" alt="image" width="50%">


## [pose_adjustment_3d_with_customfactor.cpp](https://github.com/TakuOkawara/pose_adjustment_demo/blob/main/src/pose_adjustment_3d_with_customfactor.cpp)
* This example is 3D pose graph optimization using a pose graph implemented by a custom factor.
* [The custom factor](https://github.com/TakuOkawara/pose_adjustment_demo/blob/main/src/customfactor.cpp) is created by inheriting the NoiseModelFactor; thus **this code is a tutorial to make a new factor (constraint for SLAM) using GTSAM**.
  * This custom between factor is the same as gtsam::BetweenFactor< gtsam::Pose3 >.
* Custom factors are important for implementing novel factors that are still not implemented in the official GTSAM repository, such as [full_linear_wheel_odometry_factor](https://github.com/TakuOkawara/full_linear_wheel_odometry_factor)
  * full_linear_wheel_odometry_factor is a custom factor to consider wheel slippage and wheel radius errors for skid-steering robots.
* This code also uses the Sphere dataset; thus this result is the same as [pose_adjustment_3d.cpp](https://github.com/TakuOkawara/pose_adjustment_demo/blob/main/src/pose_adjustment_3d.cpp).
* This code can be executed by the following commands
```commandline
cd build
cmake ..
make
./pose_adjustment_3d_with_customfactor 
```

# Trouble shooting

## error while loading shared libraries: libmetis-gtsam.so: cannot open shared object file: No such file or directory
Please execute the following commands when the above execution error occurs.
```commandline
echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
source ~/.bashrc
```
