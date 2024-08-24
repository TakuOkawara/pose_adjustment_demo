# Overview
* This repository has some examples of pose adjustment using a pose graph.
* All pose graphs are implemented as a factor graph defined by GTSAM.
* This repository was used for demonstration of [this seminar](https://www.j-techno.co.jp/seminar/seminar-63357/).
  * 日本語のPDF資料は、[本レポジトリ内](https://github.com/TakuOkawara/pose_adjustment_demo/blob/main/%E8%B3%87%E6%96%99%20SLAM%EF%BC%88%E8%87%AA%E5%B7%B1%E4%BD%8D%E7%BD%AE%E6%8E%A8%E5%AE%9A%E3%83%BB%E5%9C%B0%E5%9B%B3%E6%A7%8B%E7%AF%89%EF%BC%89%E3%81%AE%E5%9F%BA%E7%A4%8E%E3%81%A8%E9%AB%98%E6%80%A7%E8%83%BD%E5%8C%96%E6%8A%80%E8%A1%93%E3%81%8A%E3%82%88%E3%81%B3%E5%AE%9F%E8%A3%85%E3%83%BB%E5%AE%9F%E5%BF%9C%E7%94%A8%E3%81%AE%E3%83%9D%E3%82%A4%E3%83%B3%E3%83%88.pdf)に置いてあります。
  * 日本語の環境構築方法も、[本レポジトリ内](https://github.com/TakuOkawara/pose_adjustment_demo/blob/main/%E4%BD%9C%E6%A5%AD%E6%89%8B%E9%A0%86%E6%9B%B8.md)に置いてあります。

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
