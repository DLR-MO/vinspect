# Vinspect

Vinspect (short for "visualized inspection") provides a multiple functionalities that support an inspection process.

The inspection process can be performed by a human (with a tracked sensor), by a robot or by a combination of both.
It also is possible to use multiple sensortypes together. 
These can produce sparse measurements (e.g. a ultrasonic sensor) or dense measurements (e.g. a camera).

All measurements can be shown together with a reference object (e.g. based on the CAD model of the part that is inspected).
Doing so will allow you to see the live coverage of this parts, both by the single measurements and a reconstructed 3D mesh based on camera images.
This allows an easier inspection process for the operator and ensures that all areas where inspected.

This package provides the core software of Vinspect and can be integrated into various applications. 
We also provide a [ROS2 interface](https://github.com/DLR-MO/vinspect_ros2) which we recommend for easy integration into ROS2 based systems.

## Features
- Visualization of large amount of measurements
- Clear coverage visualization
- Saving/loading of the inspection data
- Fast visualization and data access times due to usage of octrees
- Simple to integrate C++ library
- Python bindings

### Planned Features
- Showing original camera images by selecting viewpoint in relation to the 3D model
- Recording of the robot's pose during recording of a data point
  
## Requirements for Usage
- known pose of the sensor (e.g. by using a tracking system or forward kinematics of the robot)
- for coverage visualization, a reference mesh (e.g. from the CAD model) is helpfull

## Installation and Usage

We recommend using Vinspect together with the ROS2 interface that we provide (see https://github.com/DLR-MO/vinspect_ros2 for installation and usage instructions).
If you prefer to use it without the ROS2 interface, you can include it into your project like any other C++ library.
Additionally, there are Python bindings which allow you integration into Python code.

Beware that Vinspect depends on the pose_tree library which is included as a git submodule and you need to pull this accordingly(`git submodule init && git submodule update`).
You will need to have Open3D installed on your system ([Open3D install instructions](https://www.open3d.org/docs/release/compilation.html)). 
You also need to have RocksDB and Eigen3 installed (on Ubuntu you can run `sudo apt install librocksdb-dev libeigen3-dev`).

You can intantiate an object of the `Inspection` class and specify what types of sensors you want to use in the constructor.
Afterward, you can add data, visualize it and access it (see following sections).

### Adding Data
As explained above, the inspection can hold sparse data (single sensor measurement) and dense data (e.g. RGB-D images).
Each type is handled seperately.
You can call the `addSparseMeasurement()` function to add a new single sensor measurement to the inspection.
Similarly, you can call the `integrateImage()` function to add a new camera image.

### Visualizing
You can visualize the current state of the inspection in two different ways.
You can get a 3D mesh reconstruction based on the integrated RGB-D images by calling the `extractDenseReconstruction()` function.
The sparse data is visualized by creating a mesh of points (actually octagons) that merge multiple sensor measurements that are closest together.
This mesh can be optained by calling the `createMesh()` method of the `SparseMesh` class.

### Getting Data
The `Inspection` class offers multiple methods to directly access the data.
These are based on the underlying octree datastructure and are, therefore, fast.
This includes getting the
- closest sensor measurement (`getClosestSparseMeasurement()`)
- measurements in a specific area (`getSparseMeasurementsInRadius()`) 
  
### Saving and Loading Data
The `Inspection` class allows to continously save the inspection data to prevent any data loss.
This can be activated by providing a save path in the constructor.
The data can be loaded later by calling the `load()` function.
It is then also possible to continue the inspection and appending new data to the loaded file.

# Contributing
Contributions are welcome (issues as well as pull requests).
We plan on further developing and using this software in the future and are happy for any 
contributions from the community.
We follow the [ROS2 code style](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).

# Thanks
Thanks to [@attcs] for implementing the [Octree library](https://github.com/attcs/Octree/) which creatly facilitated the creation of this software.

Thanks to [@marrts] whose [industrial_reconstruction ROS2 package](https://github.com/ros-industrial/industrial_reconstruction) inspired the TSDF based 3D reconstruction part of Vinspect.

Thanks to [@marip8] whose [REACH software](https://github.com/ros-industrial/reach)  inspired the clear seperation between core and ROS2 code.
