# Usage
First, clone the PX4 repository.

git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot

Step by step:

1). For PX4-Autopilot:

1. git checkout -b dev v1.12.3
2. git submodule update --init --recursive

3. cp /path/to/your/patches/dfuav.patch .

4. git apply --check dfuav.patch

5. git apply dfuav.patch



2). For gazebo:

1. cd Tools/sitl_gazebo
2. cp /path/to/your/patches/gazebo.patch .
3. git apply --check gazebo.patch
4. git apply gazebo.patch



3). For matrix:

1. cd ../..

2. cd src/lib/matrix

3. cp /path/to/your/patches/matrix.patch .

4. git apply --check matrix.patch

5. git apply matrix.patch

6. 

* Extract or copy your packaged STL files. Assuming you have a file named compressed_meshes.tar.gz:

tar -xzf /path/to/your/compressed_meshes.tar.gz -C Tools/sitl_gazebo/

* Or directly copy the folder:

cp -r /path/to/your/meshes/ Tools/sitl_gazebo/models/



4). Build:

1. source ~/px4_build_env.sh 
2. make px4_sitl gazebo_ductedfan4      # (If it fails, first delete residual build files inside the build directory)

