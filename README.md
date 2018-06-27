# 18b: Simulate Poses
Reads an input file of poses and simulates these poses in DART

## Dependencies
- DART (at least version 6) [Dart Homepage](https://dartsim.github.io)

## Build and Run
1: Enter the cloned/downloaded project directory

2: Build the project

    mkdir build
    cd build
    cmake ..
    make

3: Run the project

    ./sim_poses

## Usage
Keybindings:

- `Space`: Play/Pause simulation
- `p` : Print the current pose in DART format
        (aa1, aa2, aa3, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
         qLArm0, ..., qLArm6, qRArm0, ..., qRArm6)
