# Rowing software

Here my attemps at creating a rowing simulator.
The first, failed, attempt was done using gazebo with ROS.

The next attempts use OpenSim.

## Pusher

Contains the simplest version of a "leg" pushing a boat through the water. Well, actually the boat is a box on the ground an friction is used
to simulate the water.
Both IK-tool and CMC-tool work, the video is the CMC tool in action.

[//]: #  <img src="Pusher/pusher_snapshot.png" alt="pusher_image" width="800"/>

https://github.com/SietseAchterop/Rowing-simulator/puser_CMC_video.webm

This version does not have muscles yet.

## BootBaan

Contains a first version of a boat with a rower. No muscles yet, only actuators on the joints.

<img src="BootBaan/bootbaan_snapshot.png" alt="bootbaan_image" width="800"/>

  - Osim and trc files are created via the python scripts.
  - The mot file is created using the IK tool.
