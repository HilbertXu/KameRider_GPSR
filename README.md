# KameRider_GPSR
The whole workspace for Team KameRider HouseKeeper General Purpose Service Robot(GPSR) Task

## Prerequirements

1. darknet_ros package

   https://github.com/leggedrobotics/darknet_ros.git

2. Code for ROS by example part 1

   https://github.com/pirobot/rbx1.git

3. Turtlebot Arm packages

   https://github.com/turtlebot/turtlebot_arm.git

4. CMU OpenPose (Python Wrapper)

   https://github.com/CMU-Perceptual-Computing-Lab/openpose.git

   (Recommand to add build options manually when build the pybind11 to choose the python2.7, because ROS does not support Python3...)

5. PocketSphinx (Python module) & pyaudio

   sudo apt-get install libsound-dev swig python-pyaudio ros-kinetic-sound-play

   pip install pyaudio pocketsphinx

   git clone https://github.com/Pankaj-Baranwal/pocketsphinx

6. Turtlebot Description & Hardware:

   ![turtlebot_model](<https://github.com/HilbertXu/KameRider_GPSR/blob/RoboCup2019/src/Prerequirements/images/turtlebot_model.png>)

    1. kobuki base

    2. kinect/hokuyo

    3. astra

       sudo apt-get install ros-kinetic-astra-launch ros-kinetic-astra-camera

       unzip the **OpenNI-Linux-x64-2.3** package

       ```
       cd OpenNI-Linux-x64-2.3
       sudo chmod a+x install.sh
       sudo ./install.sh
       ```

       then installing the libuvc

       ```
       git clone https://github.com/ktossell/libuvc
       cd libuvc
       mkdir build
       cd build
       cmake ..
       make && sudo make install
       ```

       astra_launch/astra.launch needs to be modified

       ![modified astra launch](<https://github.com/HilbertXu/KameRider_GPSR/blob/RoboCup2019/src/Prerequirements/images/astra_launch.png>)

       replace the default value of arg "camera" with "astra" to aviod two nodelet with the same name

       

    4. turtlebot_arm

       needs to update the turtlebot urdf files

       After installing the rocon/kobuki/turtlebot packages

       *Replace* the **$(find turtlebot_description)/urdf/sensors/astra.urdf.xacro**	**$(find turtlebot_description)/robots/kobuki_hexagons_kinect.urdf.xacro** with the modified files

       *Add* the hokuyo description file to **$(find turtlebot_description)/urdf/sensors** if necessary

       *Replace* the **$(find turtlebot_arm_description)/urdf/turtlebot_arm.urdf.xacro** with the modified file

       

    5. mic & speaker

## Workspaces Arrangement

Dividing the whole task into Five parts: **Image Processing**, **Navigation**, **Speech Recognition & Human Robot Interaction(HRI)**, **Arm Manipulation** and **Task Control Part**

And I establish five packages for these five parts respectively, each package contains two ROS packages, one of them is **CORE** package, and the other is **MESSAGE** package. **CORE** package contains all nodes of its part, and the **MESSAGE** package contains the customized message files on which its part depends.

My essential idea is to seperate the **General Packages** and **Task-Driven Packages** and put them into two different ROS **workspaces** respectively. Consequently, We only need to replace the Task-Driven workspace when we switch to another task. ALL the **General Packages** like i mentioned in the Prerequirements Part will be put into a **General ROS Workspace**.