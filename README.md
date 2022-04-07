# 🎉 Project `image_generator` is released!
As of 07.04.2022 the project is officially released and no further development is planned at this moment.

# 👨‍💻 Description
Project for generating images used as a data set. The main goal is to create random scenes including objects placed on 
the table. Each scene is then used to take photo of obstructed and unobstructed view on the mug (target), which are also 
labeled with percentage of obstruction. Obstructing objects are randomized.

# 💾 Installation through bash
```
cd ~/catkin_ws/src
git clone https://github.com/piotrmarciniak1998/image_generator/
cd ~/catkin_ws
source devel/setup.bash
catkin_make
```

# 🔨 Components 
`.launch` files: 
* `image_generator.launch` - main Gazebo world.
* `empty.launch` - empty Gazebo world.
* `kinect.launch` - Gazebo world with Kinect sensor.

`.py` scripts executed normally:
* `convert_obj_to_urdf` - converts any `*.obj` to `*.urdf` and creates a `.pickle` that is used for storing
data about objects in the project. **This script should be executed to ensure that the project works properly.**

`.py` main scripts working only through `rosrun`:
* `generator.py` - takes rgb and depth images of the mug placed in different table environments.

`.py` services:
* `image_saver_server.py` - saves image with specified filename and with given index

# ▶️ Usage
**Requirements**
* Ubuntu 20.04 LTS Operating System
* ROS Noetic Ninjemys

**Modifying the scene**

Additional `.obj models` can be added or exchanged in `image_generator/resources/obj_files`. After each change, 
`convert_obj_to_urdf.py` should be executed (1. below).

**Output**

From 1 scene the program produces 4 photos labeled with:
* iteration
* type (depth / rgb)
* confirmation if the current photo is obstructed or not (o - obstructed, u - unobstructed)
* percentage of obstruction

Example:

`149_depth_o_36.png` is obstructed depth image of 149th scene, on which the target is obstructed by 36%.

**How to start it up**
1. It is recommended to execute `convert_obj_to_urdf.py`:
```
cd ~/catkin_ws/src/image_generator/scripts
python3 convert_obj_to_urdf.py
```
2. Use a `roslaunch` file (main file is `kinect.launch` for now):
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch image_generator kinect.launch
```
3. Use the image saving server (necessary for `generator.py`):
```
cd ~/catkin_ws
source devel/setup.bash
rosrun image_generator image_saver_server.py
```
4. Use a `rosrun` script (main script is `generator.py`):
```
cd ~/catkin_ws
source devel/setup.bash
rosrun image_generator generator.py
```

