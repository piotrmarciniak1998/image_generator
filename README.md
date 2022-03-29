# 👨‍💻 Project: image_generator
Project for generating images used as a data set. The main goal is to create random scenes including objects placed on 
the table. Each scene is then used to take photo of obstructed and unobstructed view on the mug (target), which are also 
labeled with % of obstruction. Obstructing objects are randomized.
# 🚧 TODO
1. Improvements to code quality.
2. Improvements to randomization of positions and rotations.
3. Adding different camera angles.
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