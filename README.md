# image_generator
Project for generating images used as a data set. Work in progress.

# Installation through bash
```
cd ~/catkin_ws/src
git clone https://github.com/piotrmarciniak1998/image_generator/
cd ~/catkin_ws
source devel/setup.bash
catkin_make
```
# Usage
Setting up the scene in Gazebo:
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch image_generator image_generator.launch
```
Objects can be spawned on scene using spawn.py script in a seperate tab:
```
cd ~/catkin_ws
source devel/setup.bash
rosrun image_generator spawn.py
```
# obj to urdf conversion
obj files can be converted to urdf using convert_obj_to_urdf.py script, which should be run as a normal py script (not through rosrun).
