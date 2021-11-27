# 👨‍💻 Project: image_generator
Project for generating images used as a data set. Work in progress.
# 🚧 TODO:
1. Subscriber node to image published by kinect
2. Service to save image
3. Main script which randomizes position of camera and items on the table and then saves all the needed images.
4. Improvements to code quality
5. If possible, add functionality to display how much visible/ obstructed is the object.
# 💾 Installation through bash
```
cd ~/catkin_ws/src
git clone https://github.com/piotrmarciniak1998/image_generator/
cd ~/catkin_ws
source devel/setup.bash
catkin_make
```
# ▶️ Usage
Setting up the scene in Gazebo:  
* `image_generator.launch` - main
* `empty.launch` - empty
* `kinect.launch` - kinect sensor
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch image_generator image_generator.launch
```
All urdf objects can be spawned on scene using `spawn.py` script in a seperate tab:
```
cd ~/catkin_ws
source devel/setup.bash
rosrun image_generator spawn.py
```
# ➡️ obj to urdf conversion
obj files must be converted to urdf using `convert_obj_to_urdf.py` script, which should be run as a normal py script
(not through rosrun). The script creates `./resources/urdf_files` subdirectory and `./scripts/categories.pickle`, used 
in `spawn.py` script. 
```
cd ~/catkin_ws/src/image_generator/scripts
python3 convert_obj_to_urdf.py
```