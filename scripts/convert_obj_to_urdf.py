import xml.etree.ElementTree as ET
import os
import pickle
from item import Item


# This script converts any obj files inside ../resources/obj_files/ to urdf files in ../resources/urdf_files, based on
# sample urdf file (../resources/sample_urdf.urdf). The script should be run as a normal py script, not through rosrun.


def bounding_box(path, scale):
    x_list = []
    y_list = []
    z_list = []

    with open(path) as file:
        for line in file:
            if line[0:2] == "v ":
                data = line.rstrip()
                data = data.lstrip("v ")
                data = data.split(" ")

                x, z, y = float(data[0]), float(data[1]), float(data[2])
                x_list.append(x)
                y_list.append(y)
                z_list.append(z)

    x_min = min(x_list) * scale
    x_max = max(x_list) * scale
    y_min = min(y_list) * scale
    y_max = max(y_list) * scale
    z_min = min(z_list) * scale
    z_max = max(z_list) * scale

    return x_min, x_max, y_min, y_max, z_min, z_max


input_path = os.path.abspath("../resources/obj_files")
output_path = os.path.abspath("../resources/urdf_files")
pickle_path = os.path.abspath("../resources/pickle")

os.makedirs(output_path, exist_ok=True)
os.makedirs(pickle_path, exist_ok=True)

tree = ET.parse("../resources/sample_urdf.urdf")
root = tree.getroot()

items = {}
category_list = [i for i in os.listdir(input_path) if os.path.isdir(os.path.join(input_path, i))]

for category in category_list:
    items[category] = []

    while True:
        try:
            scale = float(input(f"What scale should {category} be? "))
        except ValueError:
            print("That is not a valid scale number!")
        else:
            break

    os.makedirs(output_path + "/" + category, exist_ok=True)  # Create directories to contains converted urdfs.

    obj_files = [os.path.join(root, name)
                 for root, dirs, files in os.walk(input_path + "/" + category + "/")
                 for name in files
                 if name.endswith(".obj")]  # Find all obj files

    for index, obj_file in enumerate(obj_files):
        name = category + str(index)
        x_min, x_max, y_min, y_max, z_min, z_max = bounding_box(obj_file, scale)

        for robot in root.iter("robot"):
            robot.attrib["name"] = f"{name}.urdf"
        for mesh in root.iter("mesh"):
            mesh.attrib["filename"] = obj_file
            mesh.attrib["scale"] = f"{scale} {scale} {scale}"

        urdf_filepath = f"{output_path}/{category}/{name}.urdf"
        tree.write(urdf_filepath)

        item = Item(name=name,
                    category=category,
                    obj_path=obj_file,
                    urdf_path=urdf_filepath,
                    scale=scale,
                    x_min=x_min,
                    x_max=x_max,
                    y_min=y_min,
                    y_max=y_max,
                    z_min=z_min,
                    z_max=z_max)

        items[category].append(item)

pickle.dump(items, open(os.path.abspath("../resources/pickle/items.pickle"), "wb"))
