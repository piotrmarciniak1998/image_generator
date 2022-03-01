import xml.etree.ElementTree as ET
import os
import pickle
from category import Category


# This script converts any obj files inside ../resources/obj_files/ to urdf files in ../resources/urdf_files, based on
# sample urdf file (../resources/sample_urdf.urdf). The script should be run as a normal py script, not through rosrun.


def min_max(new_val, min_val, max_val):
    if min_val <= new_val <= max_val:
        return min_val, max_val
    elif new_val < min_val:
        return new_val, max_val
    else:
        return min_val, new_val


def bounding_box(path, scale):
    x_min, y_min, z_min = float("inf"), float("inf"), float("inf")
    x_max, y_max, z_max = float("-inf"), float("-inf"), float("-inf")
    with open(path) as file:
        for line in file:
            if line[0:2] == "v ":
                data = line.rstrip()
                data = data.lstrip("v ")
                data = data.split(" ")
                x, y, z = float(data[0]), float(data[1]), float(data[2])
                x_min, x_max = min_max(x, x_min, x_max)
                y_min, y_max = min_max(y, y_min, y_max)
                z_min, z_max = min_max(z, z_min, z_max)
    return (x_min * scale, x_max * scale, y_min * scale, y_max * scale, z_min * scale, z_max * scale)


input_path = os.path.abspath("../resources/obj_files")
output_path = os.path.abspath("../resources/urdf_files")
pickle_path = os.path.abspath("../resources/pickle")

os.makedirs(output_path, exist_ok=True)
os.makedirs(pickle_path, exist_ok=True)

tree = ET.parse("../resources/sample_urdf.urdf")
root = tree.getroot()

objects = {}
category_list = [i for i in os.listdir(input_path) if os.path.isdir(os.path.join(input_path, i))]
for category in category_list:
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

    names = []
    bounding_boxes = []
    for index, obj_file in enumerate(obj_files):
        mesh_filename = obj_file

        name = category + str(index)
        names.append(name)
        bounding_boxes.append(bounding_box(mesh_filename, scale))
        robot_name = name + ".urdf"

        link_name = "baseLink"
        mesh_scale = f"{scale} {scale} {scale}"

        """
        mass_value = f"{scale * 10}"
        material_name = "red"
        color_rgba = "1 0.4 0.4 1"
        """

        for robot in root.iter("robot"):
            robot.attrib["name"] = robot_name
        for link in root.iter("link"):
            link.attrib["name"] = link_name
        for mesh in root.iter("mesh"):
            mesh.attrib["filename"] = mesh_filename
            mesh.attrib["scale"] = mesh_scale
        """
        for mass in root.iter("mass"):
            mass.attrib["value"] = mass_value
        for material in root.iter("material"):
            material.attrib["name"] = material_name
        for color in root.iter("color"):
            color.attrib["rgba"] = color_rgba
        """
        urdf_filepath = output_path + "/" + category + "/" + robot_name
        tree.write(urdf_filepath)

    urdf_files = [os.path.join(root, name)
                  for root, dirs, files in os.walk(output_path + "/" + category + "/")
                  for name in files
                  if name.endswith(".urdf")]  # Find all urdf files

    print(bounding_boxes)
    objects[category] = Category(names, obj_files, urdf_files, scale, bounding_boxes)

pickle.dump(objects, open(os.path.abspath("../resources/pickle/categories.pickle"), "wb"))
