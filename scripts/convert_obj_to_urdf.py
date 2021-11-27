import xml.etree.ElementTree as ET
import os
import pickle
from category import Category


# This script converts any obj files inside ../resources/obj_files/ to urdf files in ../resources/urdf_files, based on
# sample urdf file (../resources/sample_urdf.urdf). The script should be run as a normal py script, not through rosrun.

input_path = os.path.abspath("../resources/obj_files")
output_path = os.path.abspath("../resources/urdf_files")
os.makedirs(output_path, exist_ok=True)

tree = ET.parse("../resources/sample_urdf.urdf")
root = tree.getroot()

object_categories = {}
category_list = [i for i in os.listdir(input_path) if os.path.isdir(os.path.join(input_path, i))]
for category in category_list:
    os.makedirs(output_path + "/" + category, exist_ok=True)  # Create directories to contains converted urdfs.

    obj_files = [os.path.join(root, name)
                 for root, dirs, files in os.walk(input_path + "/" + category + "/")
                 for name in files
                 if name.endswith(".obj")]  # Find all obj files

    names = []
    for index, obj_file in enumerate(obj_files):
        mesh_filename = obj_file

        name = category + str(index)
        names.append(name)
        robot_name = name + ".urdf"

        link_name = "baseLink"  # all of attributes below are constant for now
        mesh_scale = "1.0 1.0 1.0"
        mass_value = "1.0"
        material_name = "red"
        color_rgba = "1 0.4 0.4 1"

        for robot in root.iter("robot"):
            robot.attrib["name"] = robot_name
        for link in root.iter("link"):
            link.attrib["name"] = link_name
        for mesh in root.iter("mesh"):
            mesh.attrib["filename"] = mesh_filename
            mesh.attrib["scale"] = mesh_scale
        for mass in root.iter("mass"):
            mass.attrib["value"] = mass_value
        for material in root.iter("material"):
            material.attrib["name"] = material_name
        for color in root.iter("color"):
            color.attrib["rgba"] = color_rgba
        urdf_filepath = output_path + "/" + category + "/" + robot_name
        tree.write(urdf_filepath)

    urdf_files = [os.path.join(root, name)
                  for root, dirs, files in os.walk(output_path + "/" + category + "/")
                  for name in files
                  if name.endswith(".urdf")]  # Find all urdf files

    object_categories[category] = Category(names, obj_files, urdf_files)

pickle.dump(object_categories, open("categories.pickle", "wb"))