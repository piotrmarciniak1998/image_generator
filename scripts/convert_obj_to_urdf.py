import xml.etree.ElementTree as ET
import os


# This script converts any obj files inside ../resources/obj_files/ to urdf files in ../resources/urdf_files, based on
# sample urdf file (../resources/sample_urdf.urdf). The script should be run as a normal py script, not through rosrun.

tree = ET.parse("../resources/sample_urdf.urdf")
root = tree.getroot()
input_path = os.path.abspath("../resources/obj_files")
output_path = os.path.abspath("../resources/urdf_files")
obj_files = [os.path.join(root, name)
             for root, dirs, files in os.walk(input_path)
             for name in files
             if name.endswith(".obj")]

for index, obj_file in enumerate(obj_files):
    robot_name = f"table{index}.urdf"
    link_name = "baseLink"
    mesh_filename = obj_file
    mesh_scale = "1.0 1.0 1.0"
    material_name = "red"
    color_rgba = "1 0.4 0.4 1"

    for robot in root.iter("robot"):
        robot.attrib["name"] = robot_name
    for link in root.iter("link"):
        link.attrib["name"] = link_name
    for mesh in root.iter("mesh"):
        mesh.attrib["filename"] = mesh_filename
        mesh.attrib["scale"] = mesh_scale
    for material in root.iter("material"):
        material.attrib["name"] = material_name
    for color in root.iter("color"):
        color.attrib["rgba"] = color_rgba
    tree.write(output_path + "/" + robot_name)
