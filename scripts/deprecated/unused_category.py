class Category:
    def __init__(self, names, obj_files, urdf_files, scale, bounding_boxes):
        self.names = names
        self.obj_files = obj_files
        self.urdf_files = urdf_files
        self.len = len(urdf_files)
        self.scale = scale
        self.bounding_boxes = bounding_boxes
