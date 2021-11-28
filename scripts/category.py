class Category:
    def __init__(self, names, obj_files, urdf_files):
        self.names = names
        self.obj_files = obj_files
        self.urdf_files = urdf_files
        self.len = len(urdf_files)
