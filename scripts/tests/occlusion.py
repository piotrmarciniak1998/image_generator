from os import listdir, path
from os.path import isfile, join
abs_path = path.abspath("../images")
import shutil

onlyfiles = [f for f in listdir(abs_path) if isfile(join(abs_path, f))]
for file in onlyfiles:
    occ = int(file.rstrip(".png").split('_')[3])
    if occ > 20 and occ < 80:
        print(file)
        shutil.move(f"{abs_path}/{file}", f"{abs_path}/train/{file}")