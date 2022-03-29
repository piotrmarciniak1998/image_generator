from os.path import isfile, join
import shutil
import os


abs_path = os.path.abspath("../images")
if not os.path.exists(f"{abs_path}/train"):
    os.mkdir(f"{abs_path}/train")

files = [f for f in os.listdir(abs_path) if isfile(join(abs_path, f))]
for file in files:
    occ = int(file.rstrip(".png").split('_')[3])
    if 20 <= occ <= 80:
        print(file)
        shutil.move(f"{abs_path}/{file}", f"{abs_path}/train/{file}")
