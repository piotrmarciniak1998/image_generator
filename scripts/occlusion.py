
from PIL import Image, ImageChops
import os
import cv2
number_of_picture = 20
abs_path = os.path.abspath("../../images")
print(abs_path)
def diffrence_calculating(img1, img2):
    diff = 0
    for i in range(img1.shape[0]):
        for j in range(img1.shape[1]):
            if img1[i][j] != img2[i][j]:
                diff += 1
    return diff

for i in range(number_of_picture):
    # empty scene
    img_em = cv2.imread(f'{abs_path}/{i}_depth_em.png', 0)
    # unobstructed target
    img_unt = cv2.imread(f'{abs_path}/{i}_depth_un.png', 0)
    # obstructed target
    img_obt = cv2.imread(f'{abs_path}/{i}_depth_ob.png', 0)
    # obstructor
    img_ob = cv2.imread(f'{abs_path}/{i}_depth_owob.png', 0)

    # the number of pixels of the unobstructed target
    tar_obj_pix = diffrence_calculating(img_unt, img_em)
    # the number of pixels of covered part of target
    obs_tar_pix = diffrence_calculating(img_obt, img_ob)

    occlusion = round((tar_obj_pix-obs_tar_pix)/tar_obj_pix *100, 1)
    # Add value of occlusion to file name
    os.rename(f'{abs_path}/{i}_depth_ob.png', f'{abs_path}/{i}_depth_ob_{occlusion}%.png')

cv2.waitKey(0)
cv2.destroyAllWindows()

