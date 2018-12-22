import os
import sys

picture_path = '/home/blindfind/Documents/Blindfind3/tools/imageCollector/build/all'

all_images_path = os.listdir(picture_path)

def mySlice(whole_str):
    return int(whole_str[-2:])

def imageNameSlicer(image_str):
    return int(image_str.split('_')[-1][:-4])

sorted_all_images_path = sorted(all_images_path, key=mySlice)


img_id = 0

for each_path in sorted_all_images_path:
    left_total_dir_path = os.path.join(picture_path, each_path, 'left')
    right_total_dir_path = os.path.join(picture_path, each_path, 'right')
    left_images = os.listdir(left_total_dir_path)
    right_images=os.listdir(right_total_dir_path)
    sorted_left_images = sorted(left_images, key=imageNameSlicer)
    sorted_right_images = sorted(right_images, key=imageNameSlicer)
    for k in sorted_left_images:
        print k
        left_image_path = os.path.join(left_total_dir_path, k)
        right_image_path = os.path.join(right_total_dir_path, sorted_right_images(sorted_left_images.index(k)))
    for k in sorted_right_images:
        pass



