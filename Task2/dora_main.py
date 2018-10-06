import os
import cv2

from utils import load_unlabeled_data
from bounding_boxes process_image

images = load_unlabeled_data()
for i, image in enumerate(images):
    images[i] = process_image(image)
write_to_scv(images)






'''
load inn bilder med load unlabeled data

kjør prosessering

kjør dora_predict


'''