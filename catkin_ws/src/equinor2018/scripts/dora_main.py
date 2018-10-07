import glob
import os
import cv2
import numpy as np

from process_images import process_image
from test_predictions import write_predictions_to_csv, predict_image, predict


# images = load_unlabeled_data()

def dora_main(images):
    for i, image in enumerate(images):
        images[i] = process_image(image)
    write_predictions_to_csv(images)


def predict(image):
    image = process_image(image)
    return predict_image(image)


filename = glob.glob("labeledData/7/385.jpg")
image = cv2.imread(filename[0])
print(predict(image))
