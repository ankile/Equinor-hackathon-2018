import os
import cv2

from process_images import process_image
from test_predictions import write_predictions_to_csv

# images = load_unlabeled_data()

def dora_main(images):
    for i, image in enumerate(images):
        images[i] = process_image(image)
    write_predictions_to_csv(images)


