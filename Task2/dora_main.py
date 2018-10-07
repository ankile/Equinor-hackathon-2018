import glob
import random as rand

import cv2
from .process_images import process_image
from .test_predictions import write_predictions_to_csv, predict_image


# images = load_unlabeled_data()

def dora_main(images, to_file):
    for i, image in enumerate(images):
        images[i] = process_image(image)
    write_predictions_to_csv(images, to_file)


def predict(image):
    image = process_image(image)
    if image is int:
        return image
    return predict_image(image)


if __name__ == '__main__':
    filenames = glob.glob("unlabeledData/*.jpg")
    filenames = sorted(filenames, key=lambda x: int(x.split('/')[1].split('.')[0]))
    images = [cv2.imread(img) for img in filenames]

    print "Started processing images"
    dora_main(images, 'dora_test.csv')
    print "Done and saved to file"
