import glob
import os

import cv2
import numpy as np


def load_labeled_data(img_path="Data", do_shuffle=False):
    """
    Returns a list of openCV images and a list of the corresponding labels
    """

    images = []
    labels = []

    for i in range(1, 10):
        path = ("cropped", str(i), "*.png")
        filenames = glob.glob("/".join(path))
        images_one_type = [cv2.imread(img, 0) for img in filenames]
        labels_one_type = [i] * len(images_one_type)
        images += images_one_type
        labels += labels_one_type

        if do_shuffle:
            images, labels = shuffle(images, labels)

    return images, labels


def shuffle(images, labels):
    result = [(images[i], labels[i]) for i in range(len(images))]
    np.random.shuffle(result)
    images = []
    labels = []
    for i in range(len(result)):
        images.append(result[i][0])
        labels.append(result[i][1])

    images = np.array(images)
    return images, labels



def load_unlabeled_data():
    filenames = os.listdir("unlabeledData")
    filenames = sorted(filenames, key=lambda x: int(os.path.splitext(x)[0]))
    return [cv2.imread(img) for img in filenames]


if __name__ == '__main__':
    images, labels = load_labeled_data()
    images = load_unlabeled_data()
    print(len(images))
