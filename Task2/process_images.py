import cv2
import numpy as np
import glob
import math


# from utils import load_labeled_data


def load_labeled_data():
    """
    Returns a list of openCV images and a list of the corresponding labels
    """

    images = []
    labels = []

    for i in range(1, 10):
        path = ("selflabeled", str(i), "*.jpg")
        filenames = glob.glob("/".join(path))
        images_one_type = [cv2.imread(img) for img in filenames]
        labels_one_type = [i] * len(images_one_type)
        images += images_one_type
        labels += labels_one_type

    return images, labels


def load_unlabeled_data():
    filenames = glob.glob("unlabeledData/*.jpg")
    return [cv2.imread(img) for img in filenames]


def process_image(i, image, labels, save=False,):
    cutoff = 20
    xs, ys = [], []
    while not len(xs) or not len(ys):
        for y, row in enumerate(image):
            for x, pixel in enumerate(row):
                if sum(pixel) < cutoff and pixel[0] == pixel[1] == pixel[2]:
                    xs.append(x)
                    ys.append(y)
        cutoff += 10

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    cropped = image[min_y:max_y, min_x:max_x]

    diff_x = max_x - min_x
    diff_y = max_y - min_y

    scale_factor = 0.30

    pad_x_l, pad_x_r, pad_y_l, pad_y_r = [int(max(diff_x, diff_y) * scale_factor)] * 4

    if diff_x > diff_y:
        pad_y_l += int(math.floor((diff_x - diff_y) / 2))
        pad_y_r += int(math.ceil((diff_x - diff_y) / 2))
    elif diff_y > diff_x:
        pad_x_l += int(math.floor((diff_y - diff_x) / 2))
        pad_x_r += int(math.ceil((diff_y - diff_x) / 2))

    try:
        gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
    except:
        return

    _, gray = cv2.threshold(gray, cutoff, 255, cv2.THRESH_BINARY)

    padded = np.pad(gray, ((pad_y_l, pad_y_r), (pad_x_l, pad_x_r)), 'constant', constant_values=255)

    if save:
        cv2.imwrite('selflabeledCropped/{}/{}.jpg'.format(labels[i], i), padded)


images, labels = load_labeled_data()

for i, image in enumerate(images):
    # if i % 50 == 0:
    print('Image: ', i)
    process_image(i, image, labels, save=True)

