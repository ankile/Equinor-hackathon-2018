import glob
import math
import random as rand

import cv2
import numpy as np


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


def process_image(img):
    y_dim = len(img)
    x_dim = len(img[0])

    cv2.imshow('-1', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    img = cv2.imread('wiki.jpg', 0)
    img = cv2.equalizeHist(img)

    cv2.imshow('-1', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    cutoff = 25
    xs, ys = [], []
    while not len(xs) or not len(ys):
        for y, row in enumerate(img):
            for x, pixel in enumerate(row):
                if sum(pixel) < cutoff and pixel[0] == pixel[1] == pixel[2]:
                    xs.append(x)
                    ys.append(y)
        cutoff += 10

    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)

    img = img[max(min_y - 5, 0):min(max_y + 5, y_dim), max(min_x - 5, 0):min(max_x + 5, x_dim)]


    cv2.imshow('1', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Filtering and masking
    # Make image have one color channel
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = np.pad(img, ((5, 5), (5, 5)), 'constant', constant_values=255)

    cv2.imshow('2', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


    cutoff = 30
    _, img = cv2.threshold(img, cutoff, 255, cv2.THRESH_BINARY)

    cv2.imshow('3', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


    # Find contours
    _, contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    contours = sorted(contours, key=lambda z: cv2.boundingRect(z)[2] * cv2.boundingRect(z)[3])

    try:
        cnt = contours.pop()
        if cv2.boundingRect(cnt)[2] * cv2.boundingRect(cnt)[3] == len(img) * len(img[0]):
            cnt = contours.pop()
    except IndexError:
        return rand.randint(1, 9)

    x, y, w, h = cv2.boundingRect(cnt)

    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

    cv2.imshow('rect', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    min_x, max_x = x, x + w
    min_y, max_y = y, y + h

    img = img[min_y:max_y, min_x:max_x]

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

    img = np.pad(img, ((pad_y_l, pad_y_r), (pad_x_l, pad_x_r)), 'constant', constant_values=255)

    return img


def load_unlabeled_data():
    filenames = glob.glob("selflabeled/7/*.jpg")
    filenames = sorted(filenames, key=lambda x: int(x.split('/')[1].split('.')[0]))
    return [cv2.imread(img) for img in filenames]


if __name__ == '__main__':
    # images, labels = load_labeled_data()
    images = load_unlabeled_data()
    for i, image in enumerate(images[:3]):
        # if i % 10 == 0:
        #     print('Image:', i, 'Label:', labels[i])
        processed_image = process_image(image)
        cv2.imshow('processed', processed_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        if processed_image is not None:
            # cv2.imwrite('selflabeledCropped/{}/{}.jpg'.format(labels[i], i), processed_image)
            # cv2.imwrite('selflabeledCropped/7/' + str(i) + '.jpg', processed_image)
            pass
