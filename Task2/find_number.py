import cv2
import numpy as np
import glob
import math


def load_labeled_data():
    """
    Returns a list of openCV images and a list of the corresponding labels
    """

    images = []
    labels = []

    for i in range(1, 10):
        path = ("Data", str(i), "*.jpg")
        filenames = glob.glob("/".join(path))
        images_one_type = [cv2.imread(img) for img in filenames]
        labels_one_type = [i] * len(images_one_type)
        images += images_one_type
        labels += labels_one_type

    return images, labels


images, labels = load_labeled_data()

for i, image in enumerate(images):
    if i % 10 == 0:
        print('Image:', i + 1)

    cutoff = 50
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
    print(min_x, max_x, min_y, max_y)

    diff_x = max_x - min_x
    diff_y = max_y - min_y

    if diff_x > diff_y:
        min_y -= math.floor((diff_x - diff_y) / 2)
        max_y += math.ceil((diff_x - diff_y) / 2)
    elif diff_y > diff_x:
        min_x -= math.floor((diff_y - diff_x) / 2)
        max_x += math.ceil((diff_y - diff_x) / 2)

    scale_factor = 0.30

    scale_amount = int(((max_x - min_x) // 2) * scale_factor)
    min_x, min_y = max(int(min_x - scale_amount), 0), max(int(min_y - scale_amount), 0)
    max_x, max_y = max(int(max_x + scale_amount), 0), max(int(max_y + scale_amount), 0)

    cropped = image[min_y:max_y, min_x:max_x]

    gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)

    # Apply dilation and erosion to remove some noise
    kernel = np.ones((1, 1), np.uint8)
    gray = cv2.dilate(gray, kernel, iterations=1)
    gray = cv2.erode(gray, kernel, iterations=1)

    # thresholding to preprocess the image
    # gray = cv2.threshold(gray, 0, 255,cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
    gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 2)

    # median blurring  to remove noise
    gray = cv2.medianBlur(gray, 3)

    cv2.imwrite(f'cropped/{labels[i]}/{i}.png', gray)
