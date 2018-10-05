import glob
import cv2
import numpy as np


def load_labeled_data():
    """
    Returns a list of openCV images and a list of the corresponding labels
    """

    images = []
    labels = []

    for i in range(1, 10):
        path = ("cropped", str(i), "*.jpg")
        filenames = glob.glob("/".join(path))
        images_one_type = [cv2.imread(img) for img in filenames]
        labels_one_type = [i] * len(images_one_type)
        images += images_one_type
        labels += labels_one_type
	images,labels = shuffle(images,labels)
    return images, labels


def shuffle(images, labels):
	paired = [(images[i],labels[i]) for i in range (len(images))]
	result = np.random.shuffle(paired)
	images = []
	labels = []
	for i in range(len(result)):̈́    
		images.append(result[i][0])
		labels.append(result[i][1])
	return images,labels

    
def load_unlabeled_data():
    filenames = glob.glob("unlabeledData/*.jpg")
    return [cv2.imread(img) for img in filenames]


if __name__ == '__main__':
    images, labels = load_labeled_data()
    images = load_unlabeled_data()
    print(len(images))
