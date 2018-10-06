import glob

import cv2
import keras
import numpy as np
# from utils import load_labeled_data
from scipy.misc import imresize


def load_labeled_data(path):
    """
    Returns a list of openCV images and a list of the corresponding labels
    """

    images = []
    labels = []

    for i in range(1, 10):
        path = (path, str(i), "*.jpg")
        filenames = glob.glob("/".join(path))
        images_one_type = [np.invert(cv2.imread(img)) for img in filenames]
        labels_one_type = [i] * len(images_one_type)
        images += images_one_type
        labels += labels_one_type

    return shuffle(images, labels)

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



batch_size = 64
num_classes = 9
epochs = 12

# input image dimensions
img_rows, img_cols = 28, 28

# the data, shuffled

images, labels = load_labeled_data("cropped")
im_test, la_test = load_labeled_data("selflabeledcropped")

labels = [labels[i]-1 for i in range(len(labels))]
la_test=[la_test[i]-1 for i in range(len(la_test))]


images = np.array([imresize(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY), (28, 28)) for image in images])
im_test = np.array([imresize(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY), (28, 28)) for image in im_test])

images = images.reshape(images.shape[0], img_rows, img_cols, 1)
im_test = images.reshape(im_test.shape[0], img_rows, img_cols, 1)
input_shape = (img_rows, img_cols, 1)

images = images.astype('float32')
im_test = im_test.astype('float32')

images /= 255
im_test /= 255
print('images shape:', images.shape)
print(images.shape[0], 'train samples')
print(im_test.shape[0], 'test samples')

# convert class vectors to binary class matrices
labels = keras.utils.to_categorical(labels, num_classes)
la_test = keras.utils.to_categorical(la_test, num_classes)

# Reload model
model = keras.models.load_model('DoraNet/mnist_without_zero.h5')

model.fit(images, labels,
          batch_size=batch_size,
          epochs=epochs,
          verbose=1, validation_data=(im_test, la_test))
model.save('DoraNet/doranet.h5')

score = model.evaluate(im_test, la_test, verbose=0)
print('Test loss:', score[0])
print('Test accuracy:', score[1])

#print("Baseline Error: %.2f%%" % (100-scores[1]*100))
