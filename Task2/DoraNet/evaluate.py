import glob

import cv2
import keras


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

model = keras.models.load_model('doranet.h5')

x_test, y_test = load_labeled_data()
y_test = [y_test[i]-1 for i in range(len(y_test))]


y_test = keras.utils.to_categorical(y_test, 9)


score = model.evaluate(x_test, y_test, verbose=0)
print('Test loss:', score[0])
print('Test accuracy:', score[1])