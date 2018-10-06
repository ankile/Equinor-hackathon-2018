import keras
from keras.datasets import mnist
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten
from keras.layers import Conv2D, MaxPooling2D
from keras import backend as K
from utils import load_labeled_data
from math import floor
from scipy.misc import imresize
import numpy as np

batch_size = 64
num_classes = 10
epochs = 12

# input image dimensions
img_rows, img_cols = 28, 28

# the data, shuffled

images, labels = load_labeled_data()


images = np.array([imresize(image,(28,28)) for image in images])

images = images.reshape(images.shape[0], img_rows, img_cols, 1)
input_shape = (img_rows, img_cols, 1)

images = images.astype('float32')
images /= 255
print('images shape:', images.shape)
print(images.shape[0], 'train samples')

# convert class vectors to binary class matrices
labels = keras.utils.to_categorical(labels, num_classes)

#Reload model
model = keras.models.load_model('KerasMNIST/cnn.h5')

model.fit(images, labels,
          batch_size=batch_size,
          epochs=epochs,
          verbose=1)
model.save('cnn_mnist_techathon.h5')

#score = model.evaluate(x_test, y_test, verbose=0)
#print('Test loss:', score[0])
#print('Test accuracy:', score[1])

#print("Baseline Error: %.2f%%" % (100-scores[1]*100))

