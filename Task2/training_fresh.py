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

test_im = images[2601:]
test_la=labels[2601:]

images = images[:2600]
labels = labels[:2600]



images = np.array([imresize(image,(28,28)) for image in images])
test_im = np.array([imresize(image,(28,28)) for image in test_im])

images = images.reshape(images.shape[0], img_rows, img_cols, 1)
test_im = test_im.reshape(test_im.shape[0], img_rows, img_cols, 1)

input_shape = (img_rows, img_cols, 1)

images = images.astype('float32')
images /= 255

test_im = test_im.astype('float32')
test_im /= 255
print('images shape:', images.shape)
print(images.shape[0], 'train samples')
print(test_im.shape[0], 'test samples')

# convert class vectors to binary class matrices
labels = keras.utils.to_categorical(labels, num_classes)
test_la=keras.utils.to_categorical(test_la, num_classes)

model = Sequential()
model.add(Conv2D(32, kernel_size=(3, 3),
                 activation='relu',
                 input_shape=input_shape))
model.add(Conv2D(64, (3, 3), activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))
model.add(Dropout(0.25))
model.add(Flatten())
model.add(Dense(128, activation='relu'))
model.add(Dropout(0.5))
model.add(Dense(num_classes, activation='softmax'))

model.compile(loss=keras.losses.categorical_crossentropy,
              optimizer=keras.optimizers.Adadelta(),
              metrics=['accuracy'])

model.fit(images, labels,
          batch_size=batch_size,
          epochs=epochs,
          verbose=1,validation_data=(test_im, test_la))


#score = model.evaluate(test_im, test_la, verbose=1)
#print('Test loss:', score[0])
#print('Test accuracy:', score[1])

model.save('techathon_fresh.h5')

#print("Baseline Error: %.2f%%" % (100-scores[1]*100))

