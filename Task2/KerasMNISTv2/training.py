import keras
from keras.datasets import mnist
from keras.models import Sequential
from keras.layers import Dense, Dropout, Flatten
from keras.layers import Conv2D, MaxPooling2D
from keras import backend as K
from utils.py import load_labeled_data


images, labels = load_labeled_data()
datasize=len(images)
(x_train, y_train) = (images[:size//2],labels[size//2])
(x_test,y_test) = (images)
#Reload model
model = load_model('partly_trained.h5')

#Continue training
dataset2_x = X_train[3000:]
dataset2_y = y_train[3000:]
model.fit(dataset2_x, dataset2_y, nb_epoch=10, batch_size=200, verbose=2)
scores = model.evaluate(X_test, y_test, verbose=0)
print("Baseline Error: %.2f%%" % (100-scores[1]*100))