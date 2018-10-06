from scipy.misc import imread, imresize, imshow
import numpy as np
import argparse

# ap = argparse.ArgumentParser()
#
# ap.add_argument("-i", "--image", required=True,
#                 help="path to input image")
# ap.add_argument("-f", "--filetype", required=False, default="jpg",
#                 help="select filetype")
#
# args = vars(ap.parse_args())
#
# x = imread(args["image"] + '.' + args["filetype"], mode='L')


def predict(image):
    x = np.invert(image)
    x = imresize(x, (28, 28))

    # convert to a 4D tensor to feed into our model
    x = x.reshape(1, 28, 28, 1)
    x = x.astype('float32')
    x /= 255

    # perform the prediction
    from keras.models import load_model
    model = load_model('KerasMNIST/cnn_mnist_techathonv2.h5')
    return model.predict(x)


# out = predict(x)
# print(out)
# print(np.argmax(out))
