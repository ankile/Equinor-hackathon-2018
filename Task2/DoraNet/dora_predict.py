from scipy.misc import imread, imresize, imshow
import numpy as np



def predict(images):
    x = np.invert(images)
    x = np.array([imresize(image, (28, 28)) for image in x])

    # convert to a 4D tensor to feed into our model
    x = np.array( [image.reshape(28, 28, 1) for image in x])

    x =np.array(x.astype('float32'))
    x /= 255

    # perform the prediction
    from keras.models import load_model
    model = load_model('DoraNet/doranet.h5')
    return model.predict(x, batch_size=32)

