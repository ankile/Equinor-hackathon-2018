import datetime
import glob
import cv2
import numpy as np
from scipy.misc import imresize



def load_labeled_data():
    """
    Returns a list of openCV images and a list of the corresponding labels
    """

    images = []
    labels = []

    for i in range(1, 10):
        path = ("selflabeledCropped", str(i), "*.jpg")
        filenames = glob.glob("/".join(path))
        images_one_type = [cv2.imread(img) for img in filenames]
        labels_one_type = [i] * len(images_one_type)
        images += images_one_type
        labels += labels_one_type

    return images, labels



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




def write_predictions_to_csv(images, labels):
    start = datetime.datetime.now()

    images = np.array([cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) for image in images])

    predictions = [str(np.argmax(predictions) + 1) for predictions in predict(images)]
    print(predictions)

    total = len(predictions)
    correct = 0
    accuracy = 0.0

    for i, prediction in enumerate(predictions):
        if int(prediction) == labels[i]:
            correct += 1
        accuracy = float(correct) / total
    print(correct, total, accuracy)
    elapsed = (datetime.datetime.now()-start).total_seconds()
    print("Elapsed time: ", elapsed)

    with open('predictions2.csv', 'w') as f:
        f.write("predictions ")
        f.write(' '.join(predictions) + ' ')



if __name__ == '__main__':
    images, labels = load_labeled_data()
    write_predictions_to_csv(images, labels)
