import datetime
import glob
import cv2
import numpy as np

from DoraNet.dora_predict import predict


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

    return shuffle(images, labels)


def test_predictions(images, labels):
    with open('predctions2.csv', 'w') as f:
        f.write("predictions ")
    total = 0
    correct = 0
    accuracy = 0.0
    result = []
    start = datetime.datetime.now()
    for i, image in enumerate(images):
        total += 1

        bw_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        prediction = np.argmax(predict(bw_image))

        result.append(str(prediction))

        if prediction == labels[i] - 1:
            correct += 1
            accuracy = float(correct) / total
            print("CORRECT: predicted", prediction + 1, "correctly")

        else:
            print("WRONG: predicted", prediction + 1, "but was", labels[i])

        if i % 10 == 0:
            print('Image: ', i)
            print("Correct:", correct)
            print("Total:", total)
            print("Accuracy:", accuracy)
            elapsed = datetime.datetime.now() - start

            print("Time elapsed:", elapsed)

    with open('predictions2.csv', 'a') as f:
        f.write(' '.join(result) + ' ')

    return True


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




if __name__ == '__main__':
    images, labels = load_labeled_data()
    test_predictions(images, labels)
