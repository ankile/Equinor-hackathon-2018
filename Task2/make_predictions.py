import glob
import cv2
import numpy as np

from KerasMNIST.cnnPredict import predict


def load_unlabeled_data():
    filenames = glob.glob("unlabeledCropped/*.jpg")
    filenames = sorted(filenames, key=lambda x: int(x.split('/')[1].split('.')[0]))
    return [cv2.imread(img) for img in filenames]


def write_predictions_to_csv(images):
    result = []
    for i, image in enumerate(images):
        bw_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        prediction = np.argmax(predict(bw_image))
        result.append(str(prediction))

        if i % 10 == 0:
            print('Image: ', i)
            with open('predictions2.csv', 'a') as f:
                f.write(' '.join(result) + ' ')
            result = []
            
    return True


if __name__ == '__main__':
    images = load_unlabeled_data()
    write_predictions_to_csv(images)
