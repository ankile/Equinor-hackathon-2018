import glob
import cv2
import numpy as np

from DoraNet.dora_predict import predict


def load_unlabeled_data():
    filenames = glob.glob("unlabeledCropped/*.jpg")
    return [cv2.imread(img) for img in filenames]


# for image in load_unlabeled_data():
#     prediction = predict(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
#     cv2.imshow(str(np.argmax(prediction)), image)
#     cv2.waitKey(0)


def write_predictions_to_csv(images):
    print('entered write predictions')
    result = []
    for i, image in enumerate(images):
        if i % 10 == 0:
            print('Image: ', i)
        bw_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        prediction = np.argmax(predict(bw_image))
        print('prediction:', prediction)
        result.append(prediction)

    with open('predictions.csv', 'w') as f:
        f.write(' '.join(result))

    return True

print('code print')

if __name__ == '__main__':
    print('main entered')
    images = load_unlabeled_data()
    print('images, length:', len(images))
    write_predictions_to_csv(images)
