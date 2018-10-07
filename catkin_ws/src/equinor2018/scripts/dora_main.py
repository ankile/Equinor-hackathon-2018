import random as rand

from process_images import process_image
from test_predictions import write_predictions_to_csv, predict_image


def predict(image):
    image = process_image(image)
    if image is None:
        return rand.randint(1, 9)
    return predict_image(image)

