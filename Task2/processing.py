import cv2
import numpy as np
import pytesseract
from PIL import Image
from pytesseract import image_to_string
import argparse
import os

ap = argparse.ArgumentParser()

ap.add_argument("-i", "--image", required=True,
                help="path to input image")
ap.add_argument("-p", "--preprocess", type=str, default="gauss",
                help="type of preprocessing to be done")
args = vars(ap.parse_args())

# Path of working folder on Disk
src_path = "/home/morgan/PycharmProjects/hackathon_team_1/Task2"
img_path = src_path + "/" + args["image"] + ".jpg"

# Read image with opencv
original = cv2.imread(img_path)

# Convert to gray
img = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)

# -- order of filters below should be reconsidered --
# -- also consider whether or not a given filter should be used or not --

# Apply dilation and erosion to remove some noise
kernel = np.ones((1, 1), np.uint8)
img = cv2.dilate(img, kernel, iterations=1)
img = cv2.erode(img, kernel, iterations=1)

# Median blurring to remove noise
img = cv2.medianBlur(img, 3)

# Write image after removed noise
cv2.imwrite(src_path + "removed_noise.png", img)

#  Apply threshold to get image with only black and white

# check to see if we should apply thresholding to preprocess the
# image
if args["preprocess"] == "gauss":
    img = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 2)


# make a check to see if median blurring should be done to remove
# noise
elif args["preprocess"] == "bin":
    img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

# Write the image after applying threshold
cv2.imwrite(src_path + "thres.png", img)

# Recognize text with tesseract for python
# result = pytesseract.image_to_string(Image.open(src_path + "thres.png"))

# Remove template file
# os.remove(temp)

cv2.imshow("Image", original)
cv2.imshow("Output", img)
cv2.waitKey(0)
