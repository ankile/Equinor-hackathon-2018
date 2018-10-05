# import the necessary packages
from PIL import Image
import numpy as np
import pytesseract
import cv2
import os

src_path = "/home/morgan/PycharmProjects/hackathon_team_1/Task2/"
filename = "3.png"

img_path = src_path + filename

# load the example image and convert it to grayscale
image = cv2.imread(img_path)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Apply dilation and erosion to remove some noise
 kernel = np.ones((1, 1), np.uint8)
 gray = cv2.dilate(gray, kernel, iterations=1)
 gray = cv2.erode(gray, kernel, iterations=1)

# thresholding to preprocess the image
#gray = cv2.threshold(gray, 0, 255,cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
gray = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 2)

#median blurring  to remove noise
gray = cv2.medianBlur(gray, 3)

# write the grayscale image to disk as a temporary file so we can
# apply OCR to it
filename = "{}.png".format(os.getpid())
cv2.imwrite(filename, gray)

# load the image as a PIL/Pillow image, apply OCR, and then delete
# the temporary file
text = pytesseract.image_to_string(Image.open(filename))
os.remove(filename)
print(text)

# show the output images
cv2.imshow("Image", image)
cv2.imshow("Output", gray)
cv2.waitKey(0)
