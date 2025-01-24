import cv2
import numpy as np

imcolor = cv2.imread("nuka.png")

b, g, r = cv2.split(imcolor)

mask_green = np.uint8((g > 100) & (r < 80) & (b < 80))  

cv2.imshow("Mask Green", mask_green * 255)  

kernel = np.ones((5, 5), np.uint8)
mask_green = cv2.dilate(mask_green, kernel, iterations=2)

im_green = cv2.bitwise_and(imcolor, imcolor, mask=mask_green)

cv2.imshow("Green Object", im_green)
cv2.waitKey(0)
cv2.destroyAllWindows()

cv2.imwrite("green_object.jpg", im_green)
