
import cv2
import numpy as np

arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
tag = np.zeros((300, 300, 1), dtype="uint8")
cv2.aruco.drawMarker(arucoDict, 23, 300, tag, 1)
cv2.imwrite("marker23.png", tag)