import cv2
import apriltag
from PIL import Image

path = "/auto/homes/kh790/rosws/src/camera_calibration/big_marker.png"
image = cv2.imread(path)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
options = apriltag.DetectorOptions(families="tag16h5")
detector = apriltag.Detector(options)
results = detector.detect(gray)
cv2.imshow("Image", gray)
cv2.waitKey()
print(results)