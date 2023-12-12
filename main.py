import cv2
import apriltag
import os
import numpy as np

im_dir = "/auto/homes/kh790/rosws/src/camera_calibration/images/"
files = os.listdir(im_dir)

options = apriltag.DetectorOptions(families="tag36h11") # all calibration tags are tag36h11
detector = apriltag.Detector(options)

# example pattern of 4 rows and 6 columns, assuming 30mm tags and 10mm gaps between them, all lie in the z-plane:
corner_array = np.array([[0, 0, 0], [40, 0, 0], [80, 0, 0], [120, 0, 0], [160, 0, 0], [200, 0, 0],
                         [0, 40, 0], [40, 40, 0], [80, 40, 0], [120, 40, 0], [160, 40, 0], [200, 40, 0],
                         [0, 80, 0], [40, 80, 0], [80, 80, 0], [120, 80, 0], [160, 80, 0], [200, 80, 0],
                         [0, 120, 0], [40, 120, 0], [80, 120, 0], [120, 120, 0], [160, 120, 0], [200, 120, 0]], dtype=np.float32)

obj_points = []
img_points = []

for image in files:
    img = cv2.imread(im_dir + image)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    results = detector.detect(gray)
    detected_ids = np.asarray([r.tag_id for r in results])
    image_coords = np.asarray([r.corners[0,:] for r in results], dtype=np.float32)
    world_coords = corner_array[detected_ids]

    obj_points.append(world_coords)
    img_points.append(image_coords)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

# cv2.imshow("Image", gray)
# cv2.waitKey(0)
# cv2.imwrite(im_dir + 'test.jpg', img)