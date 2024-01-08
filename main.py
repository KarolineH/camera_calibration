import cv2
import apriltag
import os
import numpy as np


class CameraCalibration():
    def __init__(self) -> None:
        pass

    def init_calibrate(self, cameras, filename=None):
        # Estimate intrinsics and extrinsics for all cameras
        # write to file if filename is provided
        # in real-world physical units
        pass

    def read_calibration(self, cameras, filename):
        # Read intrinsics and extrinsics for all cameras from file
        # adjust extrinsics to match current camera positions
        # quick fine-tuning for example after a break in the experiment
        pass

    def update_calibration(self, cameras, old_extrinsics, delta):
        # Update extrinsics for all cameras after a controlled external movement
        # make sure to convert to the correct frame
        pass

    def colmap_calibrate(self, in_dir):
        pass

    def april_tag_calibration(self, in_dir):
        """
        Calibrate a single camera using AprilTags.
        :param in_dir: The directory containing the images used for calibration.
        :param cam_id: The unique camera ID, e.g. a serial number.
        :return: dict of intrinics (incl. RMS re-projection error, camera matrix, distortion coefficients), estimated rotation vectors and translation vectors for all provided images
        """
        # These parameters are specific to the printed calibration pattern used.
        tag_family = "tag36h11" # All tags in the pattern are of the same tag family to distinguish them from other tags in the environment.
        tag_spacing = 0.01  # in meters
        tag_size = 0.03 # in meters
        tag_layout = np.array([4,6]) # rows x columns
        spacing = tag_spacing + tag_size

        # create the detector
        options = apriltag.DetectorOptions(families=tag_family)
        detector = apriltag.Detector(options)

        # recreate the calibration pattern in 'world' coordinates
        corner_array = np.array([[i * spacing, j * spacing, 0] for j in range(tag_layout[0]) for i in range(tag_layout[1])], dtype=np.float32)

        obj_points = []
        img_points = []

        for image in os.listdir(in_dir):
            img = cv2.imread(in_dir + image)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # detect all tags in the image
            # if only part of the pattern is visible, we still use all available tags
            results = detector.detect(gray)
            detected_ids = np.asarray([r.tag_id for r in results])
            image_coords = np.asarray([r.corners[0,:] for r in results], dtype=np.float32)
            world_coords = corner_array[detected_ids]

            obj_points.append(world_coords)
            img_points.append(image_coords)

        # Calibrate the camera based on all detected tags in all provided images
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
        return ret, mtx, dist, rvecs, tvecs #RMS re-projection error, camera matrix, distortion coefficients, rotation vectors, translation vectors 
    
    def write_to_file(file_path, cam_id, ret, mtx, dist, rvecs, tvecs):
        cam_parameters = {
            "camera_id": cam_id,
            "reprojection_error": ret,
            "cam_matrix": mtx.tolist(),
            "distortion_coeffs": dist.tolist(),
            "last_rotation": rvecs[-1].tolist(),
            "last_translation": tvecs[-1].tolist()
        }

        cams[cam_id] = cam_parameters
        
        with open(file_path, "w") as f:
            json.dump(cams, f, indent=4)

        return

if __name__ == "__main__":

    cc = CameraCalibration()
    rp_error, intrinsic_matrix, distortion_coeff, rvecs, tvecs = cc.april_tag_calibration(in_dir = "/auto/homes/kh790/rosws/src/camera_calibration/images/")


    files = os.listdir(im_dir)

    # cv2.imshow("Image", gray)
    # cv2.waitKey(0)
    # cv2.imwrite(im_dir + 'test.jpg', img)