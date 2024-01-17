import cv2
import apriltag
import os
import numpy as np
from colmap_wrapper.llff.poses.pose_utils import gen_poses, load_data
from colmap_wrapper.llff.poses.colmap_read_model import read_cameras_binary


class CameraCalibration():
    def __init__(self) -> None:
        pass



    def init_calibrate(self, cameras, filename=None):
        # Estimate intrinsics and extrinsics for all cameras
        # write to file if filename is provided
        # in real-world physical units
        # Initial calibration can be slow and precise. 
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

    def colmap_calibration(self, in_dir):
        """
        Calibrate a single camera using multiple images taken of the same scene.
        :param in_dir: The base directory containing a subfolder 'images'.
        :return: ...
        """

        # The input directory should contain a folder 'images' with all images used for calibration.
        # OPENCV model yields fx, fy, cx, cy, k1, k2, p1, p2
        # SIMPLE_RADIAL yields f, cx, cy, k
        gen_poses(basedir=in_dir, match_type='sequential_matcher', model_type='OPENCV') # exhaustive_matcher or sequential_matcher

        # Now retrieve the camera parameters from the colmap output
        camerasfile = os.path.join(in_dir, 'sparse/0/cameras.bin')
        camdata = read_cameras_binary(camerasfile)
        list_of_keys = list(camdata.keys())
        cam = camdata[list_of_keys[0]] 

        # Now retrieve the camera poses from the colmap output
        poses,_,_ = load_data(in_dir)
        return cam, poses
    
    def april_tag_cam_position(self, in_dir):
        """
        Determine the position of the camera in the world frame based on a pre-made pattern of AprilTags and previous calibration.
        :param in_dir: The directory containing the images used for calibration.
        :return: estimated rotation vectors and translation vectors for all provided images
        """
        return

    def april_tag_calibration(self, in_dir):
        """
        Calibrate a single camera using multiple images of a pre-made pattern of AprilTags.
        :param in_dir: The base directory for this calibration operation, containing a subfolder called 'images' holding the collection of images used for calibration.
        :return: dict of intrinics (incl. RMS re-projection error, camera matrix, distortion coefficients), estimated rotation vectors and translation vectors for all provided images
        """
        # These parameters are specific to the printed calibration pattern used.
        # The IDs of the tags are laid out in a grid pattern, so that the exact location can always be determined.
        tag_family = "tag36h11" # All tags in the pattern are of the same tag family to distinguish them from other tags in the environment.
        tag_spacing = 0.01  # in meters
        tag_size = 0.03 # in meters
        tag_layout = np.array([4,6]) # rows x columns
        spacing = tag_spacing + tag_size

        # create the detector
        options = apriltag.DetectorOptions(families=tag_family)
        detector = apriltag.Detector(options)

        # recreate the calibration pattern in 'world' coordinates
        # starting at the upper left corner of the pattern (origin) going first right in x direction, then down in y direction
        corner_array = np.array([[i * spacing, j * spacing, 0] for j in range(tag_layout[0]) for i in range(tag_layout[1])], dtype=np.float32)

        obj_points = []
        img_points = []
        im_dir = in_dir + "images/"

        for image in os.listdir(im_dir):
            img = cv2.imread(im_dir + image)
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
        # For this initial calibration no cmaera matrix or distortion coefficients are provided
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None) # arguments are object points, image points, image size, camera matrix, distortion coefficients
        # the extrinsics are the rotation and translation vectors bring the pattern from object frame to camera frame, same as the position of the pattern in the camera frame
        return ret, mtx, dist, rvecs, tvecs # RMS re-projection error, camera matrix, distortion coefficients, rotation vectors, translation vectors 
    
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
    #rp_error, intrinsic_matrix, distortion_coeff, rvecs, tvecs = cc.april_tag_calibration(in_dir = "/home/karo/rosws/src/camera_calibration/images/")

   # cc.april_tag_calibration(in_dir="/home/kh790/rosws/src/camera_calibration/tag_dir/")
    cc.colmap_calibration(in_dir="/home/kh790/Desktop/")
    #cc.colmap_calibration(in_dir = "/home/kh790/rosws/src/camera_calibration/scenedir/")


    files = os.listdir(im_dir)

    # cv2.imshow("Image", gray)
    # cv2.waitKey(0)
    # cv2.imwrite(im_dir + 'test.jpg', img)