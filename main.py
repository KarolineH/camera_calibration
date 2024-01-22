import cv2
import apriltag
import os
import numpy as np
import yaml
from colmap_wrapper.llff.poses.pose_utils import gen_poses, load_data, load_camera_poses
from colmap_wrapper.llff.poses.colmap_read_model import read_cameras_binary


class CameraCalibration():

    # shared between all instances of the class


    def __init__(self, cam_id, base_dir=None):
        self.cam_id = cam_id
        self.resolution = None
        self.base_dir = base_dir
        self.matrix = None
        self.distortion = None 
        pass

    def init_calibrate(self):
        # Estimate intrinsics and extrinsics for all cameras
        # write to file if filename is provided
        # in real-world physical units
        # Initial calibration can be slow and precise. 
        pass

    def update_calibration(self, cameras, old_extrinsics, delta):
        # Update extrinsics for all cameras after a controlled external movement
        # make sure to convert to the correct frame
        pass

    def undistort_cv(self, img, camera_matrix, distortion_coefficients):
        # need height and width
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
        # undistort
        dst = cv.undistort(img, mtx, dist, None, newcameramtx)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]

        # OTHER METHOD
        # undistort
        mapx, mapy = cv.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h), 5)
        dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]

        #colmap also has an undistorted implemented
        # ROS has a package for this as well, works directly on the incoming camera stream
        # but can undistortion be done offline maybe?
        return dst
    
    def reconstruct_3d(self):
        # Reconstruct the 3D scene from the 2D images

        image_file_names = []
        poses = []

        out_dir = '/path/sparse/model'
        open(out_dir+'/cameras.txt', 'a')
        open(out_dir+'/images.txt', 'a')
        open(out_dir+'/points3D.txt', 'a')

        # write a placeholder for the images.txt file
        # Image list with two lines of data per image:
        #   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
        #   POINTS2D[] as (X, Y, POINT3D_ID)
        # every second line to be left blank
        # more details here https://colmap.github.io/format.html#output-format
        # and here https://colmap.github.io/faq.html#reconstruct-sparse-dense-model-from-known-camera-poses

    def colmap_calibration(self):
        """
        Calibrate the camera using a provided series of calibration images.
        Will use the images in base_dir/images/ for calibration.
        Currently assumes sequential image series and the OpenCV camera model.
        Does not take any known camera poses, so the reference frame of the reconstructed model is NOT anchored to the real world frame.
        """
        # First run COLMAP to get all camera parameters
        # Options:
            # OPENCV model yields fx, fy, cx, cy, k1, k2, p1, p2
            # SIMPLE_RADIAL yields f, cx, cy, k
            # exhaustive_matcher or sequential_matcher
            # If a sparse reconstruction model already exist in the folder, it does not run again. But you can force it to run again by setting force_rerun=True.
        gen_poses(basedir=self.base_dir, match_type='sequential_matcher', model_type='OPENCV', force_rerun=False)

        # Retrieve the camera parameters from the colmap output
        camerasfile = os.path.join(self.base_dir, 'sparse/0/cameras.bin')
        camdata = read_cameras_binary(camerasfile)
        list_of_keys = list(camdata.keys())
        cam = camdata[list_of_keys[0]]
        resolution = np.array([cam.width, cam.height]) # width, height
        mtx = np.array([[cam.params[0],0,cam.params[2]],[0,cam.params[1],cam.params[3]],[0,0,1]])
        dist = np.array(cam.params[4:])

        # Retrieve the camera poses from the colmap output
        cam_in_world = load_camera_poses(self.base_dir) #c2w: camera to world transformation matrices for each frame
        # these assume a frame only specified as [r, -u, t], which we'll assume is x-right, y-down, z-through the image plane] (bc colmap uses right handed frames)

        # poses,_ = load_data(self.base_dir, load_imgs=False)
        # from the regular COLMAP output orientation, this is rotated so that the targeted frame is: x-down, y-right, and x-backwards/towards the camera.
        # Each poses[:,:-1,0] is a 3x4 homogeneous transformation matrix, with the last row left out (because it is always [0,0,0,1])
        # w2c_mats and c2w_mats are avaliable also in pose_utils

        return resolution, mtx, dist, cam_in_world
    
    def april_tag_cam_position(self, in_dir):
        """
        Determine the position of the camera in the world frame based on a pre-made pattern of AprilTags and previous calibration.
        :param in_dir: The directory containing the images used for calibration.
        :return: estimated rotation vectors and translation vectors for all provided images
        """
        return

    def april_tag_calibration(self, obj_in_world=np.eye(4)):
        """
        Calibrate the camera using a pre-made pattern of AprilTags visible in a series of calibration images.
        Assumes that all images were taken with the same camera, at the same resolution and focal length, and have not been rotated.
        :return: dict of intrinics (incl. RMS re-projection error, camera matrix, distortion coefficients), estimated rotation vectors and translation vectors for all provided images
        """
        # cv2 (opencv) uses a right-handed coordinate system with positive axes going x-right, y-down, z-forward (away from camera through the image plane)
        # We use the same convention throughout this function.

        # The pose of the pattern in the world frame (obj_in_world) must be known. It is assumed to be the identity matrix by default. 
        # So unless specified, the origin of the pattern is also the origin of the world frame.

        # create the detector
        '''APRILTAG pattern specifications'''
        # These parameters are specific to the printed calibration pattern used.
        # The IDs of the tags are laid out in a grid pattern, so that the exact location can always be determined.
        tag_family = "tag36h11" # All tags in the pattern are of the same tag family to distinguish them from other tags in the environment.
        tag_spacing = 0.01  # in meters
        tag_size = 0.03 # in meters
        tag_layout = np.array([4,6]) # rows x columns
        options = apriltag.DetectorOptions(families=tag_family)
        detector = apriltag.Detector(options)
        # recreate the calibration pattern in 'world coordinates' (opencv), this means in real world units but given in the pattern's local frame
        # starting at the upper left corner of the pattern (origin) going first left to right in x direction, then down in y direction, z is always 0
        spacing = tag_spacing + tag_size
        corner_array = np.array([[i * spacing, j * spacing, 0] for j in range(tag_layout[0]) for i in range(tag_layout[1])], dtype=np.float32)

        obj_points = []
        img_points = []
        image_directory = self.base_dir + "images/"

        # Detect tags in all provided images
        for image in os.listdir(image_directory):
            img = cv2.imread(image_directory + image) # row, column
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # detect all tags in the image
            # if only part of the pattern is visible, we still use all available tags
            results = detector.detect(gray)
            detected_ids = np.asarray([r.tag_id for r in results])
            image_coords = np.asarray([r.corners[0,:] for r in results], dtype=np.float32)
            world_coords = corner_array[detected_ids]

            obj_points.append(world_coords)
            img_points.append(image_coords)
        resolution = gray.shape[::-1] # width, height 

        # Calibrate the camera based on all detected tags in all provided images
        # For this initial calibration no cmaera matrix or distortion coefficients are provided
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None) # arguments are object points, image points, image size, camera matrix, distortion coefficients
        print("RMS re-projection error: ", ret)

        # Printing the pattern origin frame onto the images to check
        # plot = False
        # if plot:     
            # img = cv2.imread(image_directory + os.listdir(image_directory)[-4]) # row, column
            # im2 = cv2.drawFrameAxes(img, mtx, dist, rvecs[-4], tvecs[-4], 0.1, 6)
            # # rescale the image to fit on screen
            # scale_percent = 15 # percent of original size
            # width = int(im2.shape[1] * scale_percent / 100)
            # height = int(im2.shape[0] * scale_percent / 100)
            # dim = (width, height)
            # im2 = cv2.resize(im2, dim, interpolation = cv2.INTER_AREA)

            # cv2.imshow("Image", im2)
            # cv2.waitKey(0)

        homogeneous_transforms = []
        # the rotation and translation vectors bring the pattern from object frame to camera frame, that's the same as the pose of the pattern origin given in the camera frame
        for rvec, tvec in zip(rvecs, tvecs):
            # Convert rotation vector to rotation matrix using Rodrigues formula
            rotation_matrix, _ = cv2.Rodrigues(rvec)

            homogeneous_transform = np.eye(4)
            homogeneous_transform[:3, :3] = rotation_matrix
            homogeneous_transform[:3, 3] = tvec.flatten()
            homogeneous_transforms.append(homogeneous_transform)

        obj_in_cam = np.array(homogeneous_transforms) # shape: (n_images, 4, 4), transformation matrices describing object location in the camera frame
        cam_in_obj = np.linalg.inv(obj_in_cam) # camera poses relative to the pattern origin
        cam_in_world = np.matmul(cam_in_obj,obj_in_world) # camera poses relative to the world frame
        # NEED TO KNOW THE POSITION OF THE PATTERN IN THE WORLD FRAME

        return resolution, mtx, dist, cam_in_world # camera matrix, distortion coefficients, 4x4 homogeneous transforms of the camera poses in world frame
    
    def write_to_file(self, file_path, cam_ids, matrices, distortion_params):
        """
        Write the calibration parameters to a file.
        :param file_path: The path to the output file.
        :param cam_ids: A list of camera IDs (strings or integers).
        :param matrices: A list of camera matrices (3x3 numpy arrays).
        :param distortion_params: A list of distortion parameters (usually 5x1 numpy arrays).
        :return: The data that was written to the file, a dictionary with the camera IDs as keys and the camera parameters as values.
        """

        # TODO: Also write timestamp??

        data = {}
        for ID, mat, dist in zip(cam_ids, matrices, distortion_params):
            cam_parameters = {
                "camera_matrix": mat.tolist(),
                "distortion_parameters": dist.tolist()
            }
            data[str(ID)] = cam_parameters

        with open(file_path, "w") as f:
            yaml.dump(data, f)
        return data 

    def read_from_file(self, file_path):
        """
        Read intrinsics for all calibrated cameras from file
        :param file_path: The path to the input file.
        :return: A dictionary of dictionarties, with camera IDs as top-level keys.
        """
        with open(file_path, "r") as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
        return data

if __name__ == "__main__":

    cc = CameraCalibration(cam_id=0, base_dir="/home/karo/rosws/src/camera_calibration/")
    cc.april_tag_calibration()
    # cc = CameraCalibration(cam_id=0, base_dir="/home/karo/Desktop/calibration_test/")
    # cc.colmap_calibration()
    
    # cc.write_to_file("test.yaml", np.array([0,1]), [np.array([[1,2,3],[4,5,6],[7,8,9]]), np.array([[1,2,3],[4,5,6],[7,8,9]])], [np.array([1,2,3,4,5]), np.array([1,2,3,4,5])])
    # out = cc.read_from_file("test.yaml")
    # print(out)
    #cc.april_tag_calibration()
    cc.colmap_calibration()
    #cc.colmap_calibration(in_dir = "/home/kh790/rosws/src/camera_calibration/scenedir/")


    #files = os.listdir(im_dir)

    # cv2.imshow("Image", gray)
    # cv2.waitKey(0)
    # cv2.imwrite(im_dir + 'test.jpg', img)