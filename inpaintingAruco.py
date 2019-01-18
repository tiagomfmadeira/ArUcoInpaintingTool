import argparse
import cv2 as cv
import cv2.aruco
import numpy as np
import OCDatasetLoader.OCDatasetLoader as OCDatasetLoader
import matplotlib.pyplot as plt

from collections import namedtuple
from copy import deepcopy

# -------------------------------------------------------------------------------
# --- FUNCTIONS
# -------------------------------------------------------------------------------


def keyPressManager():
    print('keyPressManager.\nPress "c" to continue or "q" to abort.')
    while True:
        key = cv.waitKey(15)
        if key == ord('c'):
            print('Pressed "c". Continuing.')
            break
        elif key == ord('q'):
            print('Pressed "q". Aborting.')
            exit(0)


def drawAxis3D(ax, transform, text, axis_scale=0.1, line_width=1.0):
    pt_origin = np.array([[0, 0, 0, 1]], dtype=np.float).transpose()
    x_axis = np.array([[0, 0, 0, 1], [axis_scale, 0, 0, 1]], dtype=np.float).transpose()
    y_axis = np.array([[0, 0, 0, 1], [0, axis_scale, 0, 1]], dtype=np.float).transpose()
    z_axis = np.array([[0, 0, 0, 1], [0, 0, axis_scale, 1]], dtype=np.float).transpose()

    pt_origin = np.dot(transform, pt_origin)
    x_axis = np.dot(transform, x_axis)
    y_axis = np.dot(transform, y_axis)
    z_axis = np.dot(transform, z_axis)

    ax.plot(x_axis[0, :], x_axis[1, :], x_axis[2, :], 'r-', linewidth=line_width)
    ax.plot(y_axis[0, :], y_axis[1, :], y_axis[2, :], 'g-', linewidth=line_width)
    ax.plot(z_axis[0, :], z_axis[1, :], z_axis[2, :], 'b-', linewidth=line_width)
    ax.text(pt_origin[0, 0], pt_origin[1, 0], pt_origin[2, 0], text, color='black')  # , size=15, zorder=1


def drawMask(img, imgpts):

    imgpts = np.int32(imgpts).reshape(-1, 2)

    img = cv.drawContours(img, [imgpts[:4]], -1, (255, 255, 255), -3)

    img = cv.drawContours(img, np.hstack([[imgpts[4:6]],[imgpts[1::-1]]]), -1, (255, 255, 255), -3)

    img = cv.drawContours(img, np.hstack([[imgpts[5:7]],[imgpts[2:0:-1]]]), -1, (255, 255, 255), -3)

    img = cv.drawContours(img, np.hstack([[imgpts[6:8]],[imgpts[3:1:-1]]]), -1, (255, 255, 255), -3)

    img = cv.drawContours(img, np.hstack([[imgpts[3::-3]],[imgpts[4::3]]]), -1, (255, 255, 255), -3)

    #img = cv.drawContours(img, [imgpts[4:]], -1, (255, 255, 255), -3)

    return img



# -------------------------------------------------------------------------------
# --- MAIN
# -------------------------------------------------------------------------------
if __name__ == "__main__":

    # ---------------------------------------
    # --- Parse command line argument
    # ---------------------------------------
    ap = argparse.ArgumentParser()

    # Dataset loader arguments
    ap.add_argument("-p", "--path_to_images", help="path to the folder that contains the OC dataset", required=True)
    ap.add_argument("-ext", "--image_extension", help="extension of the image files, e.g., jpg or png", default='jpg')
    ap.add_argument("-m", "--mesh_filename", help="full filename to input obj file, i.e. the 3D model", required=True)
    ap.add_argument("-i", "--path_to_intrinsics", help="path to intrinsics yaml file", required=True)
    ap.add_argument("-ucci", "--use_color_corrected_images", help="Use previously color corrected images",
                    action='store_true', default=False)
    ap.add_argument("-si", "--skip_images", help="skip images. Useful for fast testing", type=int, default=1)
    ap.add_argument("-vri", "--view_range_image", help="visualize sparse and dense range images", action='store_true',
                    default=False)

    # InpaintingScript arguments
    # TODO:

    args = vars(ap.parse_args())
    print(args)

    # ---------------------------------------
    # --- INITIALIZATION
    # ---------------------------------------
    dataset_loader = OCDatasetLoader.Loader(args)
    dataset_cameras = dataset_loader.loadDataset()
    num_cameras = len(dataset_cameras.cameras)
    print("#########################################################################################################\n")
    print("Loaded " + str(num_cameras) + " cameras to the dataset!\n")

    # ---------------------------------------
    # --- Utility functions
    # ---------------------------------------
    def matrixToRodrigues(T):
        rods, _ = cv.Rodrigues(T[0:3, 0:3])
        rods = rods.transpose()
        return rods[0]


    def rodriguesToMatrix(r):
        rod = np.array(r, dtype=np.float)
        matrix = cv.Rodrigues(rod)
        return matrix[0]


    def traslationRodriguesToTransform(translation, rodrigues):
        R = rodriguesToMatrix(rodrigues)
        T = np.zeros((4, 4), dtype=np.float)
        T[0:3, 0:3] = R
        T[0:3, 3] = translation
        T[3, 3] = 1
        return T

    # ---------------------------------------
    # --- Detect ARUCOS
    # ---------------------------------------

    ArucoT = namedtuple('ArucoT', 'id center translation rodrigues aruco2camera camera2aruco')

    class ArucoConfiguration:
        def __init__(self):
            pass

    dataset_arucos = ArucoConfiguration()
    dataset_arucos.aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_ARUCO_ORIGINAL)
    dataset_arucos.parameters = cv.aruco.DetectorParameters_create()

    dataset_arucos.markerSize = 0.082
    dataset_arucos.distortion = np.array(dataset_cameras.cameras[0].rgb.camera_info.D)
    dataset_arucos.intrinsics = np.reshape(dataset_cameras.cameras[0].rgb.camera_info.K, (3, 3))
    dataset_arucos.world_T_aruco = {}

    # For each camera
    for i, camera in enumerate(dataset_cameras.cameras):
        camera.rgb.arucos = {}
        print("In camera " + camera.name + " there is:")

        image = cv.cvtColor(camera.rgb.image, cv.COLOR_BGR2GRAY)
        corners, ids, _ = cv.aruco.detectMarkers(image, dataset_arucos.aruco_dict,
                                                 parameters=dataset_arucos.parameters)

        if ids is None:
            print("\t\t\t No ArUco detected!")
            continue

        # Estimate pose of each marker
        rotationVecs, translationVecs, _ = cv.aruco.estimatePoseSingleMarkers(corners, dataset_arucos.markerSize,
                                                                               dataset_arucos.intrinsics,
                                                                               dataset_arucos.distortion)
        # For each ArUco marker detected in the camera
        for j, id in enumerate(ids):

            # OpenCV format hack
            id = id[0]

            print("\t\t\t Aruco " + str(id) + ";")

            # OpenCV format hack
            my_corners = corners[j][0][:]

            # separate corners into x and y coordinates
            x = []
            y = []
            for corner in my_corners:
                x.append(corner[0])
                y.append(corner[1])

            # Get tuple with center of marker
            center = (np.average(x), np.average(y))

            rodrigues = np.array(rotationVecs[j][0])

            translation = np.array(translationVecs[j][0])

            aruco2camera = traslationRodriguesToTransform(translation, rodrigues)

            camera2aruco = np.linalg.inv(aruco2camera)

            # Create and assign the ArUco object to data structure
            camera.rgb.arucos[id] = (ArucoT(id, center, translation, rodrigues, aruco2camera, camera2aruco))

            # Add only if there is still not an estimate (made by another camera)
            if id not in dataset_arucos.world_T_aruco:
                dataset_arucos.world_T_aruco[id] = np.dot(camera.rgb.matrix,
                                                          np.linalg.inv(camera.rgb.arucos[id].camera2aruco))

    # Display information over the ArUcos
    font = cv.FONT_HERSHEY_SIMPLEX
    for i, camera in enumerate(dataset_cameras.cameras):
        image = deepcopy(camera.rgb.image)
        corners, ids, _ = cv.aruco.detectMarkers(image, dataset_arucos.aruco_dict,
                                                 parameters=dataset_arucos.parameters)

        # Draw axis and write info
        for key, aruco in camera.rgb.arucos.items():
            cv.aruco.drawAxis(image, dataset_arucos.intrinsics, dataset_arucos.distortion, aruco.rodrigues,
                               aruco.translation, 0.05)
            cv.putText(image, "Id:" + str(aruco.id), aruco.center, font, 1, (0, 255, 0), 2, cv.LINE_AA)

        # Draw the outer square
        cv.aruco.drawDetectedMarkers(image, corners)

        # Show the image
        cv.namedWindow('cam' + str(i), cv.WINDOW_NORMAL)
        cv.imshow('cam' + str(i), image)

    print("Displaying aruco detections for all images.")
    keyPressManager()
    cv.destroyAllWindows()

    # ---------------------------------------
    # --- Inpainting
    # ---------------------------------------

    arucoBorder = 0.015
    arucoThickness = 0.006

    mask3DPoints = np.float32(
        [[-(dataset_arucos.markerSize / 2 + arucoBorder), dataset_arucos.markerSize / 2 + arucoBorder, 0.002],
         [dataset_arucos.markerSize / 2 + arucoBorder, dataset_arucos.markerSize / 2 + arucoBorder, 0.002],
         [dataset_arucos.markerSize / 2 + arucoBorder, -(dataset_arucos.markerSize / 2 + arucoBorder), 0.002],
         [-(dataset_arucos.markerSize / 2 + arucoBorder), -(dataset_arucos.markerSize / 2 + arucoBorder), 0.002],
         [-(dataset_arucos.markerSize / 2 + arucoBorder), dataset_arucos.markerSize / 2 + arucoBorder, -arucoThickness],
         [dataset_arucos.markerSize / 2 + arucoBorder, dataset_arucos.markerSize / 2 + arucoBorder, -arucoThickness],
         [dataset_arucos.markerSize / 2 + arucoBorder, -(dataset_arucos.markerSize / 2 + arucoBorder), -arucoThickness],
         [-(dataset_arucos.markerSize / 2 + arucoBorder), -(dataset_arucos.markerSize / 2 + arucoBorder), -arucoThickness]])

    for i, camera in enumerate(dataset_cameras.cameras):

        print("Creating a mask for camera " + camera.name + "...")

        image = deepcopy(camera.rgb.image)
        height, width, channels = image.shape
        mask = np.zeros((height, width), dtype=np.uint8)

        # For each ArUco detected in the image
        for key, aruco in camera.rgb.arucos.items():

            # project 3D points to image plane
            imgpts, jac = cv2.projectPoints(mask3DPoints, aruco.rodrigues, aruco.translation,
                                            dataset_arucos.intrinsics, dataset_arucos.distortion)

            # TODO: project aruco to world and back OR from one camera to another?
            # for each camera
            #   for all arucos detected
            #       create mask world coords: take mask from aruco to world with inv(dataset_arucos.world_T_aruco[id])
            #
            # for each camera
            #   for world2aruco in dataset_arucos.world_T_aruco
            #       apply world2aruco to mask world coords

            mask = drawMask(mask, imgpts)

            print("\t\t\t Added Aruco " + str(key) + " to the mask;")


        # Show the masks
        # cv.namedWindow('cam mask ' + str(i), cv.WINDOW_NORMAL)
        # cv.imshow('cam mask ' + str(i), mask)

        # redImg = np.zeros(image.shape, image.dtype)
        # redImg[:, :] = (0, 0, 255)
        # redMask = cv2.bitwise_and(redImg, redImg, mask=mask)

        # inpaintedImage = cv.addWeighted(redMask, 0.5, image, 0.5, 0.0)

        # Apply inpainting algorithm
        inpaintedImage = cv.inpaint(image, mask, 5, cv.INPAINT_TELEA)
        # inpaintedImage = cv.inpaint(image, mask, 5, cv.INPAINT_NS)

        # Show the final image
        cv.namedWindow('Inpainted image ' + str(i), cv.WINDOW_NORMAL)
        # cv.namedWindow('Inpainted image ' + str(i), cv.WINDOW_FULLSCREEN)
        cv.imshow('Inpainted image ' + str(i), inpaintedImage)

    keyPressManager()

    exit(0)
