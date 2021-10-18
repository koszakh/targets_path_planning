import cv2
import numpy as np

middle_point_pose = None
middle_point_orientation = None
# m_0_rvec = None
# m_0_tvec = None
# m_1_rvec = None
# m_1_tvec = None
def detect_show_markers(img, gray, aruco_dict, parameters, camera_matrix, dist_coeffs, i=6, j=5):
    """
    :param img: coloured image without distortion
    :param gray: grayscale image without distortion
    :param aruco_dict: markers data base to detect
    :param parameters: aruco detector parameters
    :param camera_matrix: matrix with camera parameters
    :param dist_coeffs: distortion coefficients  (from calibrating)
    :param i: 6, Id of aruco - reference system
    :param j: 5, Id of target aruco
    :return: show image
    """
    MARKER_SIZE = 0.15

    detected_1, detected_2 = False, False
    distance_1, distance_2 = None, None
    font = cv2.FONT_HERSHEY_SIMPLEX
    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    img = cv2.aruco.drawDetectedMarkers(img, corners, ids)
    if ids is not None:
        for k in range(len(ids)):
            rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(corners[k], MARKER_SIZE, camera_matrix, dist_coeffs)
            if ids[k] == i:
                img = cv2.aruco.drawAxis(img, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
                m_0_rvec = rvec
                m_0_tvec = tvec
                distance_1 = tvec[0][0][2]
                detected_1 = True
            elif ids[k] == j:
                img = cv2.aruco.drawAxis(img, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
                m_1_rvec = rvec
                m_1_tvec = tvec
                distance_2 = tvec[0][0][2]
                detected_2 = True
            if detected_1 and detected_2:
                global middle_point_pose
                global middle_point_orientation
                middle_point_pose = np.array([(m_0_tvec[0][0][0] + m_1_tvec[0][0][0]) / 2,
                                              (m_0_tvec[0][0][1] + m_1_tvec[0][0][1]) / 2,
                                              (m_0_tvec[0][0][2] + m_1_tvec[0][0][2]) / 2])
                middle_point_pose = middle_point_pose.reshape((1, 1, 3))
                middle_point_orientation = np.array([(m_0_rvec[0][0][0] + m_1_rvec[0][0][0]) / 2,
                                                     (m_0_rvec[0][0][1] + m_1_rvec[0][0][1]) / 2,
                                                     (m_0_rvec[0][0][2] + m_1_rvec[0][0][2]) / 2])
                middle_point_orientation = middle_point_orientation.reshape((1, 1, 3))
                img = cv2.aruco.drawAxis(img, camera_matrix, dist_coeffs,
                                         middle_point_orientation, middle_point_pose, 0.05)
                cv2.putText(img, 'distance_to_platform: %.4fm' % (middle_point_pose[0][0][2]), (0, 32), font, 1,
                            (0, 255, 0), 2, cv2.LINE_AA)
                #cv2.putText(img, '%.4fm' % (middle_point_pose[0][0][0]), (0, 124), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                #cv2.putText(img, '%.4fm' % (middle_point_pose[0][0][1]), (0, 144), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

    if distance_1 is not None:
        cv2.putText(img, 'Id' + str(i) + ' %.4fm' % distance_1, (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
    if distance_2 is not None:
        cv2.putText(img, 'Id' + str(j) + ' %.4fm' % distance_2, (0, 104), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
    '''
    if cv2.waitKey(5) == 115:
        print('saved')
        cv2.imwrite('test.png', img)
    '''

    return middle_point_pose, middle_point_orientation, distance_1, distance_2


def undistort_image(img, camera_matrix, dist_coeffs):
    """
    :param img: image
    :param camera_matrix: matrix with camera parameters
    :param dist_coeffs: distortion coefficients  (from calibrating)
    :return: image without distortion
    """
    h, w = img.shape[:2]
    new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))

    ''' Undistort '''
    dst = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_mtx)

    ''' Crop the image '''
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    return dst
