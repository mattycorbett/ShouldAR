import cv2
import numpy as np
import sys

# You should replace these 3 lines with the output in calibration step
DIM=(1280, 720)
K=np.array([[1249.8089688067782, 0.0, 641.9237510452458], [0.0, 1240.4487176173016, 331.44420525622223], [0.0, 0.0, 1.0]])
D=np.array([[0.6983763633559256], [-7.92845301235481], [66.48311646581958], [-176.59573201741406]])
def undistort(img_path):
    img = cv2.imread(img_path)
    h,w = img.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    cv2.imshow("undistorted", undistorted_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
if __name__ == '__main__':
    for p in sys.argv[1:]:
        undistort(p)