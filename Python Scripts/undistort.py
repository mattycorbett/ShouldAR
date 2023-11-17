import cv2
import numpy as np
import sys

# You should replace these 3 lines with the output in calibration step
DIM=(1280, 720)
K=np.array([[423.42148167631404, 0.0, 636.5623275064047], [0.0, 424.45713087028645, 373.22044867018457], [0.0, 0.0, 1.0]])
D=np.array([[0.115157114962095], [4.334499524854154], [-25.857323064325005], [83.76511073289078]])
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