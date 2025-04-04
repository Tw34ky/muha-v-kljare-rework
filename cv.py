import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback
import math
import numpy as np
# x = subp3
# 
# rocess.Popen(["rosrun", "image_view", "video_recorder", "image:=/main_camera/image_raw"], shell=False)
pipes = []
rospy.init_node('cv')
bridge = CvBridge()
def angle(x1, y1, x2, y2):
    return math.atan((y2 - y1)/(x2 - x1))
@long_callback
def image_callback(data):
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    img = img[30:210, 80:240]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)     
    # hsv[:,:,2] = 255
    # light_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR )
    gray = cv2.inRange(hsv, (23, 74, 112), (41, 166, 251))
    edges = cv2.Canny(gray,50,150,apertureSize = 3)
    # cv2.imshow("edges", edges)
    lines = cv2.HoughLines(edges,1,np.pi/180,45)

    if lines is not None:
        lines = list(map(lambda ln: [ln[0][0], np.arctan2(np.sin(ln[0][1]), np.cos(ln[0][1]))], lines))
        lines.sort()
        print(lines)
        for i in lines:
            if math.pi - i[1] < 0.26:
                pipe = i
                rho,theta = pipe
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                k = b / a
                bp = y0 - k * x0
                break
        else:
            image_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
            return
        for line in lines:
            if pipe[1] - line[1] > 0.3:
                rho,theta = line
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a*rho
                y0 = b*rho
                kx = b / a
                bpx = y0 - k * x0
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                interx = (bpx - bp) / (k - kx)
                intery = interx * kx + bpx
                img = cv2.circle(img, (int(interx), int(intery)), 10, (0, 255, 255), -1) 


                cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
    image_pub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
image_sub = rospy.Subscriber('/main_camera/image_raw', Image, image_callback)
image_pub = rospy.Publisher('~debug', Image)

# cap = cv2.VideoCapture("/home/clover/Desktop/xd.avi")
# flag, img = cap.read()
# while flag:
#     # image_callback(img)
#     # rospy.sleep(0.03)
#     flag, img = cap.read()
#     img = img[30:210, 80:240]
#     hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)     
#     # hsv[:,:,2] = 255
#     # light_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR )
#     gray = cv2.inRange(hsv, (23, 74, 112), (41, 166, 251))
#     edges = cv2.Canny(gray,50,150,apertureSize = 3)
#     cv2.imshow("edges", edges)
#     lines = cv2.HoughLines(edges,1,np.pi/180,45)
#     print(list(map(lambda ln: np.arctan2(np.sin(ln[0][1]), np.cos(ln[0][1])), lines)))
#     if lines is not None:
#         lines = list(sorted(lines, key=lambda ln: np.arctan2(np.sin(ln[0][1]), np.cos(ln[0][1]))))
        
#         lines = (lines[0], lines)
#         for line in lines:
#             rho,theta = line[0]
#             a = np.cos(theta)
#             b = np.sin(theta)
#             x0 = a*rho
#             y0 = b*rho
#             x1 = int(x0 + 1000*(-b))
#             y1 = int(y0 + 1000*(a))
#             x2 = int(x0 - 1000*(-b))
#             y2 = int(y0 - 1000*(a))

#             cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
#     # cv2.imshow("penis", img)
    # cv2.waitKey(0)
rospy.spin()
