import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback

rospy.init_node('cv')
bridge = CvBridge()

@long_callback
def image_callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    max_temp = img[..., 1].max()
    binary = cv2.inRange(img, (40, 40, 40), (255, 255, 255))
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)       #find contours
    res = [0, 0]
    for cnt_norm in contours:
        M = cv2.moments(cnt_norm)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        res[0] += cx
        res[1] += cy
    res[0] /= len(contours)
    res[1] /= len(contours)
    return max_temp, res

image_sub = rospy.Subscriber('heat_camera/matrix', Image, image_callback)

rospy.spin()