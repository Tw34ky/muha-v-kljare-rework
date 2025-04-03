import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback
import numpy as np

rospy.init_node('cv')
bridge = CvBridge()

@long_callback
def image_callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    a, b = np.array_split(img, 2)
    colorized = cv2.applyColorMap(a, cv2.COLORMAP_JET)
    
    image_pub.publish(bridge.cv2_to_imgmsg(colorized, 'bgr8'))
image_sub = rospy.Subscriber('/thermal_camera/image_raw', Image, image_callback)
image_pub = rospy.Publisher('~debug', Image)
rospy.spin()