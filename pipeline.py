import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback
import numpy as np

rospy.init_node('heat_camera')
bridge = CvBridge()
width = 256 #Sensor width
height = 192

@long_callback
def image_callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    imdata, thdata = np.array_split(img, 2)
    colorized = cv2.applyColorMap(imdata, cv2.COLORMAP_JET)
    matrix = np.ndarray((height, width, 3), dtype=np.dtype("uint8"))
    image_pub.publish(bridge.cv2_to_imgmsg(colorized, 'bgr8'))
    
    for i in range(height):
        for j in range(width):
            matrix[i][j] = [int(((thdata[i][j][0] + thdata[i][j][1] * 256) / 64 - 273.15))]
    image_pub_m.publish(bridge.cv2_to_imgmsg(matrix, 'bgr8'))
image_sub = rospy.Subscriber('/thermal_camera/image_raw', Image, image_callback)
image_pub = rospy.Publisher('~heatmap', Image)
image_pub_m = rospy.Publisher('~matrix', Image)
rospy.spin()
