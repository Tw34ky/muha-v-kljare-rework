import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback
import math

# x = subprocess.Popen(["rosrun", "image_view", "video_recorder", "image:=/main_camera/image_raw"], shell=False)

rospy.init_node('cv')
bridge = CvBridge()
def angle(x1, y1, x2, y2):
    return 2 * math.pi - math.atan((y2 - y1)/(x2 - x1))
is_pipe = False
@long_callback
def image_callback(data):
    global is_pipe
    # img = data
    img = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image                          #blur image
    hsv = cv2.cvtColor(img[30:210, 80:240], cv2.COLOR_BGR2HSV)     
    # hsv[:,:,2] = 255
    # light_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR )
    thresh = cv2.inRange(hsv, (23, 74, 112), (41, 166, 251))   
    cleft, _ = cv2.findContours(thresh[:, :90], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cleft = list(filter(lambda x: cv2.contourArea(x) > 100, cleft))
    cright, _ = cv2.findContours(thresh[:, 90:], cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cright = list(filter(lambda x: cv2.contourArea(x) > 100, cright))
    ml, mr = [0, 0], [0, 0]
    print(is_pipe)
    if cleft:
        for cnt in cleft:
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            ml[0] += cx
            ml[1] += cy
        ml[0] //= len(cleft)
        ml[1] //= len(cleft)
        img = cv2.circle(img[30:120, 80:240], tuple(ml), radius=5, color=(255, 0, 0), thickness=5)
        cv2.imshow("xd", img)
        for cnt in cleft:
            if cv2.pointPolygonTest(cnt, tuple(ml), False) != 1:
                is_pipe = True
                return True
    if cright:
        for cnt in cright:
            M = cv2.moments(cnt)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            ml[0] += cx
            ml[1] += cy
        mr[0] //= len(cright)
        mr[1] //= len(cright)
        for cnt in cright:
            if cv2.pointPolygonTest(cnt, tuple(mr), False) != 1:
                is_pipe = True
                cv2.imwrite("pipe.png", img)
                return True
    is_pipe = False
    return False
image_sub = rospy.Subscriber('/main_camera/image_raw', Image, image_callback)
image_pub = rospy.Publisher('~debug', Image)
rospy.spin()
# cap = cv2.VideoCapture("/home/clover/Desktop/xd.avi")
# flag, img = cap.read()
# while flag:
#     # image_callback(img)
#     rospy.sleep(0.03)
#     flag, img = cap.read()
