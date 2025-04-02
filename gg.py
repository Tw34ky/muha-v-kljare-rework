import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
import subprocess
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_altitude = rospy.ServiceProxy('set_altitude', srv.SetAltitude)
set_yaw = rospy.ServiceProxy('set_yaw', srv.SetYaw)
set_yaw_rate = rospy.ServiceProxy('set_yaw_rate', srv.SetYawRate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

bridge = CvBridge()

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

@long_callback
def image_callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    colorized = cv2.applyColorMap(img, cv2.COLORMAP_JET)

    image_pub.publish(bridge.cv2_to_imgmsg(colorized, 'bgr8'))

image_pub = rospy.Publisher('~debug', Image)
rospy.spin()

navigate_wait(z=1, frame_id='body', auto_arm=True)

navigate_wait(x=6, y=0.5, z=1, frame_id='aruco_map', auto_arm=True)
navigate_wait(x=6, y=0.5, z=1, frame_id='aruco_map', auto_arm=True)
image_sub = rospy.Subscriber('/thermal_camera/image_raw', Image, image_callback)
subprocess.Popen['rosrun', 'image_view', 'video_recorder', 'image:=/main_camera/image_raw']
subprocess.Popen['rosrun', 'image_view', 'video_recorder', 'image:=/cv/debug']
navigate_wait(x=3.95, y=0.4, z=1, frame_id='aruco_map', auto_arm=True)
image_sub.unregister()
navigate_wait(x=6.45, y=3.4, z=1, frame_id='aruco_map', auto_arm=True)
navigate_wait(x=6, y=0.5, z=1, frame_id='aruco_map', auto_arm=True)
navigate_wait(x=6, y=0.5, z=0.5, frame_id='aruco_map', auto_arm=True)
land()
rospy.sleep(3)
rospy.signal_shutdown('end')
rospy.spin()