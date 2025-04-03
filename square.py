import rospy
import math
from clover import srv
from std_srvs.srv import Trigger
import requests as rq
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback

rospy.init_node('flight')
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out_main = cv2.VideoWriter('main_camera.avi', fourcc, 30.0, (320,240))
out_thermal = cv2.VideoWriter('thermal.avi', fourcc, 25.0, (320,240))
bridge = CvBridge()

@long_callback
def image_callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image 
    out_main.write(img)                 #blur imag

@long_callback
def thermal_callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image 
    out_thermal.write(img)   

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

def cords() -> dict:
    data = rq.get("http://157.180.22.113:8080/coords_1.json", headers={"Authorization": f"Bearer a16043eb-ca30-4f42-9b02-da727ded44a4"}).json()
    return data['nodes']
# def heat_pipe() -> dict:
#     data = rq.get()
def angle(x1, y1, x2, y2):
    return math.atan((y2 - y1)/(x2 - x1))

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

coords = cords()
coords = ((coords[0]["x"], coords[0]["y"]), (coords[1]["x"], coords[1]["y"]))
navigate_wait(z=1, frame_id='body', auto_arm=True)

navigate_wait(x=6, y=0.6, z=1.0, frame_id='aruco_map')

navigate_wait(x=3.95, y=0.4, z=1.0, frame_id='aruco_map')
image_sub = rospy.Subscriber('/main_camera/image_raw', Image, image_callback)
thermal_sub = rospy.Subscriber('/cv/debug', Image, thermal_callback)
rospy.sleep(1)
navigate_wait(x=6.45, y=3.4, z=1.0, frame_id='aruco_map')
out_main.release()
out_thermal.release()
navigate_wait(x=coords[0][0], y=coords[0][1], z=1.0, frame_id='aruco_map')
navigate_wait(x=coords[1][0], y=coords[1][0], z=1.0, frame_id='aruco_map')

rospy.sleep(5)

rospy.spin()
