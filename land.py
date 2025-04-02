import rospy
import math
from std_srvs.srv import Trigger
from clover import srv

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)

navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)


def navigate_wait(x=0, y=0, z=0, yaw=0, speed=1, frame_id='', auto_arm=False, tolerance=0.2):

    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

navigate_wait(x=0, y=0, z=1.5, frame_id='aruco_map')
navigate_wait(z=1, frame_id='aruco_map')
navigate_wait(z=0.5, frame_id='body')
navigate_wait(z=0, frame_id='body')
land()