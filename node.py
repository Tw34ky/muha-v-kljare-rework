from aruco_pose.msg import MarkerArray
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State
import rospy
import tf2_ros
from clover import srv
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool


# def dummy_function(): pass
# rospy.core.register_signals = dummy_function

class CloverAPI:
    get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
    land = rospy.ServiceProxy('land', Trigger)
    arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    navigate = rospy.ServiceProxy('navigate', srv.Navigate)
    def __new__(cls):
        if not hasattr(cls, 'instance'):
            cls.instance = super(CloverAPI, cls).__new__(cls)
            rospy.init_node("remote_panel_clover", disable_signals=True)
            print("registered")
        return cls.instance

    @staticmethod
    def get_map():
        return rospy.wait_for_message('/aruco_map/map', MarkerArray)
    
    @staticmethod
    def get_battery_state():
        return rospy.wait_for_message('/mavros/battery', BatteryState)

    @staticmethod
    def get_state():
        return rospy.wait_for_message('/mavros/state', State)