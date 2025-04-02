import rospy
import cv2
import tf2_ros
import time
import threading
import math
from mavros_msgs.srv import CommandBool
from sensor_msgs.msg import Range
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from clover import long_callback, srv
import numpy as np
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import String
import tf2_geometry_msgs

normis_cnt = []
cord_list = []
count = 0
flag = False
isActive = False
is_pursuit_active = False
check = False

color_dict = {
    "red": [(0, 100, 215), (10, 255, 255)],
    "blue": [(110, 35, 210), (140, 255, 255)],
    "green": [(50, 165, 230), (75, 255, 255)],
    "yellow": [(20, 85, 160), (50, 255, 255)]
    }

rospy.init_node('cv', disable_signals=True) # disable signals to allow interrupting with ctrl+c

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
land = rospy.ServiceProxy('land', Trigger)

bridge = CvBridge()

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

mask_pub = rospy.Publisher('~mask', Image, queue_size=1)
point_pub = rospy.Publisher('~red_circle', PointStamped, queue_size=1)

arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
# read camera info
camera_info = rospy.wait_for_message('main_camera/camera_info', CameraInfo)
camera_matrix = np.float64(camera_info.K).reshape(3, 3)
distortion = np.float64(camera_info.D).flatten()

checked_cords = []
cords = """
"""
def check_cord(data):
    points_by_color = {}
    for line in data.strip().split('\n'):
        color, x, y = line.split()
        x, y = float(x), float(y)
        if color not in points_by_color:
            points_by_color[color] = []
        points_by_color[color].append([x, y])

    distance_threshold = 1.25

    def euclidean_distance(point1, point2):
        return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


    def cluster_and_compute_centers(points, distance_threshold):
        clusters = []
        for point in points:
            added_to_cluster = False
            for cluster in clusters:
                if any(euclidean_distance(point, other_point) <= distance_threshold for other_point in cluster):
                    cluster.append(point)
                    added_to_cluster = True
                    break
            if not added_to_cluster:
                clusters.append([point])

    
        centers = []
        for cluster in clusters:
            cluster_array = np.array(cluster)
            center = cluster_array.mean(axis=0)
            centers.append(center)

        return centers


    all_centers = {}
    for color, points in points_by_color.items():
        centers = cluster_and_compute_centers(points, distance_threshold)
        all_centers[color] = centers

    for color, centers in all_centers.items():
        print(f"color: {color}")
        for i, center in enumerate(centers):
            x, y = center
            print(f"  object {i + 1}: mid in ({x:.3f}, {y:.3f})")
            checked_cords.append((color, round(x, 3), round(y, 3)))

def range_callback(msg):                     #disarm if range < 10 sm 
    if msg.range < 0.1:
        arming(False)
        time.sleep(10)
        print('disarm')
        rospy.signal_shutdown('closed')

def navigate_wait(x=0, y=0, z=0, yaw=0, speed=1, frame_id='', auto_arm=False, tolerance=0.2):

    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def img_xy_to_point(xy, dist):
    xy = cv2.undistortPoints(xy, camera_matrix, distortion, P=camera_matrix)[0][0]

    # Shift points to center
    xy -= camera_info.width // 2, camera_info.height // 2

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]

    return Point(x=xy[0] * dist / fx, y=xy[1] * dist / fy, z=dist)


def cnt_center(cnt_norm, altitude, data):    #calculate centre color point    
    M = cv2.moments(cnt_norm)
    cx = int(M['m10']/M['m00']) + 20
    cy = int(M['m01']/M['m00']) + 20
    xy3d = img_xy_to_point((cx, cy), altitude)
    target = PointStamped(header=data.header, point=xy3d)
    setpoint = tf_buffer.transform(target, 'aruco_map', timeout=rospy.Duration(0.2))
    return setpoint, cx-20, cy-20

follow_red_circle = False

@long_callback
def image_callback(data):
    global cords
    img = bridge.imgmsg_to_cv2(data, 'bgr8')     
    ksize = (4, 4)  
    crop_image = img[30:210, 40:280]                        #cropping image                          
    img_blur = crop_image    #blur image
    hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV )
    hsv[:,:,2] = 255
    light_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR )       #lightning image
    light_img_copy = light_img.copy()
    image_pub.publish(bridge.cv2_to_imgmsg(light_img_copy, 'bgr8'))
    for color in color_dict:
        thresh = cv2.inRange(hsv, color_dict[color][0], color_dict[color][1])   

        altitude = get_telemetry('terrain').z

        contours, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)       #find contours

        normis_cnt = list(filter(lambda x: cv2.contourArea(x) > 700, contours))         #filter contours

        if normis_cnt == []:
            pass
            #    rospy.loginfo_throttle(1, 'none contours')
        else:
            for cnt_norm in normis_cnt:
                setpoint, px, py = cnt_center(cnt_norm, altitude, data)
                print(color, setpoint.point.x, setpoint.point.y)
                cords += f"""{color} {setpoint.point.x} {setpoint.point.y} \n"""

@long_callback
def image_callback_check(data):
    global color_check
    global check
    global cords
    img = bridge.imgmsg_to_cv2(data, 'bgr8')     
    ksize = (4, 4)  
    crop_image = img[30:210, 50:270]                       #cropping image                          
    img_blur = crop_image    #blur image
    hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV )
    hsv[:,:,2] = 255
    light_img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR )       #lightning image
    light_img_copy = light_img.copy()
    image_pub.publish(bridge.cv2_to_imgmsg(light_img_copy, 'bgr8'))
    
    thresh = cv2.inRange(hsv, color_dict[color_check][0], color_dict[color_check][1])   

    altitude = get_telemetry('terrain').z

    contours, hierarchy = cv2.findContours(thresh.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)       #find contours

    normis_cnt = list(filter(lambda x: cv2.contourArea(x) > 700, contours))         #filter contours

    if normis_cnt == []:
        check = False
        pass
    else:
        for cnt_norm in normis_cnt:
                setpoint, px, py = cnt_center(cnt_norm, altitude, data)
                cords += f"""{color_check} {setpoint.point.x} {setpoint.point.y} \n"""
        check = True

            
pub = rospy.Publisher('/buildings', String, queue_size=1) 
image_pub = rospy.Publisher('~debug', Image, queue_size=1)
print('takeoff')
navigate_wait(z=1, frame_id='body', auto_arm=True)
print('aruco')
navigate_wait(x=0, y=0, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()
navigate_wait(x=0, y=2, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()
navigate_wait(x=0, y=4, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()
navigate_wait(x=0, y=6, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()
navigate_wait(x=0, y=8, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()

navigate_wait(x=2, y=8, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()
navigate_wait(x=2, y=6, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()
navigate_wait(x=2, y=4, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()
navigate_wait(x=2, y=2, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()
navigate_wait(x=2, y=0, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()

navigate_wait(x=4, y=0, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()
navigate_wait(x=4, y=2, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()
navigate_wait(x=4, y=4, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()
navigate_wait(x=4, y=6, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()
navigate_wait(x=4, y=8, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()

navigate_wait(x=6, y=8, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()
navigate_wait(x=6, y=6, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()
navigate_wait(x=6, y=4, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()
navigate_wait(x=6, y=2, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()
navigate_wait(x=6, y=0, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()

navigate_wait(x=8, y=0, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()
navigate_wait(x=8, y=2, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()
navigate_wait(x=8, y=4, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()
navigate_wait(x=8, y=6, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()
navigate_wait(x=8, y=8, z=2, frame_id='aruco_map')
time.sleep(0.1)
image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback)
time.sleep(1)
image_sub.unregister()

print(cords)
check_cord(cords)
print(checked_cords)
cords = """
"""
for i in checked_cords:
    navigate_wait(x=i[1], y=i[2], z=2, frame_id='aruco_map')
    print(i)
    time.sleep(0.3)
    color_check = i[0]
    image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback_check)
    time.sleep(5)
    image_sub.unregister()
    if not check:
        checked_cords.remove(i)
    

navigate_wait(x=0, y=0, z=1.5, frame_id='aruco_map')
range_sub = rospy.Subscriber('rangefinder/range', Range, range_callback)
navigate_wait(z=1, frame_id='aruco_map')
navigate_wait(z=0.5, frame_id='body')
navigate_wait(z=0, frame_id='body')
land()

checked_cords = []
check_cord(cords)
print(checked_cords)


with open("cord.txt", "w+") as file:
    file.write(f"{checked_cords}")

   
while not rospy.is_shutdown():
        f = open('cord.txt', 'r')
        message = f"{f.read()}"
        f.close()        
        pub.publish(message)

rospy.spin()
