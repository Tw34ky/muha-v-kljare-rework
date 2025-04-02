import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import io

bridge = CvBridge()

rospy.init_node("heat_camera")

image_pub = rospy.Publisher('~heatmap', Image, queue_size=1)
matrix_pub = rospy.Publisher('~matrix', Image, queue_size=1)

def is_raspberrypi():
    try:
        with io.open('/sys/firmware/devicetree/base/model', 'r') as m:
            if 'raspberry pi' in m.read().lower(): return True
    except Exception: pass
    return False

isPi = is_raspberrypi()

dev = 0
	
#init video
cap = cv2.VideoCapture('/dev/video'+str(dev), cv2.CAP_V4L)
# cap = cv2.VideoCapture(1)
#pull in the video but do NOT automatically convert to RGB, else it breaks the temperature data!
#https://stackoverflow.com/questions/63108721/opencv-setting-videocap-property-to-cap-prop-convert-rgb-generates-weird-boolean
if isPi:
	cap.set(cv2.CAP_PROP_CONVERT_RGB, 0.0)
else:
	cap.set(cv2.CAP_PROP_CONVERT_RGB, False)

#256x192 General settings
width = 256 #Sensor width
height = 192 #sensor height
scale = 1 #scale multiplier
newWidth = width*scale 
newHeight = height*scale
alpha = 1.0 # Contrast control (1.0-3.0)
colormap = 0
font=cv2.FONT_HERSHEY_SIMPLEX
dispFullscreen = False
cv2.namedWindow('Thermal',cv2.WINDOW_GUI_NORMAL)
cv2.resizeWindow('Thermal', newWidth,newHeight)
rad = 0 #blur radius
threshold = 2
hud = True
recording = False
elapsed = "00:00:00"
snaptime = "None"

def rec():
	now = time.strftime("%Y%m%d--%H%M%S")
	#do NOT use mp4 here, it is flakey!
	videoOut = cv2.VideoWriter(now+'output.avi', cv2.VideoWriter_fourcc(*'XVID'),25, (newWidth,newHeight))
	return(videoOut)

def snapshot(heatmap):
	#I would put colons in here, but it Win throws a fit if you try and open them!
	now = time.strftime("%Y%m%d-%H%M%S") 
	snaptime = time.strftime("%H:%M:%S")
	cv2.imwrite("TC001"+now+".png", heatmap)
	return snaptime
 
while not rospy.is_shutdown():
	ret, frame = cap.read()
	if ret:
		matrix = np.ndarray((height, width, 3), dtype=np.dtype("uint8"))
		imdata,thdata = np.array_split(frame, 2)
		hi = thdata[96][128][0]
		lo = thdata[96][128][1]
		lo = lo*256
		rawtemp = hi+lo
		temp = (rawtemp/64)-273.15
		temp = round(temp,2)
		for i in range(height):
			for j in range(width):
				matrix[i][j] = [int(((thdata[i][j][0] + thdata[i][j][1] * 256) / 64 - 273.15))] * 3
		# grey = cv2.cvtColor(matrix, cv2.COLOR_GRAY2BGR)
		print(matrix[..., 0].max(), matrix[..., 0].min())
		

		# #find the max temperature in the frame
		# lomax = thdata[...,1].max()
		# posmax = thdata[...,1].argmax()
		# #since argmax returns a linear index, convert back to row and col
		# mcol,mrow = divmod(posmax,width)
		# himax = thdata[mcol][mrow][0]
		# lomax=lomax*256
		# maxtemp = himax+lomax
		# maxtemp = (maxtemp/64)-273.15
		# maxtemp = round(maxtemp,2)

		
		# #find the lowest temperature in the frame
		# lomin = thdata[...,1].min()
		# posmin = thdata[...,1].argmin()
		# #since argmax returns a linear index, convert back to row and col
		# lcol,lrow = divmod(posmin,width)
		# himin = thdata[lcol][lrow][0]
		# lomin=lomin*256
		# mintemp = himin+lomin
		# mintemp = (mintemp/64)-273.15
		# mintemp = round(mintemp,2)

		# #find the average temperature in the frame
		# loavg = thdata[...,1].mean()
		# hiavg = thdata[...,0].mean()
		# loavg=loavg*256
		# avgtemp = loavg+hiavg
		# avgtemp = (avgtemp/64)-273.15
		# avgtemp = round(avgtemp,2)

		
		bgr = cv2.cvtColor(imdata, cv2.COLOR_YUV2BGR_YUYV)
		bgr = cv2.convertScaleAbs(bgr, alpha=alpha)#Contrast
		bgr = cv2.resize(bgr,(newWidth,newHeight),interpolation=cv2.INTER_CUBIC)#Scale up!
		if rad>0:
			bgr = cv2.blur(bgr,(rad,rad))

		heatmap = cv2.applyColorMap(bgr, cv2.COLORMAP_JET)
		cmapText = 'Jet'
		image_pub.publish(bridge.cv2_to_imgmsg(heatmap, 'bgr8'))
		matrix_pub.publish(bridge.cv2_to_imgmsg(matrix, 'bgr8'))
