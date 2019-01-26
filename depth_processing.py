#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point

#np.set_printoptions(threshold=np.inf)

class depth_processing():

	def __init__(self):
		global pub1,pub2, rate
    		rospy.init_node('zed_depth', anonymous=True)
    		self.bridge_object = CvBridge()
    		rospy.Subscriber("/kinect2/sd/image_depth",Image ,self.depth_callback)
		pub1 = rospy.Publisher('error_x' , Float32, queue_size = 10)
		pub2 = rospy.Publisher('error_y' , Float32, queue_size = 10)
		#rospy.Subscriber("/kinect2/hd/image_color",Image ,self.depth_callback)
		rate = rospy.Rate(100)
    		#rospy.Subscriber("/zed/left/image_rect_color",Image,self.camera_left_callback)
    		#rospy.Subscriber("/zed/right/image_rect_color",Image,self.camera_right_callback)

	def depth_callback(self,data):
		now = rospy.get_rostime()
		global pub1,pub2,rate
		try:
        # We select bgr8 because its the OpneCV encoding by default
        		cv_image = self.bridge_object.imgmsg_to_cv2(data ,desired_encoding = '16UC1')
			#cv_image = self.bridge_object.imgmsg_to_cv2(data ,desired_encoding = 'bgr8')

			depth_array = np.array(cv_image, dtype=np.float32)
			depth_array[np.isnan(depth_array)] = 0
			cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
			depth_array2 = depth_array*255
			#target = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

			#print(depth_array.shape)
			blur = cv2.GaussianBlur(depth_array2 , (9,9) , 0)
			ret, thresh = cv2.threshold(blur, 3, 255, cv2.THRESH_BINARY_INV)		
			#ret, thresh = cv2.threshold(blur, 1, 255, cv2.THRESH_BINARY_INV)
			kernel = np.ones((9,9), np.uint8)
			img_erosion = cv2.erode(thresh, kernel, iterations=1)
			img_dilation = cv2.dilate(img_erosion, kernel, iterations=1)	 
			
			cvuint8 = cv2.convertScaleAbs(img_dilation)
			#cvuint8 = cv2.bitwise_not(cvuint8)
			frame_centre = [212,256]
			image, contours, hier = cv2.findContours(cvuint8, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
			# Find the index of the largest contour
			areas = [cv2.contourArea(c) for c in contours]
			max_index = np.argmax(areas)
			cnt=contours[max_index]
			#cv2.drawContours(cvuint8, contours, -1, (0,255,0), 3)

	                if cv2.contourArea(cnt)>5000:
				#print cv2.contourArea(cnt)
		
    				x, y, w, h = cv2.boundingRect(cnt)
    				# draw a rectangle to visualize the bounding rect
				cv2.rectangle(cvuint8, (x, y), (x+w, y+h), (255, 255, 0), 2)
				centre= (((x+x+w)/2),((y+y+h)/2))
				
				k=float(((x+x+w)/2)-212)
				l=float(((y+y+h)/2)-256)

				#er_x = "%d" % k
				#er_y = "%d" % l 
				
				#rospy.loginfo(er_x)
				#rospy.loginfo(er_y)
				
				pub1.publish(k)
				pub2.publish(l)
				
				
				print centre
				px = depth_array[k,l]
				#print(px)
				print('********************************')
				

			end = rospy.get_rostime()
			#print(end.nsecs-now.nsecs)
			#print('********')
			
			#cv2.imshow("depth",cvuint8)
    			#cv2.waitKey(1)

			rate.sleep()
				
		except CvBridgeError as e:
			print(e)
    		


if __name__ == '__main__':
		try:
			detector = depth_processing()
			rospy.spin()

		except rospy.ROSInterruptException:
			rospy.loginfo("Detector node terminated.")
			cv2.destroyAllWindows()
