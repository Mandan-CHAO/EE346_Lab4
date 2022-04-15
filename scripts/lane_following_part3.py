#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy, time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
aruco_para = cv2.aruco.DetectorParameters_create()
distortion_para = numpy.zeros(5, dtype=float)
K = numpy.matrix([[265, 0, 160], [0, 265, 120], [0, 0, 1]])
mlen = 0.1


class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()

                self.image_sub = rospy.Subscriber('camera/image',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=10)

                self.twist = Twist()
	
		

        def image_callback(self, msg):
		
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
		(corners,ids,_) = cv2.aruco.detectMarkers(image, aruco_dict, parameters=aruco_para)
		if corners:
			(rvecs,tvecs,_) = cv2.aruco.estimatePoseSingleMarkers(corners, mlen, K, distortion_para)
			if tvecs[0][0][0]>0.09 and tvecs[0][0][0]<0.1:
				self.twist.linear.x = 0 ##set the velocity to zero to stop
                  		self.twist.angular.z = 0
                    		self.cmd_vel_pub.publish(self.twist)
				print('Stop!!!!!')
				time.sleep(10)
				print('Now gooooo!!')
				time.sleep(2)
				
			corners = None
			

                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                lower_yellow = numpy.array([ 10, 10, 10])
                upper_yellow = numpy.array([255, 255, 250])
                lower_white = numpy.array([0, 0, 80])
                upper_white = numpy.array([180, 43, 220])
                
                mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask2 = cv2.inRange(hsv, lower_white, upper_white)

                h, w, d = image.shape
                search_top = 2*h/3
                mask1[0:search_top, 0:w] = 0
                mask2[0:search_top, 0:w] = 0
		mask2[:, 0:w/2] = 0

                M1 = cv2.moments(mask1)
                M2 = cv2.moments(mask2)

                if M1['m00'] > 0:
                    cx1 = int(M1['m10']/M1['m00'])
                    cy1 = int(M1['m01']/M1['m00'])

                    cx2 = int(M2['m10']/M2['m00'])
                    cy2 = int(M2['m01']/M2['m00'])

		    fpt_x = (cx1 + cx2)/2
                    fpt_y = (cy1 + cy2)/2 + 2*h/3

                    cv2.circle(image, (cx1, cy1), 10, (0,255,255), -1)
                    cv2.circle(image, (cx2, cy2), 10, (255,255,255), -1)
                    cv2.circle(image, (fpt_x, fpt_y), 10, (128,128,128), -1)

                    err = w/2 - fpt_x

                    self.twist.linear.x = 0.3
                    self.twist.angular.z = (err*90.0/160)/15
                    self.cmd_vel_pub.publish(self.twist)
                cv2.imshow("window", image)
                cv2.waitKey(1)


rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
