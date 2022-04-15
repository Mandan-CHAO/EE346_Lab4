#!/usr/bin/env python

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

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
		K = np.matrix([[265, 0, 160], [0, 265, 120], [0, 0, 1]])
		R = np.matrix([[1, 0, 0], [0, 0, -1], [0, -1, 0]])
		t = np.zeros([3, 1])
		t[1] = 0.8  #ty: ? height to the camera
		t[2] = 0.95  #tz: ? distance to camera along the road 
		n = np.zeros([1, 3])
		n[0, 1] = -1
		d = 0.115
		h = np.dot(K, np.dot(R - np.dot(t, n) / d, np.linalg.inv(K)))
		#K @ (R - t @ n / d) @ np.linalg.inv(K)
		#h = np.matrix([[-0.434, -1.33, 229.], [-0.0, -2.88, 462.], [-0.0, -0.00833, 1.0]])
		img_bv = cv2.warpPerspective(image, h, (image.shape[1],image.shape[0]))

                hsv = cv2.cvtColor(img_bv, cv2.COLOR_BGR2HSV)

                lower_yellow = np.array([ 26, 43, 46])
                upper_yellow = np.array([77, 255, 255])

                lower_white = np.array([0, 0, 200])
                upper_white = np.array([180, 43, 255])
                
                mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask2 = cv2.inRange(hsv, lower_white, upper_white)

                h, w, d = image.shape
                search_top = 1*h/100
                mask1[0:search_top, 0:w] = 0
                mask2[0:search_top, 0:w] = 0

                M1 = cv2.moments(mask1)
                M2 = cv2.moments(mask2)

                if M1['m00'] > 0:
                    cx1 = int(M1['m10']/M1['m00'])
                    cy1 = int(M1['m01']/M1['m00'])

                    cx2 = int(M2['m10']/M2['m00'])
                    cy2 = int(M2['m01']/M2['m00'])

		    fpt_x = (cx1 + cx2)/2
                    fpt_y = (cy1 + cy2)/2 + 2*h/3

                    cv2.circle(img_bv, (cx1, cy1), 10, (0,255,255), -1)
                    cv2.circle(img_bv, (cx2, cy2), 10, (255,255,255), -1)
                    cv2.circle(img_bv, (fpt_x, fpt_y), 10, (128,128,128), -1)

                    err = w/2 - fpt_x

                    self.twist.linear.x = 0.4
		    self.twist.angular.z = (err*90.0/160)/15
		    #if err > 15:
                    #    self.twist.angular.z = (err*90.0/160)/3
			#print(err, self.twist.angular.z)
		    #else:
			#self.twist.angular.z = (err*90.0/160)/15
			#print(err, self.twist.angular.z)

                    self.cmd_vel_pub.publish(self.twist)
                cv2.imshow("window", img_bv)
                cv2.imshow("window2", image)
                cv2.waitKey(1)

rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
