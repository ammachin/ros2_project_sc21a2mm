# Exercise 4 - following a colour (green) and stopping upon sight of another (blue).

#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
import signal


class Robot(Node):
    def __init__(self):
        super().__init__('robot')
        
        # For showing camera
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
        self.subscription  # prevent unused variable warning

        # Initialising values
        self.sensitivity = 10
        
        self.red_found = False
        self.green_found = False
        self.blue_found = False
        
        self.threshold_contour = 100
        self.threshold_move = 30000
        
        self.go_back = False
        self.go_forward = False
        
        # For moving the robot
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)
        

    def callback(self, data):

        # Convert the received image into a opencv image
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        
        # Set up camera image
        cv2.namedWindow('camera_feed',cv2.WINDOW_NORMAL)
        cv2.imshow('camera_feed', image)
        cv2.resizeWindow('camera_feed',320,240)
        cv2.waitKey(3)
        
        
        # Setting upper / lower bounds for RGB 
        hsv_red_lower = np.array([0 - self.sensitivity, 100, 100])
        hsv_red_upper = np.array([0 + self.sensitivity, 255, 255])
        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
        

        # Convert the rgb image into a hsv image
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Filter out everything but a particular colour
        red_image = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)
        green_image = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
        blue_image = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)

        # Combining into one image
        rg_mask = cv2.bitwise_or(red_image, green_image)
        rgb_mask = cv2.bitwise_or(rg_mask, blue_image)
        final_image = cv2.bitwise_and(image, image, mask=rgb_mask)


        # Find the contours that appear within the certain colour mask
        # Just blue first
        contours, _ = cv2.findContours(blue_image, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
        self.green_found = False
        self.go_forward = False
        self.go_back = False

        # Loop over the contours
        if len(contours)>0:
            # Find the largest contour
            c = max(contours, key=cv2.contourArea)

            if cv2.contourArea(c) > self.threshold_contour:
                # Draw circle
                (x, y), radius = cv2.minEnclosingCircle(c)
                centre = (int(x), int(y))
                radius = int(radius)
                
                cv2.circle(image, centre, radius, (255, 255, 0), 1)
                
                self.blue_found = True
            else:
                self.blue_found = False
                
        #if self.blue_found is True:
            #print("Blue detected!")
            
        # Show image
        cv2.namedWindow('blue_resultant', cv2.WINDOW_NORMAL)
        cv2.imshow('blue_resultant', image)
        cv2.resizeWindow('blue_resultant', 320, 240)
        cv2.waitKey(3)
            

        # Go to blue if found
        if self.blue_found is True:
            if cv2.contourArea(c) > self.threshold_move:
                # Too close to object
                #print("Going backwards")
                self.go_back = True
                
            elif cv2.contourArea(c) < self.threshold_move:
                # Too far away from object
                #rint("Going forwards")
                self.go_forward = True
            # else:
            #     self.stop() 

    def walk_forward(self):
        desired_vel = Twist()
        desired_vel.linear.x = 0.2
        
        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_vel)
            self.rate.sleep()

    def walk_backward(self):
        desired_vel = Twist()
        desired_vel.linear.x = -0.2

        for _ in range(30):  # Stop for a brief moment
            self.publisher.publish(desired_vel)
            self.rate.sleep()

    def stop(self):
        desired_vel = Twist()
        desired_vel.linear.x = 0.0
        self.publisher.publish(desired_vel)

def main():
    def signal_handler(sig, frame):
        robot.stop()
        rclpy.shutdown()

    # Instantiation
    rclpy.init(args=None)
    robot = Robot()
    
    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    thread.start()

    try:
        while rclpy.ok():
            if robot.blue_found is True:
                if robot.go_back is True:
                    robot.walk_backward()
                elif robot.go_forward is True:
                    robot.walk_forward()

    except ROSInterruptException:
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
