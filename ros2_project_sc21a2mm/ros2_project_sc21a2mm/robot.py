# Exercise 4 - following a colour (green) and stopping upon sight of another (blue).

#from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
from nav2_msgs.action import NavigateToPose
import signal
from math import sin, cos

class GoToPose(Node):
    def __init__(self):
        super().__init__('navigation_goal_action_client')
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        self.goals = []
        
    def manage_goals(self):
        if(len(self.goals) != 0):
            self.send_goal(self.goals[0][0], self.goals[0][1], self.goals[0][2])
            self.goals.pop(0)
        

    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Orientation - yaw / pitch / roll converted to quaternion representation
        # avoids issues like gimbal lock (loss of degree of freedom)
        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation result: {result}')
        self.manage_goals()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(feedback)


# class Robot(Node):
#     def __init__(self):
#         super().__init__('robot')
        
#         # For showing camera
#         self.bridge = CvBridge()
#         self.subscription = self.create_subscription(Image, '/camera/image_raw', self.callback, 10)
#         self.subscription  # prevent unused variable warning
        
#         # For robot movement
#         self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

#         # Initialising values
#         self.sensitivity = 10
        
#         self.red_found = False
#         self.green_found = False
#         self.blue_found = False
        
#         self.threshold_contour = 100
#         self.threshold_move = 30000
        
#         self.go_back = False
#         self.go_forward = False
        
#         # For moving the robot
#         self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
#         self.rate = self.create_rate(10)
        
#     # lab5
#     def callback(self, data):
#         # Convert the received image into a opencv image
#         image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        
#         # Set up camera image
#         cv2.namedWindow('camera_feed',cv2.WINDOW_NORMAL)
#         cv2.imshow('camera_feed', image)
#         cv2.resizeWindow('camera_feed',320,240)
#         cv2.waitKey(3)
        
        
#         # Setting upper / lower bounds for RGB 
#         hsv_red_lower = np.array([0 - self.sensitivity, 100, 100])
#         hsv_red_upper = np.array([0 + self.sensitivity, 255, 255])
#         hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
#         hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])
#         hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
#         hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])
        

#         # Convert the rgb image into a hsv image
#         hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

#         # Filter out everything but a particular colour
#         red_image = cv2.inRange(hsv_image, hsv_red_lower, hsv_red_upper)
#         green_image = cv2.inRange(hsv_image, hsv_green_lower, hsv_green_upper)
#         blue_image = cv2.inRange(hsv_image, hsv_blue_lower, hsv_blue_upper)

#         # Combining into one image
#         rg_mask = cv2.bitwise_or(red_image, green_image)
#         rgb_mask = cv2.bitwise_or(rg_mask, blue_image)
#         final_image = cv2.bitwise_and(image, image, mask=rgb_mask)


#         # Find the contours that appear within the certain colour mask
#         # Just blue first
#         b_contours, _ = cv2.findContours(blue_image, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
#         self.blue_found = False
#         self.go_forward = False
#         self.go_back = False

#         # Loop over the contours
#         if len(b_contours)>0:
#             # Find the largest contour
#             c = max(b_contours, key=cv2.contourArea)

#             if cv2.contourArea(c) > self.threshold_contour:
#                 # Draw circle
#                 (x, y), radius = cv2.minEnclosingCircle(c)
#                 centre = (int(x), int(y))
#                 radius = int(radius)
                
#                 cv2.circle(image, centre, radius, (255, 255, 0), 1)
                
#                 self.blue_found = True
#             else:
#                 self.blue_found = False
                
#         #if self.blue_found is True:
#             #print("Blue detected!")
            
        
#         # Repeat for green
#         g_contours, _ = cv2.findContours(green_image, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
#         self.green_found = False

#         # Loop over the contours
#         if len(g_contours)>0:
#             # Find the largest contour
#             c = max(g_contours, key=cv2.contourArea)

#             if cv2.contourArea(c) > self.threshold_contour:
#                 # Draw circle
#                 (x, y), radius = cv2.minEnclosingCircle(c)
#                 centre = (int(x), int(y))
#                 radius = int(radius)
                
#                 cv2.circle(image, centre, radius, (255, 0, 255), 1)
                
#                 self.green_found = True
#             else:
#                 self.green_found = False
                
#         #if self.green_found is True:
#             #print("Green detected!")
            
#         # Repeat for red
#         r_contours, _ = cv2.findContours(red_image, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
#         self.red_found = False
#         self.go_forward = False
#         self.go_back = False

#         # Loop over the contours
#         if len(r_contours)>0:
#             # Find the largest contour
#             c = max(r_contours, key=cv2.contourArea)

#             if cv2.contourArea(c) > self.threshold_contour:
#                 # Draw circle
#                 (x, y), radius = cv2.minEnclosingCircle(c)
#                 centre = (int(x), int(y))
#                 radius = int(radius)
                
#                 cv2.circle(image, centre, radius, (0, 255, 255), 1)
                
#                 self.red_found = True
#             else:
#                 self.red_found = False
                
#         #if self.red_found is True:
#             #print("Red detected!")
            
            
#         # Show image
#         cv2.namedWindow('resultant', cv2.WINDOW_NORMAL)
#         cv2.imshow('resultant', image)
#         cv2.resizeWindow('resultant', 320, 240)
#         cv2.waitKey(3)
            

#         # Go to blue if found
#         if self.blue_found is True:
#             if cv2.contourArea(c) > self.threshold_move:
#                 # Too close to object
#                 #print("Going backwards")
#                 self.go_back = True
                
#             elif cv2.contourArea(c) < self.threshold_move:
#                 # Too far away from object
#                 #print("Going forwards")
#                 self.go_forward = True
#             # else:
#             #     self.stop() 

#     # lab3
#     def walk_forward(self):
#         desired_vel = Twist()
#         desired_vel.linear.x = 0.2
        
#         for _ in range(30):  # Stop for a brief moment
#             self.publisher.publish(desired_vel)
#             self.rate.sleep()

#     # lab3
#     def walk_backward(self):
#         desired_vel = Twist()
#         desired_vel.linear.x = -0.2

#         for _ in range(30):  # Stop for a brief moment
#             self.publisher.publish(desired_vel)
#             self.rate.sleep()

#     # lab3
#     def stop(self):
#         desired_vel = Twist()
#         desired_vel.linear.x = 0.0
#         self.publisher.publish(desired_vel)
        
#     # lab4    
#     def send_goal(self, x, y, yaw):
#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose.header.frame_id = 'map'
#         goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
#         # Position
#         goal_msg.pose.pose.position.x = x
#         goal_msg.pose.pose.position.y = y
        
#         # Orientation - yaw / pitch / roll converted to quaternion representation
#         # avoids issues like gimbal lock (loss of freedom)
#         goal_msg.pose.pose.orientation.z = sin(yaw / 2)
#         goal_msg.pose.pose.orientation.w = cos(yaw / 2)
        
#         self.action_client.wait_for_server()
#         self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
#         self.send_goal_future.add_done_callback(self.goal_response_callback)
        
#     # lab4
#     def goal_response_callback(self, future):
#         print("AAAAAAAAAAAA")
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('Goal rejected')
#             print("rejection?")
#             return
        
#         self.get_logger().info('Goal accepted')
#         print("accepted?")
#         self.get_result_future = goal_handle.get_result_async()
#         self.get_result_future.add_done_callback(self.get_result_callback)
    
#     # lab4   
#     def get_result_callback(self, future):
#         result = future.result().result
#         self.get_logger().info(f'Navigation result: {result}')
        
#     # lab4 - not really used
#     def feedback_callback(self, feedback_msg):
#         feedback = feedback_msg.feedback
#         print(feedback)

def main():
    # def signal_handler(sig, frame):
    #     robot.stop()
    #     rclpy.shutdown()

    # Instantiation
    rclpy.init(args=None)
    #robot = Robot()
    
    # signal.signal(signal.SIGINT, signal_handler)
    # thread = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    # thread.start()
    
    # Corners of the map - picked from looking at the map in RViz
    poses = [(-10.8, 3.5, 0.0),
             (7.0, 5.1, 0.0),
             (8.25, -12.5, 0.0),
             (-9.3, -14.3, 0.0)]

    go_to_pose = GoToPose()
    
    for pose in poses:
        go_to_pose.goals.append(pose)
        
    
    go_to_pose.manage_goals()
    rclpy.spin(go_to_pose)
    
    go_to_pose.destroy_node()
    rclpy.shutdown()



    # Remember to destroy all image windows before closing node
    #cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
