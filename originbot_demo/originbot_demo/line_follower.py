import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2
import cv_bridge
import time

class LineFollower(Node):
    def __init__(self):
        super().__init__('follower')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        self.bridge = cv_bridge.CvBridge()
        
        self.searching = False  # Indicates if the robot is in search mode
        self.search_direction = 1  # 1 = left, -1 = right

    def image_callback(self, msg):
        print("Received image frame")  # Debugging: Check if callback is triggering
        image_input = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image_input, cv2.COLOR_BGR2HSV)
        
        # Define the yellow color range
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        h, w, d = image_input.shape
        search_top = int(3 * h / 4)
        search_bot = int(3 * h / 4 + 20)
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        M = cv2.moments(mask)
        twist = Twist()

        if M['m00'] != 0:  # Line detected
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image_input, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - w / 2
            twist.linear.x = 0.7  # Move forward
            twist.angular.z = -float(err) / 500  # Adjust direction
            self.searching = False  # Reset search mode
            print(f"Following line. Linear: {twist.linear.x}, Angular: {twist.angular.z}")

            if cx == 0:
                twist.linear.x = 0.0  # Stop moving forward
                twist.angular.z = 0.0
                
        else:  # No line detected -> Start searching
            twist.linear.x = 0.0  # Stop moving forward
            twist.angular.z = 0.0

        self.publisher.publish(twist)
        cv2.imshow("detect_line", image_input)
        cv2.waitKey(3)

    def stop_robot(self):
        """Stops the robot when the node is destroyed."""
        print("Shutting down: Stopping the robot")
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        for _ in range(5):  # Publish multiple times to ensure stop command is received
            self.publisher.publish(stop_twist)

def main():
    rclpy.init()
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected. Stopping robot.")
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()