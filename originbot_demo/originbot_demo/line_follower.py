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

    def image_callback(self, msg):
        print("Received image frame")  # Debugging
        image_input = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image_input, cv2.COLOR_BGR2HSV)
        
        # Define the yellow color range (Adjust based on your environment)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])  # Adjusted range
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        h, w, _ = image_input.shape
        search_top = int(3 * h / 4)
        search_bot = int(3 * h / 4 + 20)
        mask[:search_top, :] = 0  # Mask out upper part
        mask[search_bot:, :] = 0  # Mask out lower part

        M = cv2.moments(mask)
        twist = Twist()

        if M['m00'] > 0:  # If a yellow line is detected
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(image_input, (cx, cy), 20, (0, 0, 255), -1)

            err = cx - w / 2
            twist.linear.x = 0.3  # Move forward
            twist.angular.z = -float(err) / 500  # Adjust direction
            print(f"Following line. Linear: {twist.linear.x}, Angular: {twist.angular.z}") 

        else:  # No line detected -> Stop the robot
            print("No line detected. Stopping the robot.")
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        # Publish movement command
        self.publisher.publish(twist)
        s = str(M['m00'])
        # Debugging
        cv2.imshow("Check cache", image_input)
        cv2.waitKey(3)

    def stop_robot(self):
        """Stops the robot when the node is destroyed."""
        print("Shutting down: Stopping the robot")
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        for _ in range(5):  
            self.publisher.publish(stop_twist)
            time.sleep(0.1)

def main():
    rclpy.init()
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected. Stopping robot.")
    finally:
        node.stop_robot()
        time.sleep(1)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
