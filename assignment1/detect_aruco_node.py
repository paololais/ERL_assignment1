#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import cv2
import cv2.aruco as aruco
import math


class DetectArucoNode(Node):
    """
    ROS2 node to detect ArUco markers and sequentially center the robot on each marker.

    Workflow:
    1. Rotate to detect all markers and store their approximate angles.
    2. Sequentially rotate to center each marker in the camera image and publish an annotated image.
    3. Pause briefly after each marker before moving to the next.
    """
    def __init__(self):
        super().__init__('detect_aruco_node')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_pub = self.create_publisher(Image, '/aruco_centered_image', 10)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Image, '/camera/image', self.image_callback, 10)

        self.bridge = CvBridge()
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters_create()

        # Odometry
        self.current_yaw = 0.0
        self.yaw_received = False

        # State machine: "rotating" -> "centering" -> "done"
        self.state = "rotating"
        
        # Rotation
        self.angular_speed = 0.25
        self.total_rotated = 0.0
        self.prev_yaw = 0.0
        
        # Markers found during rotation
        self.detected_ids = set()
        self.marker_angles = {}  # Store angular position of each marker
        self.target_list = []
        
        # Current image
        self.current_image = None
        self.current_target = None
        self.last_published_id = None
        self.pause_timer = None

        self.get_logger().info("Waiting for odometry...")
        while not self.yaw_received:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Starting 360° rotation")
        self.rotation_timer = self.create_timer(0.1, self.rotate)
        
    def odom_callback(self, msg: Odometry):
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_yaw = yaw
        self.yaw_received = True
        
    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        if self.state == "rotating":
            self.detect_markers()
        elif self.state == "centering":
            # Skip processing if in pause after centering a marker
            if self.pause_timer is None:
                self.track_and_center_current_marker()
        elif self.state == "done":
            self.cmd_vel_pub.publish(Twist())
    
    ############################################################
    # PHASE 1: Rotation
    ############################################################
    
    def rotate(self):
        """
        Rotates the robot to detect all markers. Stops if all markers found.
        """
        if self.state != "rotating":
            return
        
        # Early exit if all markers found
        if len(self.detected_ids) == 5:
            self.get_logger().info(f"All 5 markers found! Stopping rotation early.")
            self.stop_rotation()
            return
        
        twist = Twist()
        twist.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(twist)
        
        # Accumulate rotation
        delta_yaw = self.current_yaw - self.prev_yaw
        delta_yaw = (delta_yaw + math.pi) % (2 * math.pi) - math.pi
        self.total_rotated += abs(delta_yaw)
        self.prev_yaw = self.current_yaw        
    
    def stop_rotation(self):
        """
        Stops rotation and transitions to centering phase.
        """
        self.cmd_vel_pub.publish(Twist())
        self.rotation_timer.cancel()
        
        # Transition to centering phase
        self.target_list = sorted(list(self.detected_ids))
        if self.target_list:
            self.state = "centering"
            self.current_target = self.target_list.pop(0)
            self.get_logger().info(f"Starting centering phase. First target: {self.current_target}")
        else:
            self.get_logger().error("No markers found!")
            self.state = "done"
    
    def detect_markers(self):
        """
        Detects ArUco markers in the current image and stores their IDs and approximate angles.
        """
        if self.current_image is None:
            return

        gray = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            for m_id in ids.flatten():
                if m_id not in self.detected_ids:
                    self.detected_ids.add(m_id)
                    self.marker_angles[m_id] = self.current_yaw
                    self.get_logger().info(f"Found marker ID {m_id} at angle {math.degrees(self.current_yaw):.1f}°")

    ############################################################
    # PHASE 2: Centering
    ############################################################
    def get_angular_error_to_marker(self, marker_id):
        """Calculate shortest angular distance to marker's known position."""
        target_angle = self.marker_angles.get(marker_id, self.current_yaw)
        error = target_angle - self.current_yaw
        # Normalize to [-pi, pi]
        error = (error + math.pi) % (2 * math.pi) - math.pi
        return error

    def track_and_center_current_marker(self):
        if self.current_image is None:
            return

        gray = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        # Search for target if not visible
        if ids is None or self.current_target not in ids.flatten():
            # Use known marker angle to search more efficiently
            angle_error = self.get_angular_error_to_marker(self.current_target)
            twist = Twist()
            twist.angular.z = 0.25 if angle_error > 0 else -0.25
            self.cmd_vel_pub.publish(twist)
            return

        # Target found - center it
        idx = list(ids.flatten()).index(self.current_target)
        pts = corners[idx][0]

        img_w = gray.shape[1]
        cx = pts[:, 0].mean()
        error_x = cx - (img_w / 2)

        twist = Twist()
        twist.angular.z = -0.003 * error_x
        self.cmd_vel_pub.publish(twist)

        # Marker centered
        if abs(error_x) < 5:
            if self.last_published_id != self.current_target:
                self.publish_centered_image(pts)
                self.last_published_id = self.current_target

            self.cmd_vel_pub.publish(Twist())
            self.pause_timer = self.create_timer(2.0, self.advance_to_next_marker)

    def advance_to_next_marker(self):
        self.pause_timer.cancel()
        self.pause_timer = None
        
        if self.target_list:
            self.current_target = self.target_list.pop(0)
            self.get_logger().info(f"Next marker: {self.current_target}")
        else:
            self.get_logger().info("All markers processed. Complete.")
            self.state = "done"

    def publish_centered_image(self, pts):
        cx = int(pts[:, 0].mean())
        cy = int(pts[:, 1].mean())
        out = self.current_image.copy()
        
        cv2.circle(out, (cx, cy), 20, (0, 255, 0), 3)
        text = f"MarkerID = {self.current_target}"
        cv2.putText(out, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3, cv2.LINE_AA)

        msg = self.bridge.cv2_to_imgmsg(out, encoding="bgr8")
        self.image_pub.publish(msg)
        self.get_logger().info(f"Published centered image for marker {self.current_target}")

    
def main(args=None):
    rclpy.init(args=args)
    node = DetectArucoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()