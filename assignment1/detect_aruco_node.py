#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import cv2
import cv2.aruco as aruco
import math
from collections import deque


class DetectArucoNode(Node):
    """
    ROS2 node to detect ArUco markers and sequentially center the robot on each marker.

    Workflow:
    1. Rotate to detect all markers and store their approximate angles.
    2. Sequentially rotate to center each marker in the camera image and publish an annotated image.
    3. Stop briefly after each marker before moving to the next.
    """
    def __init__(self):
        super().__init__('detect_aruco_node')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_pub = self.create_publisher(Image, '/aruco_centered_image', 10)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(CompressedImage, '/camera/image/compressed', self.image_callback, 10)

        self.bridge = CvBridge()
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters_create()

        # Odometry
        self.current_yaw = 0.0
        self.yaw_received = False

        # State machine: "rotating" -> "centering" -> "done"
        self.state = "rotating"
        
        # Rotation
        self.angular_speed = 0.2
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
        
        self.center_history = deque(maxlen=5)   # smoothing buffer for cx
        self.REQUIRED_CONSECUTIVE = 3           # frames required to confirm centered
        self.centered_counts = {}               # counts per marker id
        self.controller_gain = 0.0005           # P gain
        self.max_angular = 0.15                 # clamp for angular speed (rad/s)
        self.center_deadband = 10               # pixels tolerance

        self.get_logger().info("Waiting for odometry...")
        while not self.yaw_received:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Starting 360° rotation")
        
        # Create FSM timer to handle state updates
        self.fsm_timer = self.create_timer(0.1, self.update_fsm)
    
    ##############################################################
    # Callbacks
    ##############################################################
    def odom_callback(self, msg: Odometry):
        """
        Update current yaw from odometry message.
        """
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_yaw = yaw
        self.yaw_received = True
        
    def image_callback(self, msg):
        """
        Processes camera images. 
        """
        try:
            # self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return
    
    #############################################################
    # Finite State Machine (FSM) Methods
    #############################################################
    def update_fsm(self):
        """
        FSM timer callback.
        """
        if self.state == "rotating":
            self.update_rotating_state()
        elif self.state == "centering":
            self.update_centering_state()
        elif self.state == "done":
            self.update_done_state()

    def update_rotating_state(self):
        """
        Handle rotation phase: detect markers and rotate until complete.
        """
        # Detect markers in current image
        if self.current_image is not None:
            self.detect_markers()
        
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

    def update_centering_state(self):
        """
        Handle centering phase: track and center current marker.
        Skip if in pause after centering.
        """
        if self.pause_timer is not None:
            return
        
        if self.current_image is not None:
            self.track_and_center_current_marker()

    def update_done_state(self):
        """
        Handle done phase: stop all motion.
        """
        self.cmd_vel_pub.publish(Twist())
        
    ##############################################################
    # Helper Methods
    ##############################################################
    def stop_rotation(self):
        """
        Stops rotation and transitions to centering phase.
        """
        self.cmd_vel_pub.publish(Twist())
        
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
        _, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            for m_id in ids.flatten():
                if m_id not in self.detected_ids:
                    self.detected_ids.add(m_id)
                    self.marker_angles[m_id] = self.current_yaw
                    self.get_logger().info(f"Found marker ID {m_id} at angle {math.degrees(self.current_yaw):.1f}°")

    def get_angular_error_to_marker(self, marker_id):
        """
        Calculate shortest angular distance to marker's known position.
        """
        target_angle = self.marker_angles.get(marker_id, self.current_yaw)
        error = target_angle - self.current_yaw
        # Normalize to [-pi, pi]
        error = (error + math.pi) % (2 * math.pi) - math.pi
        return error

    def track_and_center_current_marker(self):
        """
        Rotate to center the current target marker. Uses:
        - robust index lookup for target
        - smoothing on cx via moving average
        - consecutive-frame confirmation before declaring centered to avoid false positives
        """
        if self.current_image is None:
            return

        gray = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        # If the target is not visible, search using known angle
        if ids is None or self.current_target not in ids.flatten():
            # Use known marker angle to search more efficiently
            angle_error = self.get_angular_error_to_marker(self.current_target)
            twist = Twist()
            twist.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
            # publish a limited (safe) angular velocity
            twist.angular.z = max(min(twist.angular.z, self.max_angular), -self.max_angular)
            self.cmd_vel_pub.publish(twist)
            # clear smoothing so old values don't affect future centering
            self.center_history.clear()
            return

        # Find the index corresponding to current_target safely
        ids_flat = ids.flatten()
        idxs = [i for i, val in enumerate(ids_flat) if val == self.current_target]
        if not idxs:
            # somehow not found, fallback to search behavior
            angle_error = self.get_angular_error_to_marker(self.current_target)
            twist = Twist()
            twist.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
            twist.angular.z = max(min(twist.angular.z, self.max_angular), -self.max_angular)
            self.cmd_vel_pub.publish(twist)
            self.center_history.clear()
            return

        idx = idxs[0]
        pts = corners[idx][0]   # shape (4,2)

        # compute cx and smooth it
        img_w = gray.shape[1]
        cx = float(pts[:, 0].mean())
        self.center_history.append(cx)
        smooth_cx = sum(self.center_history) / len(self.center_history)

        error_x = smooth_cx - (img_w / 2)

        # P controller with clamp
        twist = Twist()
        twist.angular.z = -self.controller_gain * error_x
        # limit angular speed
        twist.angular.z = max(min(twist.angular.z, self.max_angular), -self.max_angular)
        self.cmd_vel_pub.publish(twist)

        # consecutive-frame confirmation before declaring centered
        if abs(error_x) < self.center_deadband:
            self.centered_counts[self.current_target] = self.centered_counts.get(self.current_target, 0) + 1
        else:
            self.centered_counts[self.current_target] = 0

        if self.centered_counts[self.current_target] >= self.REQUIRED_CONSECUTIVE:
            # publish only once per marker
            if self.last_published_id != self.current_target:
                self.publish_centered_image(pts)
                self.last_published_id = self.current_target

            # stop rotation and set a pause before moving to next
            self.cmd_vel_pub.publish(Twist())
            # reset counter and smoothing buffer (clean state)
            self.centered_counts[self.current_target] = 0
            self.center_history.clear()
            self.pause_timer = self.create_timer(2.0, self.advance_to_next_marker)


    def advance_to_next_marker(self):
        """
        Timer callback after centering a marker; moves to next marker or finishes.
        """
        self.pause_timer.cancel()
        self.pause_timer = None
        
        if self.target_list:
            self.current_target = self.target_list.pop(0)
            self.get_logger().info(f"Next marker: {self.current_target}")
        else:
            self.get_logger().info("All markers processed. Complete.")
            self.state = "done"

    def publish_centered_image(self, pts):
        """
        Draws and publishes an image with:
        - Circle passing through all 4 marker corners
        - Marker ID label
        """
        out = self.current_image.copy()

        # Compute center of marker
        center = pts.mean(axis=0)
        cx, cy = int(center[0]), int(center[1])

        # Compute radius so circle covers all 4 corners
        dists = [math.dist((cx, cy), (p[0], p[1])) for p in pts]
        radius = int(max(dists))

        # Draw circle around marker
        cv2.circle(out, (cx, cy), radius, (0, 0, 255), 2)

        # Write marker ID on image
        text = f"MarkerID = {self.current_target}"
        cv2.putText(out, text, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3, cv2.LINE_AA)

        # Publish image
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