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
    def __init__(self):
        super().__init__('detect_aruco_node')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_pub = self.create_publisher(Image, '/aruco_centered_image', 10)

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Image, '/camera/image', self.image_callback, 10)

        # ArUco config
        self.bridge = CvBridge()
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = aruco.DetectorParameters_create()

        # Odometry state
        self.current_yaw = 0.0
        self.yaw_received = False

        # Rotation state
        self.angular_speed = 0.25
        self.rotation_done = False
        self.total_rotated = 0.0
        self.prev_yaw = self.current_yaw

        # Marker detection state
        self.detected_ids = set()
        self.total_expected = 5
        self.all_found = False  # set when all markers have been detected during sweep

        # Movement to marker state
        self.current_image = None
        self.target_list = []
        self.current_target = None

        # flags
        self.centering_started = False   # set once to begin visiting markers in order
        self.processing_done = False     # set when all markers have been visited & published
        self.last_published_id = None    # avoid repeated publishes for same marker
        self.pause_active = False        # to manage 2-second pause after centering marker
        self.pause_timer = None          # timer handle for pause

        self.get_logger().info("Waiting for odometry...")
        while not self.yaw_received:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Odometry OK - starting 360° rotation")
        self.rotation_timer = self.create_timer(0.1, self.rotate_360)
        
    # Callback for odometry messages
    def odom_callback(self, msg: Odometry):
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.current_yaw = yaw
        self.yaw_received = True
        
    # Callback for image messages
    def image_callback(self, msg):
        # Convert to OpenCV image
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        # If still rotating and haven't found all markers yet -> keep detecting
        if not self.rotation_done and not self.all_found:
            self.detect_markers()
            return

        # If all markers were detected during sweep but we haven't started the centering phase yet,
        # initialize the ordered visit exactly once
        if self.rotation_done and self.all_found and not self.centering_started and not self.processing_done:
            self.centering_started = True
            # create sorted ID list and pick first target
            self.target_list = sorted(list(self.detected_ids))
            if len(self.target_list) == 0:
                self.get_logger().error("No markers to visit even though all_found is True.")
                self.processing_done = True
                return
            self.current_target = self.target_list.pop(0)
            self.get_logger().info(f"Starting centering phase. First target: {self.current_target}")

        # If centering / visiting is active and not finished, do the centering loop
        if self.centering_started and not self.processing_done:
            self.track_and_center_current_marker()
            return

        # If processing done -> make sure robot is stopped
        if self.processing_done:
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return
    
    ##############################################################
    # First phase:
    
    # 360-degree rotation with marker detection   
    def rotate_360(self):
        if self.rotation_done:
            return
        
        # Stop early if all markers found
        if len(self.detected_ids) == self.total_expected:
            self.get_logger().info("All markers found - stopping rotation early.")
            self.stop_rotation()
            self.all_found = True
            return
        
        
        twist = Twist()
        twist.angular.z = self.angular_speed
        self.cmd_vel_pub.publish(twist)
        
        # Compute yaw delta
        delta_yaw = self.current_yaw - self.prev_yaw
        delta_yaw = (delta_yaw + math.pi) % (2 * math.pi) - math.pi
        self.total_rotated += abs(delta_yaw)
        self.prev_yaw = self.current_yaw

        # check if rotated 360
        if self.total_rotated >= 2 * math.pi:
            self.get_logger().info("360° rotation complete.")
            self.stop_rotation()

            if len(self.detected_ids) == self.total_expected:
                self.all_found = True
                self.get_logger().info("All markers detected.")
            else:
                self.get_logger().info(f"Not all markers detected (found {len(self.detected_ids)} of {self.total_expected}).")
       
    def stop_rotation(self):
        stop = Twist()
        self.cmd_vel_pub.publish(stop)
        self.rotation_done = True

        # Kill the timer safely
        self.rotation_timer.cancel()
    
    # detect markers during rotation
    def detect_markers(self):
        if self.current_image is None:
            return

        gray = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None and len(ids) > 0:
            for m_id in ids.flatten():
                if m_id not in self.detected_ids:
                    self.get_logger().info(f"Found marker ID {m_id}")
                    self.detected_ids.add(m_id)

        # Only mark all_found and prepare the ordered list once, don't repeatedly re-run this block
        if len(self.detected_ids) == self.total_expected and not self.all_found:
            self.all_found = True
            self.get_logger().info("All markers found during rotation! Preparing to visit markers in ascending order.")
            # target_list will be created when centering starts (after rotation_done)
    
    ####################################################################
    # Second phase:
    # Move robot to center on the current target marker
    def track_and_center_current_marker(self):
        if self.current_image is None:
            return

        gray = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        # If we don't see any marker in this frame, rotate slowly to search for current target
        if ids is None or len(ids) == 0:
            twist = Twist()
            twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(twist)
            return

        ids = ids.flatten()
        if self.current_target not in ids:
            # rotate slowly to search for the target marker
            twist = Twist()
            twist.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(twist)
            return

        # we see the current target → center it
        idx = list(ids).index(self.current_target)
        pts = corners[idx][0]  # 4x2 array

        img_h, img_w = gray.shape
        cx = pts[:,0].mean()
        cy = pts[:,1].mean()

        error_x = cx - (img_w/2)

        twist = Twist()        
        k = 0.003  # gain
        twist.angular.z = -k * error_x # angular correction
        self.cmd_vel_pub.publish(twist)

        # When the marker is centered → publish final annotated image and advance
        if abs(error_x) < 5 and not self.pause_active:
            # avoid republishing same ID multiple consecutive frames
            if self.last_published_id != self.current_target:
                self.publish_centered_image(pts)
                self.last_published_id = self.current_target

            # Stop robot for 2 sec
            stop = Twist()
            stop.linear.x = 0.0
            stop.angular.z = 0.0
            self.cmd_vel_pub.publish(stop)
            
            # activate 2-second pause
            self.pause_active = True
            self.pause_timer = self.create_timer(2.0, self.finish_pause_and_advance)

            return
            
    def finish_pause_and_advance(self):
        # stop calling this timer
        self.pause_timer.cancel()
        self.pause_timer = None
        self.pause_active = False

        # Move to next target, or finish
        if self.target_list:
            self.current_target = self.target_list.pop(0)
            self.get_logger().info(f"Next marker: {self.current_target}")
        else:
            self.get_logger().info("All markers processed and published. Finishing.")
            self.processing_done = True
            self.cmd_vel_pub.publish(Twist())

    # Draw circle and publish image
    def publish_centered_image(self, pts):
        cx = int(pts[:,0].mean())
        cy = int(pts[:,1].mean())
        out = self.current_image.copy()
        cv2.circle(out, (cx, cy), 20, (0,255,0), 3)
        
        # Write marker ID on image
        text = f"MarkerID = {self.current_target}"
        cv2.putText(out, text, (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,0), 3, cv2.LINE_AA)

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
