#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, MotionPlanRequest
from shape_msgs.msg import SolidPrimitive
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class HCR5SuctionVisionController(Node):
    def __init__(self):
        super().__init__('hcr5_vision_controller')
        self.bridge = CvBridge()
        self.latest_coords = None
        self.target_color_mode = "PURPLE" 
        
        # HSV Color Ranges for Gazebo
        self.color_configs = {
            "PURPLE": {"low": np.array([120, 50, 50]), "high": np.array([170, 255, 255]), "bgr": (255, 0, 255)},
            "BLUE": {"low": np.array([100, 50, 50]), "high": np.array([140, 255, 255]), "bgr": (255, 0, 0)}
        }

        # Load Calibration Matrix
        script_dir = os.path.dirname(os.path.realpath(__file__))
        calib_path = os.path.join(script_dir, "table_calibration.npy")
        if os.path.exists(calib_path):
            self.M = np.load(calib_path)
            self.get_logger().info("Calibration loaded.")
        else:
            self.get_logger().error(f"Calibration file NOT FOUND at {calib_path}")
            self.M = None

        # MoveIt Action Client
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Camera Subscription
        self.sub = self.create_subscription(Image, 'camera1/image_raw', self.image_callback, 10)
        
        self.get_logger().info("-" * 30)
        self.get_logger().info("HCR5 SUCTION TIP CONTROL READY")
        self.get_logger().info("SPACE: Move to Object | P: Purple | B: Blue")
        self.get_logger().info("-" * 30)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.color_configs[self.target_color_mode]["low"], 
                                self.color_configs[self.target_color_mode]["high"])
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.latest_coords = None

        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 200:
                Moments = cv2.moments(largest)
                if Moments["m00"] != 0 and self.M is not None:
                    u, v = int(Moments["m10"]/Moments["m00"]), int(Moments["m01"]/Moments["m00"])
                    pixel_pt = np.array([u, v, 1], dtype="float32")
                    world_pt = np.dot(self.M, pixel_pt)
                    self.latest_coords = (world_pt[0] / world_pt[2], world_pt[1] / world_pt[2])
                    cv2.circle(frame, (u, v), 8, (0, 255, 0), -1)

        cv2.imshow("Vision Control", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('p'): self.target_color_mode = "PURPLE"
        elif key == ord('b'): self.target_color_mode = "BLUE"
        elif key == ord(' '): 
            if self.latest_coords:
                # Set Z to 0.05 (5cm above base) to avoid table collisions during initial tests
                self.send_move_goal(self.latest_coords[0], self.latest_coords[1], 0.04)

    def send_move_goal(self, x, y, z):
        if not self.move_group_client.wait_for_server(timeout_sec=1.0):
            return

        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = "arm"
        req.num_planning_attempts = 10
        req.max_velocity_scaling_factor = 0.5
        req.allowed_planning_time = 5.0 # Fixed the 0.0s timeout issue

        constraints = Constraints()
        
        # Position Constraint
        pos_con = PositionConstraint()
        pos_con.header.frame_id = "base_link"
        pos_con.link_name = "suction_cup_link" # Target the cup link directly
        pos_con.weight = 1.0
        
        target_pose = PoseStamped()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        
        box = SolidPrimitive()
        box.type, box.dimensions = SolidPrimitive.BOX, [0.01, 0.01, 0.01]
        
        pos_con.constraint_region.primitives.append(box)
        pos_con.constraint_region.primitive_poses.append(target_pose.pose)
        constraints.position_constraints.append(pos_con)

        # Orientation Constraint (Suction cup pointing down)
        ori_con = OrientationConstraint()
        ori_con.header.frame_id = "base_link"
        ori_con.link_name = "suction_cup_link"
        ori_con.orientation.x = 0.0
        ori_con.orientation.y = 1.0
        ori_con.orientation.z = 0.0
        ori_con.orientation.w = 0.0
        ori_con.weight = 1.0
        ori_con.absolute_x_axis_tolerance = 0.1
        ori_con.absolute_y_axis_tolerance = 0.1
        ori_con.absolute_z_axis_tolerance = 0.1
        constraints.orientation_constraints.append(ori_con)

        req.goal_constraints.append(constraints)
        goal_msg.request = req
        
        self.get_logger().info(f"Moving suction_cup_link to: {x:.2f}, {y:.2f}, {z:.2f}")
        self.move_group_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    node = HCR5SuctionVisionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()