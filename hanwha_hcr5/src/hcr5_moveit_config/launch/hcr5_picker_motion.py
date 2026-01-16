import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, MotionPlanRequest
from shape_msgs.msg import SolidPrimitive
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class HCR5SloMoPicker(Node):
    def __init__(self):
        super().__init__('hcr5_slomo_picker')
        
        # --- Settings ---
        self.TARGET_Z = 0.04
        self.bridge = CvBridge()
        self.latest_coords = None
        self.vacuum_state = False  # Track suction state
        
        # Load Calibration Matrix
        script_dir = os.path.dirname(os.path.realpath(__file__))
        file_path = os.path.join(script_dir, "table_calibration.npy")
        try:
            self.M = np.load(file_path)
            self.get_logger().info("Calibration loaded.")
        except Exception as e:
            self.get_logger().error(f"Calibration error: {e}")
            self.M = None

        # Publishers & Subscriptions
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.vacuum_pub = self.create_publisher(Bool, '/hcr5/vacuum_on', 10)
        self.sub = self.create_subscription(Image, 'camera1/image_raw', self.image_callback, 10)
        
        self.get_logger().info("=" * 40)
        self.get_logger().info("SLOW MOTION MODE ACTIVE")
        self.get_logger().info("SPACE : Move above cube (Slow)")
        self.get_logger().info("'f'   : Toggle Vacuum ON/OFF")
        self.get_logger().info("=" * 40)

    def image_callback(self, msg):
        if self.M is None: return
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, np.array([120, 40, 40]), np.array([175, 255, 255]))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        self.latest_coords = None
        for cnt in contours:
            if cv2.contourArea(cnt) > 150:
                M_moments = cv2.moments(cnt)
                if M_moments["m00"] != 0:
                    u = int(M_moments["m10"] / M_moments["m00"])
                    v = int(M_moments["m01"] / M_moments["m00"])
                    
                    pixel_pt = np.array([u, v, 1], dtype="float32")
                    world_pt = np.dot(self.M, pixel_pt)
                    self.latest_coords = (world_pt[0] / world_pt[2], world_pt[1] / world_pt[2])

                    cv2.circle(frame, (u, v), 8, (0, 255, 0), -1)
                    cv2.putText(frame, f"VACUUM: {'ON' if self.vacuum_state else 'OFF'}", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("Slow Picker Feed", frame)
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord(' '):
            if self.latest_coords:
                self.send_move_command(self.latest_coords[0], self.latest_coords[1], self.TARGET_Z)
        
        elif key == ord('f'):
            self.vacuum_state = not self.vacuum_state
            msg_vac = Bool()
            msg_vac.data = self.vacuum_state
            self.vacuum_pub.publish(msg_vac)
            self.get_logger().info(f"Vacuum toggled: {self.vacuum_state}")

    def send_move_command(self, x, y, z):
        goal_msg = MoveGroup.Goal()
        request = MotionPlanRequest()
        request.group_name = "arm"
        
        # --- SLOW MOTION SETTINGS ---
        request.max_velocity_scaling_factor = 0.7      # 10% of max speed
        request.max_acceleration_scaling_factor = 0.7  # 10% of max acceleration
        
        request.num_planning_attempts = 15
        request.allowed_planning_time = 5.0
        request.pipeline_id = "ompl"

        constraints = Constraints()
        
        # Position
        pos_con = PositionConstraint()
        pos_con.header.frame_id = "base_link"
        pos_con.link_name = "suction_cup_link"
        
        bounding_box = SolidPrimitive()
        bounding_box.type = SolidPrimitive.BOX
        bounding_box.dimensions = [0.005, 0.005, 0.005] 
        
        target_pose = PoseStamped()
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        
        pos_con.constraint_region.primitives.append(bounding_box)
        pos_con.constraint_region.primitive_poses.append(target_pose.pose)
        pos_con.weight = 1.0

        # Orientation (Wide tolerance to prevent planning failure)
        ori_con = OrientationConstraint()
        ori_con.header.frame_id = "base_link"
        ori_con.link_name = "suction_cup_link"
        ori_con.orientation.x = 0.0
        ori_con.orientation.y = 1.0 
        ori_con.orientation.z = 0.0
        ori_con.orientation.w = 0.0
        ori_con.absolute_x_axis_tolerance = 0.5
        ori_con.absolute_y_axis_tolerance = 0.5
        ori_con.absolute_z_axis_tolerance = 3.14
        ori_con.weight = 1.0

        constraints.position_constraints.append(pos_con)
        constraints.orientation_constraints.append(ori_con)
        
        request.goal_constraints.append(constraints)
        goal_msg.request = request

        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    node = HCR5SloMoPicker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()