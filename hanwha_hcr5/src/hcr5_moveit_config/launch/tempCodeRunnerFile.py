import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class HCR5LinearPicker(Node):
    def __init__(self):
        super().__init__('hcr5_linear_picker')
        
        self.bridge = CvBridge()
        self.latest_coords = None
        self.target_color_mode = "PURPLE"

        # Calibration
        script_dir = os.path.dirname(os.path.realpath(__file__))
        self.M = np.load(os.path.join(script_dir, "table_calibration.npy"))

        # Color Ranges
        self.color_ranges = {
            "PURPLE": {"low": np.array([130, 50, 50]), "high": np.array([170, 255, 255]), "bgr": (255, 0, 255)},
            "BLUE": {"low": np.array([100, 50, 50]), "high": np.array([130, 255, 255]), "bgr": (255, 0, 0)}
        }

        # Services and Actions
        self.cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self.sub = self.create_subscription(Image, 'camera1/image_raw', self.image_callback, 10)
        
        self.get_logger().info("LINEAR MOTION MODE: SPACE to move in a straight line.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        current = self.color_ranges[self.target_color_mode]
        mask = cv2.inRange(hsv, current["low"], current["high"])
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 50:
                x, y, w, h = cv2.boundingRect(largest)
                u, v = x + (w // 2), y + (h // 2)
                if self.M is not None:
                    pixel_pt = np.array([u, v, 1], dtype="float32")
                    world_pt = np.dot(self.M, pixel_pt)
                    self.latest_coords = (world_pt[0] / world_pt[2], world_pt[1] / world_pt[2])
                cv2.rectangle(frame, (x, y), (x+w, y+h), current["bgr"], 2)

        cv2.imshow("Linear Vision", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('p'): self.target_color_mode = "PURPLE"
        elif key == ord('b'): self.target_color_mode = "BLUE"
        elif key == ord(' '): 
            if self.latest_coords:
                self.request_linear_move(self.latest_coords[0], self.latest_coords[1], 0.08)

    def request_linear_move(self, x, y, z):
        if not self.cartesian_client.service_is_ready():
            self.get_logger().error("Cartesian service not ready!")
            return

        # Prepare Cartesian Request
        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.group_name = "arm"
        req.link_name = "suction_cup_link"
        
        # Define waypoint
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        # Lock orientation to pointing down
        target_pose.orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)
        
        req.waypoints = [target_pose]
        req.max_step = 0.01  # 1cm resolution
        req.jump_threshold = 0.0 # Strict joint jump check
        req.avoid_collisions = True

        self.get_logger().info("Computing linear trajectory...")
        future = self.cartesian_client.call_async(req)
        future.add_done_callback(self.path_response_callback)

    def path_response_callback(self, future):
        res = future.result()
        if res.fraction < 1.0:
            self.get_logger().warn(f"Linear move only {res.fraction*100:.1f}% possible. Reach limit!")
            return

        self.get_logger().info("Linear path found. Executing...")
        # Send to Execution Action
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution.joint_trajectory
        
        self.execute_client.wait_for_server()
        self.execute_client.send_goal_async(goal)

def main():
    rclpy.init()
    node = HCR5LinearPicker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()