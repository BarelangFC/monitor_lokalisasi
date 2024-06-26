import rclpy
import math
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from bfc_msgs.msg import Button
from bfc_msgs.msg import Coordination
from bfc_msgs.msg import HeadMovement
from darknet_ros_msgs.msg import BoundingBoxes

class MyNode(Node):
    def __init__(self):
        super().__init__('ball_pose_node')
        self.create_subscription(Odometry, 'pose', self.pose_callback_robot, 1)
        self.create_subscription(Button, 'button', self.button_status_robot, 1)
        self.create_subscription(Bool, 'ball_status', self.ball_found_status, 1)
        self.create_subscription(HeadMovement, 'head', self.pan_tilt_radian, 1)
        self.create_subscription(Imu, 'imu', self.robot_imu, 1)
        self.publishers_ = self.create_publisher(Odometry, 'ball_pose', 1)
        self.publishers_dis = self.create_publisher(Float32, 'ball_distance', 1)
        self.create_timer(0.5, self.predict_ball)
        self.robot_PosX
        self.robot_PosY
        self.robot_kill
        self.robot_strategy
        self.robot_ball
        self.pan_radian
        self.tilt_radian
        self.imu_roll
        self.imu_pitch
        self.imu_yaw

    def pose_callback_robot(self, msg):
        self.robot_PosX = msg.pose.pose.position.x
        self.robot_PosY = msg.pose.pose.position.y

    def button_status_robot(self, msg):
        self.robot_kill = msg.kill
        self.robot_strategy = msg.strategy

    def robot_ball_found_status(self, msg):
        self.robot_ball = msg.found_ball

    def pan_tilt_radian(self, msg):
        self.pan_radian = msg.pan
        self.tilt_radian = msg.tilt

    def robot_imu(self, msg):
        self.imu_roll = msg.angular_velocity.x
        self.imu_pitch = msg.angular_velocity.y
        self.imu_yaw = msg.angular_velocity.z

    def predict_ball(self):
        yaw_robot = np.radians(self.imu_yaw * -1) # yaw_robot needs to be updated according to IMU data
        pan_head = self.pan_radian
        body_pitch = 0.261799
        tilt_head = (-self.tilt_radian - body_pitch) # Tilt no minus
        robot_body = 58

         #Validate yaw_robot value
        if yaw_robot < -180 or yaw_robot > 180:
            raise ValueError("Invalid yaw_robot value. It should be between -180 and 180.")

        theta =-(yaw_robot + pan_head)# theta calculation now accounts for yaw_robot value
        
        Logitech_tan = (robot_body * np.tan(tilt_head)) 
        Logitech_adjusted = (-0.0004 * Logitech_tan**2) + (0.7657 * Logitech_tan)

        A = np.array([[1, 0, self.robot_3_PosX + 450],
                     [0, 1,  self.robot_3_PosY + 300],
                     [0, 0, 1]])

        B = np.array([[np.cos(theta), -np.sin(theta), 0.0000],
                    [np.sin(theta), np.cos(theta), 0.0000],
                     [0.0000, 0.0000, 1.0000]])

        C = np.array([[1.0000, 0.0000, Logitech_adjusted],
                    [0.0000, 1.0000, 0.0000],
                   [0.0000, 0.0000, 1.0000]])
        
        D = A.dot(B).dot(C)
        # Extracting specific values from D
        self.ball_x = int(D[0, 2])  # First row, third column
        self.ball_y = int(D[1, 2])  # Second row, third column
        self.dBall = Logitech_adjusted
        
        msg_ball_pose = Odometry()
        msg_ball_dis = Float32()
        if self.robot_ball:
            msg_ball_pose.pose.pose.position.x = float(ball_coor_x)
            msg_ball_pose.pose.pose.position.y = float(ball_coor_y)
            msg_ball_pose.pose.pose.position.z = 0.0
            msg_ball_dis.data = self.dBall
        else:
            msg_ball_pose.pose.pose.position.x = -1.0
            msg_ball_pose.pose.pose.position.y = -1.0
            msg_ball_pose.pose.pose.position.z = 0.0
            msg_ball_dis.data = -1.0
        self.publishers_.publish(msg_ball_pose)
        self.publishers_dis.publish(msg_ball_dis)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()