import rclpy
import math
import numpy as np
# from sklearn.preprocessing import PolynomialFeatures
# from sklearn.linear_model import LinearRegression
# from sklearn.pipeline import make_pipeline
# from sklearn.metrics import mean_squared_error
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from bfc_msgs.msg import Button
from bfc_msgs.msg import Coordination
from bfc_msgs.msg import HeadMovement
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from darknet_ros_msgs.msg import BoundingBoxes

import cv2
import numpy as np
import time
import math

np.set_printoptions(precision=3, suppress=True)




class monitor_lokalisasi(Node):
    def __init__(self):
        super().__init__("monitor_lokalisasi_nodes")
        self.create_timer(0.5, self.timer_callback)
        self.create_timer(0.5, self.predict_ball)
        #self.publishers_ = self.create_publisher(Float32MultiArray, '/robot_1/distance_to_ball', 10)
        self.create_subscription(Odometry, '/robot_1/pose', self.pose_callback_robot_1, 10)
        #self.create_subscription(Odometry, '/robot_1/pose', self.pose_callback_robot_1_new, 10)
        self.create_subscription(Odometry, '/robot_2/pose', self.pose_callback_robot_2, 10)
        #self.create_subscription(Odometry, '/robot_2/pose', self.pose_callback_robot_2_new, 10)
        self.create_subscription(Odometry, '/robot_3/pose', self.pose_callback_robot_3, 10)
       # self.create_subscription(Odometry, '/robot_3/pose', self.pose_callback_robot_3_new, 10)
        self.create_subscription(Odometry, '/robot_4/pose', self.pose_callback_robot_4, 10)
       # self.create_subscription(Odometry, '/robot_4/pose', self.pose_callback_robot_4_new, 10)
        self.create_subscription(Odometry, '/robot_5/pose', self.pose_callback_robot_5, 10)

        self.create_subscription(Button, '/robot_1/button', self.button_status_robot_1, 10)
        self.create_subscription(Button, '/robot_2/button', self.button_status_robot_2, 10)
        self.create_subscription(Button, '/robot_3/button', self.button_status_robot_3, 10)
        self.create_subscription(Button, '/robot_4/button', self.button_status_robot_4, 10)
        self.create_subscription(Button, '/robot_5/button', self.button_status_robot_5, 10)


        self.create_subscription(Coordination, '/robot_1/coordination', self.robot_1_ball_status, 10)
        self.create_subscription(Coordination, '/robot_2/coordination', self.robot_2_ball_status, 10)
        self.create_subscription(Coordination, '/robot_3/coordination', self.robot_3_ball_status, 10)
        self.create_subscription(Coordination, '/robot_4/coordination', self.robot_4_ball_status, 10)
        self.create_subscription(Coordination, '/robot_5/coordination', self.robot_5_ball_status, 10)


        self.create_subscription(Bool, 'ball_status', self.ball_found_status, 10)
        self.create_subscription(HeadMovement, 'head', self.pan_tilt_radian, 10)
        self.create_subscription(Imu, 'imu', self.robot_imu, 10)
        self.publishers_ = self.create_publisher(Odometry, 'ball_pose', 1)
        self.publishers_dis = self.create_publisher(Float32, 'ball_distance', 1)


        #self.create_subscription(Float32,'/zed_camera/distance_to_person', self.predict_ball_location, 10)

        
        self.ball_found = True
        self.imu_yaw = 0
        self.robot_1_ball = 0
        self.robot_2_ball = 0
        self.robot_3_ball = 0
        self.robot_4_ball = 0
        self.robot_5_ball = 0
        
        self.robot_1_PosX = 0
        self.robot_1_PosY = 0
        self.robot_1_PosX_new = 0
        self.robot_1_PosY_new = 0

        self.robot_2_PosX = 0
        self.robot_2_PosY = 0
        self.robot_2_PosX_new = 0
        self.robot_2_PosY_new = 0

        self.robot_3_PosX = 0
        self.robot_3_PosY = 0
        self.robot_3_PosX_new = 0
        self.robot_3_PosY_new = 0

        self.robot_4_PosX = 0
        self.robot_4_PosY = 0
        self.robot_4_PosX_new = 0
        self.robot_4_PosY_new = 0

        self.robot_5_PosX = 0
        self.robot_5_PosY = 0

        self.robot_1_kill = 10
        self.robot_1_strategy = 10
        self.robot_2_kill = 10
        self.robot_2_strategy = 10
        self.robot_3_kill = 10      
        self.robot_3_strategy = 10
        self.robot_4_kill = 10
        self.robot_1_strategy = 10
        self.robot_4_strategy = 10
        self.robot_5_kill = 10
        self.robot_5_strategy = 10

        self.centre_x = 0
        self.centre_y = 0

        self.x_max = 0
        self.x_min = 0
        self.y_max = 0
        self.y_min = 0
        self.id = None
        self.class_id = ""   

        self.pan_radian = 0
        self.tilt_radian = 0    
        self.ball_distance = 0

        self.ball_x = 0
        self.ball_y = 0

        self.value_x = 0
        self.value_y = 0

        self.ball_detected = False

        self.imu_roll = 0
        self.imu_pitch = 0

        self.previous_time = 0

        self.estimated_distance = 0

        self.dBall = 0

        self.mapImage = np.zeros((800,1100,3), np.uint8)
        self.robotInitialPosition = np.zeros((3))

    def worldCoorToImageCoor(self, x, y):		
         x = x				
         y = y		
         return int(x), int(y)
    
    def button_status_robot_1(self, msg):
        self.robot_1_kill = msg.kill
        self.robot_1_strategy = msg.strategy

    def button_status_robot_2(self, msg):
        self.robot_2_kill = msg.kill
        self.robot_2_strategy = msg.strategy

    def button_status_robot_3(self, msg):
        self.robot_3_kill = msg.kill
        self.robot_3_strategy = msg.strategy

    def button_status_robot_4(self, msg):
        self.robot_4_kill = msg.kill
        self.robot_4_strategy = msg.strategy

    def button_status_robot_5(self, msg):
        self.robot_5_kill = msg.kill
        self.robot_5_strategy = msg.strategy

    def pose_callback_robot_1(self, msg):
        self.robot_1_PosX = msg.pose.pose.position.x
        self.robot_1_PosY = msg.pose.pose.position.y
        # print("coordinate x robot =", self.robot_1_PosX)
        # print("coordinate y robot =", self.robot_1_PosY)

    def pose_callback_robot_2(self, msg):
        self.robot_2_PosX = msg.pose.pose.position.x
        self.robot_2_PosY = msg.pose.pose.position.y

    def pose_callback_robot_3(self, msg):
        self.robot_3_PosX = msg.pose.pose.position.x
        self.robot_3_PosY = msg.pose.pose.position.y

    def pose_callback_robot_4(self, msg):
        self.robot_4_PosX = msg.pose.pose.position.x
        self.robot_4_PosY = msg.pose.pose.position.y

    def pose_callback_robot_5(self, msg):
        self.robot_5_PosX = msg.pose.pose.position.x
        self.robot_5_PosY = msg.pose.pose.position.y

    def pose_callback_robot_1_new(self, msg):
        self.robot_1_PosX_new = msg.pose.pose.position.x
        self.robot_1_PosY_new = msg.pose.pose.position.y

    def pose_callback_robot_2_new(self, msg):
        self.robot_2_PosX_new = msg.pose.pose.position.x
        self.robot_2_PosY_new = msg.pose.pose.position.y

    def pose_callback_robot_3_new(self, msg):
        self.robot_3_PosX_new = msg.pose.pose.position.x
        self.robot_3_PosY_new = msg.pose.pose.position.y

    def pose_callback_robot_4_new(self, msg):
        self.robot_4_PosX_new = msg.pose.pose.position.x
        self.robot_4_PosY_new = msg.pose.pose.position.y

    def robot_1_ball_status(self, msg):
        self.robot_1_ball = msg.found_ball

    def robot_2_ball_status(self, msg):
        self.robot_2_ball = msg.found_ball

    def robot_3_ball_status(self, msg):
        self.robot_3_ball = msg.found_ball

    def robot_4_ball_status(self, msg):
        self.robot_4_ball = msg.found_ball

    def robot_5_ball_status(self, msg):
        self.robot_5_ball = msg.found_ball

    def ball_jarak(self, msg):
        self.ball_distance = msg.data * 100

    def ball_found_status(self, msg):
        self.ball_found = msg.data
            
    def robot_imu(self, msg):
        self.imu_roll = msg.angular_velocity.x
        self.imu_pitch = msg.angular_velocity.y
        self.imu_yaw = msg.angular_velocity.z

    def pan_tilt_radian(self, msg):
        self.pan_radian = msg.pan
        self.tilt_radian = msg.tilt


    def estimated_distance_ball(self, msg):
        self.estimated_distance = msg.data


    def predict_ball(self):
        yaw_robot = np.radians(self.imu_yaw * -1) # yaw_robot needs to be updated according to IMU data
        pan_head = self.pan_radian
        body_pitch = 0.261799
        tilt_head = (-self.tilt_radian - body_pitch) # Tilt no minus
        robot_body = 58

        # yaw_robot = np.radians(0) # yaw_robot needs to be updated according to IMU data
        # pan_head = 0.1
        # body_pitch = 0.261799
        # tilt_head = 1.39 - body_pitch # Tilt no minus
        # robot_body = 58

         #Validate yaw_robot value
        if yaw_robot < -180 or yaw_robot > 180:
            raise ValueError("Invalid yaw_robot value. It should be between -180 and 180.")

        theta =-(yaw_robot + pan_head)# theta calculation now accounts for yaw_robot value
        
        Logitech_tan = (robot_body * np.tan(tilt_head)) 
        Logitech_adjusted = (-0.0004 * Logitech_tan**2) + (0.7657 * Logitech_tan)
        #Logitech = np.sin(tilt_head) * Logitech_tan

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
        #print("Array D adalah ", D)
        #print("Matri A ", A)
        # print("Coordinate Ball X adalah = ", self.ball_x)
        # print("Coordinate Ball Y adalahc= ", self.ball_y)
        # print("Hasil Jarak = ", Logitech_adjusted)
        #print(pan_head)


    # def predict_ball(self):
    #     # Your measured data
    #     tilts = np.array([0.56, 0.92 , 1.14, 1.30, 1.36, 1.39, 1.50, 1.50, 1.53, 1.65, 1.68, 1.73, 1.73 , 1.77,1.90, 0.80, 0.86  , 1.06 , 1.23, 1.31  , 1.33  , 1.46 ,1.42   , 1.45 , 1.52])
    #     pans = np.array([0.03, 0.050, 0.04, 0.01, 0.02, 0.02, 0.07, 0.06, 0.07, 0.07, 0.08, 0.04, 0.11 , 0.12,0.08, -0.6 ,-0.68 , -1.00,-1.15, -1.23 , -1.26 , -1.33, -1.33 ,-1.36, -1.36])
    #     distances = np.array([20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240, 260, 280, 300, 30, 50, 66, 90, 120, 150, 180, 210, 240, 270])

    #     # Combine your inputs into one array
    #     X = np.vstack((tilts, pans)).T

    #     # Fit a polynomial regression model to the data
    #     degree = 3
    #     model = make_pipeline(PolynomialFeatures(degree), LinearRegression())
    #     model.fit(X, distances)

    #     # Now, given a tilt and pan value, we can find the estimated distance
    #     tilt = self.tilt_radian * -1
    #     pan = self.pan_radian
    #     yaw_robot = np.radians(self.imu_yaw * -1) # yaw_robot needs to be updated according to IMU data
    #     estimated_distance = model.predict([[tilt, pan]])
    #     print(f"Estimated distance: {estimated_distance[0]} cm")

    #     # Create a grid of points in the range of the data and evaluate the model at each point
    #     tilt_range = np.linspace(tilts.min(), tilts.max(), num=100)
    #     pan_range = np.linspace(pans.min(), pans.max(), num=100)
    #     tilt_grid, pan_grid = np.meshgrid(tilt_range, pan_range)
    #     X_grid = np.vstack((tilt_grid.ravel(), pan_grid.ravel())).T
    #     distances_grid = model.predict(X_grid).reshape(tilt_grid.shape)

    #     # Validate yaw_robot value
    #     if yaw_robot < -180 or yaw_robot > 180:
    #         raise ValueError("Invalid yaw_robot value. It should be between -180 and 180.")

    #     theta =-(yaw_robot + pan)# theta calculation now accounts for yaw_robot value
        
    #     #Logitech_tan = (robot_body * np.tan(tilt_head) )
    #         #Logitech = np.sin(tilt_head) * Logitech_tan

    #     A = np.array([[1, 0, self.robot_1_PosX + 450],
    #                 [0, 1,  self.robot_1_PosY + 300],
    #                 [0, 0, 1]])

    #     B = np.array([[np.cos(theta), -np.sin(theta), 0.0000],
    #                 [np.sin(theta), np.cos(theta), 0.0000],
    #                 [0.0000, 0.0000, 1.0000]])

    #     C = np.array([[1.0000, 0.0000, estimated_distance[0]],
    #                 [0.0000, 1.0000, 0.0000],
    #               [0.0000, 0.0000, 1.0000]])
        
    #     D = A.dot(B).dot(C)
    #     # Extracting specific values from D
    #     self.ball_x = int(D[0, 2])  # First row, third column
    #     self.ball_y = int(D[1, 2])  # Second row, third column




    def timer_callback(self):
        self.mapImage[:] = (0, 255, 0)
        # cv2.rectangle(self.mapImage, (100,100), (1000,700), (255,255,255), 3) #. Garis Luar
        # cv2.rectangle(self.mapImage, (40,530), (100,270),(255,255,255), 3) # Garis Luar Gawang Kiri
        # cv2.rectangle(self.mapImage, (1000,530), (1060,270), (255,255,255), 3) # Garis Luar Gawang Kiri
        # cv2.rectangle(self.mapImage, (100,650), (200,150), (255,255,255), 3) # Garis Luar Gawang Kiri
        # cv2.rectangle(self.mapImage, (900,650), (1000,150), (255,255,255), 3) # Garis Luar Gawang Kiri
        # cv2.line(self.mapImage, (550,100), (550,700), (255,255,255), 3) # Garis Tengah
        # cv2.circle(self.mapImage, (550,400), 75, (255,255,255), 3) # Lingkaran Tengah
        # cv2.circle(self.mapImage, (250,400), 3, (255,255,255), 5)
        # cv2.circle(self.mapImage, (850,400), 3, (255,255,255), 5)
        # cv2.line(self.mapImage, (100,200), (1000,200), (0,0,0), 1)
        # cv2.line(self.mapImage, (100,300), (1000,300), (0,0,0), 1)
        # cv2.line(self.mapImage, (100,400), (1000,400), (0,0,0), 1)
        # cv2.line(self.mapImage, (100,500), (1000,500), (0,0,0), 1)
        # cv2.line(self.mapImage, (100,600), (1000,600), (0,0,0), 1)
                            
        # cv2.line(self.mapImage, (200,100), (200,700), (0,0,0), 1)
        # cv2.line(self.mapImage, (300,100), (300,700), (0,0,0), 1)
        # cv2.line(self.mapImage, (400,100), (400,700), (0,0,0), 1)
        # cv2.line(self.mapImage, (500,100), (500,700), (0,0,0), 1)
        # cv2.line(self.mapImage, (600,100), (600,700), (0,0,0), 1)
        # cv2.line(self.mapImage, (700,100), (700,700), (0,0,0), 1)
        # cv2.line(self.mapImage, (800,100), (800,700), (0,0,0), 1)
        # cv2.line(self.mapImage, (900,100), (900,700), (0,0,0), 1)
        x_1, y_1 = self.worldCoorToImageCoor(self.robot_1_PosX+450, self.robot_1_PosY+300)
        x_2, y_2 = self.worldCoorToImageCoor(self.robot_2_PosX+450, self.robot_2_PosY+300)
        x_3, y_3 = self.worldCoorToImageCoor(self.robot_3_PosX+450, self.robot_3_PosY+300)
        x_4, y_4 = self.worldCoorToImageCoor(self.robot_4_PosX+450, self.robot_4_PosY+300)         
        x_5, y_5 = self.worldCoorToImageCoor(self.robot_5_PosX+450, self.robot_5_PosY+300)
        ball_coor_x, ball_coor_y = (self.ball_x, self.ball_y)
        # tempat publish
        msg_ball_pose = Odometry()
        msg_ball_dis = Float32()
        if self.robot_3_ball == 1:
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
        

########################## STATUS STRATEGI ROBOT 1 ##########################       
        robot_1_text = "ROBOT 1 :"
        robot_1_condition = self.robot_1_strategy

        if robot_1_condition == 0:
            robot_1_status = "GK"
        elif robot_1_condition == 1:
            robot_1_status = "ATTACK"
        elif robot_1_condition == 2:
            robot_1_status = "DEF.R"
        elif robot_1_condition == 3:
            robot_1_status = "DEF.L"
        else:
            robot_1_status = "BINGUNG"

########################## STATUS ROBOT 2 ##########################       
        robot_2_text = "ROBOT 2 :"
        robot_2_condition = self.robot_2_strategy

        if robot_2_condition == 0:
            robot_2_status = "GK"
        elif robot_2_condition == 1:
            robot_2_status = "ATTACK"
        elif robot_2_condition == 2:
            robot_2_status = "DEF.R"
        elif robot_2_condition == 3:
            robot_2_status = "DEF.L"
        else:
            robot_2_status = "BINGUNG"

########################## STATUS ROBOT 3 ##########################       
        robot_3_text = "ROBOT 3 :"
        robot_3_condition = self.robot_3_strategy

        if robot_3_condition == 0:
            robot_3_status = "GK"
        elif robot_3_condition == 1:
            robot_3_status = "ATTACK"
        elif robot_3_condition == 2:
            robot_3_status = "DEF.R"
        elif robot_3_condition == 3:
            robot_3_status = "DEF.L"
        else:
            robot_3_status = "BINGUNG"

########################## STATUS ROBOT 4 ##########################       
        robot_4_text = "ROBOT 4 :"
        robot_4_condition = self.robot_4_strategy

        if robot_4_condition == 0:
            robot_4_status = "GK"
        elif robot_4_condition == 1:
            robot_4_status = "ATTACK"
        elif robot_4_condition == 2:
            robot_4_status = "DEF.R"
        elif robot_4_condition == 3:
            robot_4_status = "DEF.L"
        else:
            robot_4_status = "BINGUNG"
########################## STATUS ROBOT 5 ##########################       
        robot_5_text = "ROBOT 5 :"
        robot_5_condition = self.robot_5_strategy

        if robot_5_condition == 0:
            robot_5_status = "GK"
        elif robot_5_condition == 1:
            robot_5_status = "ATTACK"
        elif robot_5_condition == 2:
            robot_5_status = "DEF.R"
        elif robot_5_condition == 3:
            robot_5_status = "DEF.L"
        else:
            robot_5_status = "BINGUNG"


########################### DARI SINI JANGAN DISENTUH/DIUBAH ##################################################
        # font = cv2.FONT_HERSHEY_SIMPLEX
        # fontScale = 0.65
        # color = (0, 0, 0)
        # thickness = 2
        # robot_1_text_size = cv2.getTextSize(robot_1_text, font, fontScale, thickness)
        # robot_1_condition_size = cv2.getTextSize(robot_1_status, font, fontScale, thickness)
        
        # robot_2_text_size = cv2.getTextSize(robot_2_text, font, fontScale, thickness)
        # robot_2_condition_size = cv2.getTextSize(robot_2_status, font, fontScale, thickness)

        # robot_3_text_size = cv2.getTextSize(robot_3_text, font, fontScale, thickness)
        # robot_3_condition_size = cv2.getTextSize(robot_3_status, font, fontScale, thickness)

        # robot_4_text_size = cv2.getTextSize(robot_4_text, font, fontScale, thickness)
        # robot_4_condition_size = cv2.getTextSize(robot_4_status, font, fontScale, thickness)

        # robot_5_text_size = cv2.getTextSize(robot_5_text, font, fontScale, thickness)
        # robot_5_condition_size = cv2.getTextSize(robot_5_status, font, fontScale, thickness)
        # robot_1_x_text_coordinate = 10
        # robot_1_y_text_coordinate = robot_1_text_size[0][1] + 10
        # robot_1_x_condition = 125
        # robot_1_y_condition = robot_1_text_size[0][1] + 10

        # robot_2_x_text_coordinate = 220
        # robot_2_y_text_coordinate = robot_2_text_size[0][1] + 10
        # robot_2_x_condition = 335
        # robot_2_y_condition = robot_2_text_size[0][1] + 10

        # robot_3_x_text_coordinate = 220*2
        # robot_3_y_text_coordinate = robot_3_text_size[0][1] + 10
        # robot_3_x_condition = 275*2 + 5
        # robot_3_y_condition = robot_3_text_size[0][1] + 10

        # robot_4_x_text_coordinate = 220*3
        # robot_4_y_text_coordinate = robot_4_text_size[0][1] + 10
        # robot_4_x_condition = 775
        # robot_4_y_condition = robot_4_text_size[0][1] + 10

        # robot_5_x_text_coordinate = 220*4
        # robot_5_y_text_coordinate = robot_5_text_size[0][1] + 10
        # robot_5_x_condition = 995
        # robot_5_y_condition = robot_5_text_size[0][1] + 10
        # cv2.circle(self.mapImage, (x_1 + 100, y_1 + 100), 3, (255,0,0), 5)
        # print("posisi robot 1 " + str(x_1) + " " + str(y_1))

        #cv2.circle(self.mapImage, (x_2 + 100, y_2 + 100), 3, (55,55,55), 5)
        # print("posisi robot 2 " + str(x_2) + " " + str(y_2))

        #cv2.circle(self.mapImage, (x_3 + 100, y_3 + 100), 3, (100,100,100), 5)
        # print("posisi robot 3 " + str(x_3) + " " + str(y_3))

        # cv2.circle(self.mapImage, (x_4 + 100, y_4 + 100), 3, (125,125,125), 5)
        # print("posisi robot 4 " + str(x_4) + " " + str(y_4))

        #cv2.circle(self.mapImage, (x_5 + 100, y_5 + 100), 3, (175,175,175), 5)
        # print("posisi robot 5 " + str(x_5) + " " + str(y_5))

        #if self.ball_found == True: 
        # cv2.circle(self.mapImage, (ball_coor_x + 100, ball_coor_y + 100), 3, (0,0,255), 10)
            
        # cv2.putText(self.mapImage, robot_1_text, (robot_1_x_text_coordinate, robot_1_y_text_coordinate), font, fontScale, color, thickness, cv2.LINE_AA)
        # cv2.putText(self.mapImage, robot_1_status, (robot_1_x_condition, robot_1_y_condition), font, fontScale, color, thickness, cv2.LINE_AA)

        # cv2.putText(self.mapImage, robot_2_text, (robot_2_x_text_coordinate, robot_2_y_text_coordinate), font, fontScale, color, thickness, cv2.LINE_AA)
        # cv2.putText(self.mapImage, robot_2_status, (robot_2_x_condition, robot_2_y_condition), font, fontScale, color, thickness, cv2.LINE_AA)

        # cv2.putText(self.mapImage, robot_3_text, (robot_3_x_text_coordinate, robot_3_y_text_coordinate), font, fontScale, color, thickness, cv2.LINE_AA)
        # cv2.putText(self.mapImage, robot_3_status, (robot_3_x_condition, robot_3_y_condition), font, fontScale, color, thickness, cv2.LINE_AA)

        # cv2.putText(self.mapImage, robot_4_text, (robot_4_x_text_coordinate, robot_4_y_text_coordinate), font, fontScale, color, thickness, cv2.LINE_AA)
        # cv2.putText(self.mapImage, robot_4_status, (robot_4_x_condition, robot_4_y_condition), font, fontScale, color, thickness, cv2.LINE_AA)

        # cv2.putText(self.mapImage, robot_5_text, (robot_5_x_text_coordinate, robot_5_y_text_coordinate), font, fontScale, color, thickness, cv2.LINE_AA)
        # cv2.putText(self.mapImage, robot_5_status, (robot_5_x_condition, robot_5_y_condition), font, fontScale, color, thickness, cv2.LINE_AA)
############################################################### DIATAS JANGAN DISENTUH/DIUBAH #######################################################################

        # smallMapImage = cv2.resize(self.mapImage, (1280,720), interpolation = cv2.INTER_AREA)
        # cv2.imshow("Barelang Localization", smallMapImage)
        # cv2.waitKey(1)

        

def main(args=None):
    rclpy.init(args=args)
    node = monitor_lokalisasi()
    rclpy.spin(node)
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
