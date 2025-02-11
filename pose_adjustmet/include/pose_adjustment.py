#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden
# 人体跟踪
import os
import sys
import cv2
import rospy
#import tf2_ros
import math
import queue
import rospy
import signal
import math
import time 
import numpy as np
import faulthandler
import jetauto_sdk.pid as pid
import jetauto_sdk.fps as fps
from sensor_msgs.msg import Image, LaserScan
from std_srvs.srv import Trigger
from std_msgs.msg import Float64
from nav_msgs.msg import OccupancyGrid, Path


import subprocess
import jetauto_sdk.misc as misc
from geometry_msgs.msg import Twist
from jetauto_interfaces.msg import ObjectsInfo
sys.path.append('/home/jetauto/jetauto_software/arm_pc')
from action_group_controller import ActionGroupController
from datetime import datetime

faulthandler.enable()

# Get the Python version
version = sys.version
version_info = sys.version_info

print("Python Version:", version)
print("Version Info:", version_info)

MAX_SCAN_ANGLE = 240 # the scanning angle of lidar. The covered part is always eliminated
image_detect_folder = "/home/jetauto/jetauto_ws/src/jetauto_example/scripts/body_control/include/detected_objects/"
label_folder = "/home/jetauto/jetauto_ws/src/jetauto_example/scripts/body_control/include/labels/"

class Defect:
    id_counter = 0  # Class-level counter for unique IDs

    def __init__(self, obj, x, y):
        Defect.id_counter += 1
        self.id = Defect.id_counter
        self.defect = obj
        self.type = obj.class_name if hasattr(obj, 'class_name') else None
        self.location = (x, y)
        self.horizontal_area = []
        self.confidence = []  # List to store last 3 confidence scores
        self.img = []
        self.large = False # if edges doesnt fit both sides
        self.recorded = True


class PoseAdjustmentNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
       
        self.pid_d = pid.PID(0.1, 0, 0)
        #self.pid_d = pid.PID(0, 0, 0)
        
        self.pid_angular = pid.PID(0.002, 0, 0)
        #self.pid_angular = pid.PID(0, 0, 0)
        
        self.go_speed, self.turn_speed = 0.007, 0.04
        self.linear_x, self.angular = 0, 0

        self.running = True
        self.fps = fps.FPS()  # fps计算器
        signal.signal(signal.SIGINT, self.shutdown)
        self.image_queue = queue.Queue(maxsize=2)
        self.camera_intrinsics = {
            'fx': 473.4507,  # Focal length in x-direction (pixels)
            'fy': 474.2169,  # Focal length in y-direction (pixels)
            'cx': 323.5512,  # Principal point x-coordinate (pixels)
            'cy': 238.6016   # Principal point y-coordinate (pixels)
        }


        # Initialize variables for sweeping
        self.position = Float64()
        self.current_angle = -60  # Start angle
        self.sweep_direction = 1  # 1 for increasing, -1 for decreasing
        self.turn_left = False
        self.turn_right = False
        self.go_forward = False
        self.move_x = 0
        self.move_y = 0
        self.check_counter = 0 
        self.back = False
        self.stop = False
        self.response_time = time.time()
        self.lidar_response = time.time()
        self.stop_threshold = 0.15
        self.scan_angle = math.radians(45)
        self.count = 0
        self.distance = 0
        self.next_frame = True
        self.depth_frame = None
        self.center = None
        self.lidar_data = None
        self.detected_objects = []
        self.defects = []
        self.machine_type = os.environ.get('MACHINE_TYPE')
        self.lidar_sub = None
        self.lidar_type = os.environ.get('LIDAR_TYPE')
        camera = rospy.get_param('/depth_camera/camera_name', 'depth_cam')
        self.image_sub = rospy.Subscriber('/yolov5/object_image', Image, self.image_callback, queue_size=1)
        self.depth_image_sub = rospy.Subscriber('/%s/depth/image_raw' % camera, Image, self.depth_image_callback, queue_size=1)
        self.mecanum_pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=1)
        self.cam_servo_pub = rospy.Publisher('/joint1_controller/command', Float64, queue_size=10)

        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)  # subscribe to Lidar

        # SLAM and Map Handling
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.path_sub = rospy.Subscriber("/robot_path", Path)

        #self.processed_scan_pub = rospy.Publisher('/processed_scan', LaserScan, queue_size=10)


        # self.tf_listener = tf.TransformListener()

        self.controller = ActionGroupController(use_ros=True)
        rospy.Subscriber('/yolov5/object_detect', ObjectsInfo, self.get_object_callback)
        while not rospy.is_shutdown():
            try:
                if rospy.get_param('/yolov5/init_finish'):
                    break
            except:
                rospy.sleep(0.1)
        rospy.ServiceProxy('/yolov5/start', Trigger)()
        rospy.sleep(1)
        self.controller.runAction('camera_up')
        self.mecanum_pub.publish(Twist())
        rospy.set_param('~init_finish', True)
        self.run()
	
    def shutdown(self, signum, frame):
        self.running = False
        if self.lidar_sub is not None:
                self.lidar_sub.unregister()
        rospy.loginfo('shutdown')

    def path_callback(self, path_msg):
        # Process the received Path message
        # rospy.loginfo(f"Received path with {len(path_msg.poses)} poses")
        
        # Example: Accessing the latest pose in the path
        if path_msg.poses:
            latest_pose = path_msg.poses[-1]
            rospy.loginfo(f"Latest pose: x{latest_pose.pose.position.x}, y{latest_pose.pose.position.y}")

    def map_callback(self, map_data):
        # rospy.loginfo("Map data received")
        self.current_map = map_data
        # Example: Retrieve the robot's pose relative to the map


    # 获取目标检测结果
    def get_object_callback(self, msg):
        detected_objects = []
        for i in msg.objects:

            class_name = i.class_name
            if class_name == 'crack':

                box = i.box
                roi = self.depth_frame[box[1]:box[3], box[0]:box[2]]
                try:
                    distances = roi[np.logical_and(roi > 0, roi < 40000)]
                    avg_distance = int(np.mean(distances)/10)
                except:
                    avg_distance = 0 

                
                if i.box[1] < 10:
                    center = [int((i.box[0] + i.box[2])/2), int(i.box[1]) + abs(int((i.box[1] - i.box[3])/4))]
                else:
                    center = [int((i.box[0] + i.box[2])/2), int(i.box[1]) + abs(int((i.box[1] - i.box[3])/3))]
                detected_objects.append((i, center, avg_distance))
        detected_objects.sort(key=lambda x: x[2])  # Sort by avg_distance
        self.response_time = time.time()
        if self.detected_objects:
                # Focus on the closest detected object
                _, self.center, _ = self.detected_objects[0]
        self.detected_objects = detected_objects


    def lidar_callback(self, lidar_data):
        self.lidar_data = lidar_data
        twist = Twist()
        STOP_THRESHOLD = self.stop_threshold  # Threshold distance to stop
        # 数据大小 = 扫描角度/每扫描一次增加的角度(data size= scanning angle/ the increased angle per scan)
        if self.lidar_type != 'G4':
            max_index = int(math.radians(MAX_SCAN_ANGLE / 2.0) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[:max_index]  # 左半边数据(left data)
            right_ranges = lidar_data.ranges[::-1][:max_index]  # 右半边数据(right data)
        elif self.lidar_type == 'G4':
            min_index = int(math.radians((360 - MAX_SCAN_ANGLE) / 2.0) / lidar_data.angle_increment)
            max_index = int(math.radians(180) / lidar_data.angle_increment)
            left_ranges = lidar_data.ranges[::-1][min_index:max_index][::-1]  # 左半边数据(left data)
            right_ranges = lidar_data.ranges[min_index:max_index][::-1]  # 右半边数据(right data)

        # 根据设定取数据(Get data according to settings)
        angle = self.scan_angle / 2
        angle_index = int(angle / lidar_data.angle_increment + 0.50)
        
        front_left_range, front_right_range = np.array(left_ranges[:angle_index]), np.array(right_ranges[:angle_index])
        left_range, right_range = np.array(left_ranges[angle_index:]), np.array(right_ranges[angle_index:]) 

        front_left_nonzero = front_left_range.nonzero()
        front_right_nonzero = front_right_range.nonzero()
        left_nonzero = left_range.nonzero()
        right_nonzero = right_range.nonzero()
        # 取左右最近的距离(get the shortest distance left and right)
        min_dist_front_left = front_left_range[front_left_nonzero].min() if front_left_range.any() else float('inf')
        min_dist_front_right = front_right_range[front_right_nonzero].min() if front_right_range.any() else float('inf')
        min_dist_left = left_range[left_nonzero].min() if left_range.any() else float('inf')
        min_dist_right = right_range[right_nonzero].min() if right_range.any() else float('inf')

        # Set stop signals based on thresholds
        self.right_stop = min_dist_right < STOP_THRESHOLD
        self.left_stop = min_dist_left < STOP_THRESHOLD
        self.front_stop = (min_dist_front_right < STOP_THRESHOLD or min_dist_front_left < STOP_THRESHOLD)


        # Optional: Debugging output
        """
        print(f"Right min distance: {min_dist_right} | Stop: {self.right_stop}")
        print(f"Front-right min distance: {min_dist_front_right}")
        print(f"Front-left min distance: {min_dist_front_left}")
        print(f"Left min distance: {min_dist_left} | Stop: {self.left_stop}")
        print(f"Front stop: {self.front_stop}")
        """

        # Handle overall stop signal
        if self.front_stop:
            self.stop = True
            self.lidar_response = time.time()
            print("Front side is too close")
            self.move_robot(twist)
        elif self.right_stop:
            self.stop = True
            print("Right side is too close")
            self.lidar_response = time.time()
            print("Front side is too close")
            self.move_robot(twist)
        elif self.left_stop:
            self.stop = True
            print("Left side is too close")
            self.lidar_response = time.time()
            print("Front side is too close")
            self.move_robot(twist)
        else:
            self.count += 1
            if self.count > 5:
                self.count = 0
            self.stop = False

        #self.processed_scan_pub.publish(lidar_data)


    def depth_image_callback(self, depth_image):
        self.depth_frame = np.ndarray(shape=(depth_image.height, depth_image.width), dtype=np.uint16, buffer=depth_image.data)

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)  # 将自定义图像消息转化为图像(convert the custom image message into image)  into)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(rgb_image)
 

    def calculate_object_angle(self, center, image_width, camera_fov=60):
        """
        Calculate the angle of the detected object relative to the camera's center.

        :param center: Tuple (x, y) representing the center of the detected object in pixels.
        :param image_width: Width of the image in pixels.
        :param camera_fov: Horizontal Field of View (FOV) of the camera in degrees.
        :return: Angle in degrees relative to the camera's center.
        """
        # Camera center in pixels
        frame_center_x = image_width / 2
        # Offset in pixels
        offset_pixels = center[0] - frame_center_x

        # Map the pixel offset to the angle
        angle = (offset_pixels / frame_center_x) * (camera_fov / 2)
        rospy.loginfo(f"angle: {angle}")
        return angle

    
    def sweep_camera(self):
        start_time = time.time()
        
        # Publish the current camera position

        self.current_angle += self.sweep_direction * 60  # Increment by 5 degrees
        self.position.data = math.radians(self.current_angle)
        self.cam_servo_pub.publish(self.position)
        rospy.sleep(0.1) 
        # Adjust the current angle for the next sweep

        if self.current_angle >= 60:
            self.sweep_direction = -1  # Reverse direction at 45 degrees
        elif self.current_angle <= -60:
            self.sweep_direction = 1  # Reverse direction at -45 degrees

    def navigate(self, twist):
        
        # Move the robot forward
        twist.linear.x = 0.1  # Set forward speed
        self.mecanum_pub.publish(twist)
        rospy.sleep(0.1) 
        twist.linear.x = 0
        self.mecanum_pub.publish(twist)
        self.move_x += 1  

    def image_proc(self, bgr_image):
        # bgr_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        twist = Twist()
        if self.center is not None:
            h, w = bgr_image.shape[:-1]
            self.center = [int(self.center[0]), int(self.center[1])]
            cv2.circle(bgr_image, tuple(self.center), 10, (0, 255, 255), -1) 
            #################
            roi_h, roi_w = 5, 5
            w_1 = self.center[0] - roi_w
            w_2 = self.center[0] + roi_w
            if w_1 < 0:
                w_1 = 0
            if w_2 > w:
                w_2 = w
            h_1 = self.center[1] - roi_h
            h_2 = self.center[1] + roi_h
            if h_1 < 0:
                h_1 = 0
            if h_2 > h:
                h_2 = h
            
            cv2.rectangle(bgr_image, (w_1, h_1), (w_2, h_2), (0, 255, 255), 2)
            roi = self.depth_frame[h_1:h_2, w_1:w_2]
            distances = roi[np.logical_and(roi > 0, roi < 40000)]
            if distances != []:
                self.distance = int(np.mean(distances)/10)
            else:
                self.distance = 0
            if self.distance > 600: 
                self.distance = 600
            elif self.distance < 60:
                self.distance = 60
            
            if abs(self.distance - 150) < 15:
                self.distance = 150
            
            if abs(self.center[0] - w/2) < 30:
                self.center[0] = w/2

            #self.mecanum_pub.publish(twist)
            # print(self.distance, self.center)
        # self.mecanum_pub.publish(twist)
        
        return bgr_image

    def move_robot(self, twist, linear_x=0.0, linear_y=0.0, angular_z=0.0):
        """
        Move the robot with the specified linear and angular velocities.
        Resets the twist after movement.
        """
        rospy.loginfo(f"--------Robot_move-----")
        if not self.stop:
            lider_response_time = time.time() - self.lidar_response
            rospy.loginfo(f"---------lidar_response_time: {lider_response_time}-----")
            twist.linear.x = linear_x
            twist.linear.y = linear_y
            twist.angular.z = angular_z
            self.mecanum_pub.publish(twist)
            rospy.sleep(0.1)  # Short delay for movement to take effect
            self.move_x += 1
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.mecanum_pub.publish(twist)
        else:
            if self.front_stop:
                twist.linear.x = -0.2
                self.mecanum_pub.publish(twist)
                lidar_response_time = time.time() - self.lidar_response
                rospy.loginfo(f"lider_response_time: {lidar_response_time}")
                rospy.sleep(0.1) 
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.mecanum_pub.publish(twist)
            elif self.left_stop:

                twist.linear.y = 0.2
                self.mecanum_pub.publish(twist)
                lidar_response_time = time.time() - self.lidar_response
                rospy.loginfo(f"lider_response_time: {lidar_response_time}")
                rospy.sleep(0.1) 
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.mecanum_pub.publish(twist)
            else:

                twist.linear.y = -0.2
                self.mecanum_pub.publish(twist)
                lidar_response_time = time.time() - self.lidar_response
                rospy.loginfo(f"lider_response_time: {lidar_response_time}")
                rospy.sleep(0.1) 
                twist.linear.x = 0.0
                twist.linear.y = 0.0
                twist.angular.z = 0.0
                self.mecanum_pub.publish(twist)

    def process_and_approach_object(self, detected_object, frame, twist, direction, distance, confidence_threshold=0.3, edge_margin=30, center_tolerance=10):
        """
        Centers the object in the frame and gets closer to increase confidence.

        Parameters:
            detected_object (object): Detected object with bounding box and score attributes.
            frame (numpy array): Current frame (used to calculate frame dimensions).
            twist (Twist): Twist object for robot movement.
            direction (int): Direction multiplier (-1 for left, 1 for right).
            confidence_threshold (float): Minimum confidence to proceed with adjustments.
            edge_margin (int): Minimum margin from the frame edges for the bounding box.
            center_tolerance (int): Tolerance in pixels for centering the object.

        Returns:
            None
        """
        rospy.loginfo(detected_object)
        self.check_and_create_defect(detected_object, self.move_x, self.move_y*direction + distance, frame, distance)
        # Extract bounding box and confidence
        rospy.loginfo('checked records')
        bbox = detected_object.box  # Assuming this is a tuple (x_min, y_min, x_max, y_max)
        confidence = detected_object.score  # Confidence score of the detected object

        if confidence < confidence_threshold:
            rospy.loginfo("Object confidence too low. Halting adjustments.")
            return

        # Get frame dimensions
        frame_height, frame_width = frame.shape[:2]

        # Extract bounding box edges and center
        x_min, y_min, x_max, y_max = bbox
        bbox_center_x = (x_min + x_max) // 2
        bbox_center_y = (y_min + y_max) // 2

        # Calculate frame center
        frame_center_x = frame_width // 2
        frame_center_y = frame_height // 2

        # Calculate offsets for centering
        offset_x = bbox_center_x - frame_center_x
        offset_y = bbox_center_y - frame_center_y

        rospy.loginfo(f"Offsets - Horizontal: {offset_x}, Vertical: {offset_y}")

        # Adjust horizontal position to center the object
        if abs(offset_x) > center_tolerance:
            if offset_x > 0:
                self.move_x -= direction
                rospy.loginfo("Adjusting left to center object horizontally.")
                twist.linear.x = direction * (-0.1)  # Move left
            else:
                self.move_x += direction
                rospy.loginfo("Adjusting right to center object horizontally.")
                twist.linear.x = direction * (0.1)  # Move right
        else:
            rospy.loginfo("Horizontal alignment is optimal.")
            twist.linear.x = 0

        self.mecanum_pub.publish(twist)
        rospy.sleep(0.2)
        twist.linear.x = 0  # Stop horizontal movement
        self.mecanum_pub.publish(twist)
        rospy.loginfo(f"moved x: {self.move_x}")
        """
        #### Adjust vertical position to center the object if distance bigger than 40 cm ###
        if abs(offset_y) > center_tolerance:
            if offset_y > 0:
                if not self.stop:
                    rospy.loginfo("Adjusting down to center object vertically.")
                    twist.linear.y = direction * (-0.1)  # Move backward
            else:
                if not self.stop:
                    rospy.loginfo("Adjusting up to center object vertically.")
                    twist.linear.y = direction * (0.1)  # Move forward
        else:
            rospy.loginfo("Vertical alignment is optimal.")
            twist.linear.y = 0

        self.mecanum_pub.publish(twist)
        rospy.sleep(0.1)
        twist.linear.y = 0  # Stop vertical movement
        """
        # Check if object is centered, then move closer
        if abs(offset_x) <= center_tolerance:
            rospy.loginfo("Object is centered. Moving closer.")
            #new_object = Defect(detected_object, self.move_y, distance)
            twist.linear.y = direction * -0.1  # Move forward to get closer
            self.move_y += direction
            self.mecanum_pub.publish(twist)
            rospy.sleep(0.1)
            twist.linear.y = 0  # Stop forward movement
            rospy.loginfo(f"moved y: {self.move_y}")
            self.mecanum_pub.publish(twist)
            # Check confidence and stop if it decreases
            if y_min < edge_margin or y_max > frame_height - edge_margin:
                rospy.loginfo(f"The the bbox of the object too close to edge. Stopping.")
                rospy.loginfo(f"Relative Location of the object: {self.move_x}, {self.move_y*direction + distance}.")
                self.check_and_create_defect(detected_object, self.move_x, self.move_y*direction + distance, frame, distance)
                return True
            if distance < 15:
                rospy.loginfo(f"The object is close enough {distance}. Stopping.")
                rospy.loginfo(f"Relative Location of the object: {self.move_x}, {self.move_y*direction + distance}.")
                self.check_and_create_defect(detected_object, self.move_x, self.move_y*direction + distance, frame, distance)
                return True
            if self.check_counter < 3 : # check last 3 threshold and descide accordingly
                rospy.loginfo("Confidence increase check while approaching")
                rospy.loginfo(f"Relative Location of the object: {self.move_x}, {self.move_y*direction + distance}.")
                self.check_and_create_defect(detected_object, self.move_x, self.move_y*direction + distance, frame, distance)
            else:
                rospy.loginfo("Confidence didnt increase while approaching")
                return True
        else:
            rospy.loginfo("Object not centered yet. Adjusting position.")
        
    def check_and_create_defect(self, detected_object, new_x, new_y, image, distance,large = False):

        # Check if location is already covered
        for defect in self.defects:
            if defect.horizontal_area[0] < new_x < defect.horizontal_area[1]:
                rospy.loginfo(f"Location ({new_x}, {new_y}) is already in a defect area {defect.horizontal_area}.")
                rospy.loginfo(f"defect confidance: {defect.confidence}")
                if len(defect.confidence) < 3:
                    defect.confidence.append(detected_object.score)
                    defect.img.append(image)
                    self.check_counter = 0 
                elif detected_object.score > min(defect.confidence):
                    del defect.confidence[defect.confidence.index(min(defect.confidence))]
                    del defect.img[defect.confidence.index(min(defect.confidence))]

                    defect.confidence.append(detected_object.score)
                    defect.img.append(image)
                    self.check_counter = 0
                else:
                    self.check_counter += 1
                return False  # No new defect created

        # Create new defect object if location is new
        new_defect = Defect(detected_object, new_x, new_y)
        distance_horizontal = self.calculate_physical_width(detected_object, distance=distance)
        box = (new_x-distance_horizontal, new_x+distance_horizontal)
        print(f'box{box}')
        new_defect.horizontal_area = box
        print(f'Area is the problem')
        new_defect.large = large
        new_defect.confidence.append(detected_object.score)
        new_defect.img.append(image)
        new_defect.recorded = False
        rospy.loginfo(f"Created new defect: {new_defect.id} at location {new_defect.location}")
        rospy.loginfo(f"The area of the defect: {new_defect.horizontal_area}")
        self.defects.append(new_defect)

    def calculate_physical_width(self, defect, distance):

        box = defect.box  # [ymin, xmin, ymax, xmax]
        # problem with sizes
        box = list(box)
        if box[0] > 480:
            box[0] = 479
        if box[2] > 480:
            box[2] = 479
        box = tuple(box)
        
        # Calculate the physical width of the defect
        avg_distance = distance 

        # Approximate defect length (requires camera intrinsics)
        pixel_width = box[3] - box[1]  # Pixel distance in horizontal direction
        focal_length = self.camera_intrinsics['fx']  # Example: 525 (depends on your camera)
        physical_width = (pixel_width * avg_distance) / focal_length

        rospy.loginfo(f"Physical defect width: {physical_width:.2f} cm")
        return physical_width


    def calculate_box(self, defect, distange):
        box = defect.box
        side_roi = self.depth_frame[box[0], box[1]:box[3]]
        side_distances = side_roi[np.logical_and(side_roi > 0, side_roi < 40000)]

        side_avg_distance = int(np.mean(side_distances)/10)
        side_wing = side_avg_distance *side_avg_distance - distange*distange
        return side_wing

    def save_defect(self):
        rospy.loginfo(f"Saving the image of the Defects")
        defect = self.defects[-1]
        rospy.loginfo(f"defects: {len(self.defects)}")
        if not defect.recorded:
            defect.recorded = True
            index = defect.confidence.index(max(defect.confidence))
            current_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            img_name = f"detected_{current_time}_{defect.type}_{defect.confidence[index]:.2f}.jpg"
            label_name = img_name.replace(".jpg", ".txt")
            cv2.imwrite(image_detect_folder+ img_name ,defect.img[index])
            rospy.loginfo(f"Defect: {defect.defect} and Box: {defect.defect.box}")

            # Save label in YOLO format (class_id x_center y_center width height)
            with open(label_folder + label_name, "w") as label_file:
                box = defect.defect.box
                
                class_id = 0 # There is only 'crack' label  
                x_center = (box[0] + box[2]) / 2  # (xmin + xmax) / 2
                y_center = (box[1] + box[3]) / 2  # (ymin + ymax) / 2
                width = box[2] - box[0]  # xmax - xmin
                height = box[3] - box[1]  # ymax - ymin
                    
                # Normalize the values 
                img_height, img_width, _ = defect.img[index].shape
                x_center /= img_width
                y_center /= img_height
                width /= img_width
                height /= img_height
                    
                label_file.write(f"{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")
            
            rospy.loginfo(f"Saved image: {img_name} and label: {label_name}")
        else:
            rospy.loginfo(f"The defect is already recorded")

    def keep_bbox_in_frame_with_direction(self, detected_object, frame, twist, direction, distance,edge_margin=20):
        """
        Ensures the bounding box of the detected object stays within the frame, considering camera direction.

        Parameters:
            detected_object (object): Detected object with bounding box attributes.
            frame (numpy array): Current frame (used to calculate frame dimensions).
            twist (Twist): Twist object for robot movement.
            direction (int): Camera direction (-1 for left, 1 for right).
            edge_margin (int): Minimum distance the bbox edges should maintain from the frame edges.

        Returns:
            bool: True if adjustments were made, False if the bbox is within the frame.
        """
        # Extract bounding box
        bbox = detected_object.box  # Assuming this is a tuple (x_min, y_min, x_max, y_max)
        x_min, y_min, x_max, y_max = bbox

        # Get frame dimensions
        frame_height, frame_width = frame.shape[:2]

        # Initialize flags
        adjusted = False

        # Check horizontal edges (x_min and x_max)
        if x_min < edge_margin and x_max > frame_width - edge_margin:  # Too close to both sides
            if not self.stop:
                self.move_y -= direction
                rospy.loginfo(f"y:{self.move_y}")
                rospy.loginfo(f"Both sides doesnt fit.")
                twist.linear.y = direction * 0.1 
            else:
                ### TODO add creating demage record
                self.check_and_create_defect(detected_object, self.move_x, self.move_y*direction + distance, frame, distance)
                if distance < 150:
                    self.move_x -= 1
                    twist.linear.x = -0.1 
 
                adjusted = True

        elif x_min < edge_margin:  # Too close to left edge
            self.move_x += direction
            rospy.loginfo(f"BBox too close to left edge. Moving forward to adjust.")
            twist.linear.x = direction * 0.1  
            adjusted = True
        elif x_max > frame_width - edge_margin:  # Too close to right edge
            self.move_x -= direction
            rospy.loginfo("BBox too close to Right edge. ")
            twist.linear.x = direction * (-0.1)  
            adjusted = True
        rospy.loginfo(f"x:{self.move_x}")
        # Publish twist if horizontal adjustment is made
        if twist.linear.x != 0 or twist.linear.y != 0:
            self.mecanum_pub.publish(twist)
            rospy.sleep(0.1)
            twist.linear.x = 0
            twist.linear.y = 0
            self.mecanum_pub.publish(twist)

        # Check vertical edges (y_min and y_max)
        if y_min < edge_margin or y_max > frame_height - edge_margin:  # Too close to top edge
            if not self.stop or distance < 150:
                rospy.loginfo("BBox too close to top or bottom edge. Moving backward to adjust.")
                twist.linear.y = direction * 0.1  

        
        # Publish twist if vertical adjustment is made
        if twist.linear.y != 0:
            self.move_y -= direction
            rospy.loginfo(f"y:{self.move_y}")
            self.mecanum_pub.publish(twist)
            rospy.sleep(0.1)
            twist.linear.y = 0
            self.mecanum_pub.publish(twist)
        return adjusted




    def run(self):
        """
        Continuously processes frames, detects objects, and moves the robot towards the detected objects.
        """
        twist = Twist()
        
        self.position.data = 0
        self.cam_servo_pub.publish(self.position)

        frame_center_h = self.depth_frame.shape[0] // 2
        frame_center_w = self.depth_frame.shape[1] // 2
        check_left = False
        check_right = False
        manuel = True
        allinged = False
        search_timeout = 3  # Timeout in seconds
        sweep_timeout = 1
        start_time = time.time()
        start_sweep_time = time.time()
        

        while self.running:
            # Get the latest image from the queue
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                rospy.logwarn("Image queue is empty. Skipping frame.")
                continue

            try:
                # Process the image and detect objects
                result_image = self.image_proc(np.copy(image))
                # Display the result image
                cv2.imshow(self.name, result_image)
                key = cv2.waitKey(1)

                if key == ord('m'):
                    manuel = not manuel
                    self.move_robot(twist, linear_x=0, linear_y=0)
                    check_left = False
                    check_right = False
                    sweep = True
                    start_time = time.time()  # Reset timeout
                    rospy.loginfo(f"Switched to {'manual' if manuel else 'autonomous'} mode.")

                elif key == ord('q') or key == 27:  # Exit on 'q' or 'Esc'
                        self.mecanum_pub.publish(Twist())
                        self.position.data = 0
                        self.cam_servo_pub.publish(self.position)
                        rospy.signal_shutdown('shutdown')
                        break

                if manuel:
                    # Handle manual object detection details
                    if self.detected_objects:

                        current_object, center, distance = self.detected_objects[0]
                        #rospy.loginfo(f"distance: {distance}")
                        #angle = self.calculate_object_angle(center, image_width = result_image.shape[1])
                        #radian = math.radians(angle)
                        #self.position.data += radian
                        #self.cam_servo_pub.publish(self.position)
                        #rospy.sleep(0.2)
                        #box = current_object.box
                        #left_angle, right_angle = self.calculate_bbox_angles(box, image_width=self.depth_frame.shape[1])
                        #rospy.loginfo(f"Object angles: Left {math.degrees(left_angle)}°, Right {math.degrees(right_angle)}°.")
                        #distances = self.get_distances_at_angles(self.lidar_data, [right_angle, left_angle])
                        #for angle, distance in distances.items():
                        #    if distance is None:
                        #        rospy.logwarn(f"No valid data at angle {math.degrees(angle)}°.")
                        #    else:
                        #        rospy.loginfo(f"Distance at angle {math.degrees(angle)}°: {distance} meters.")

                    # Movement control
                    if key in [81, 82, 83, 84, ord('a'), ord('d'), ord('c'), ord('x')]:
                        if key == 81:  # Left arrow
                            self.move_robot(twist, linear_y=0.2)
                        elif key == 82:  # Up arrow
                            self.move_robot(twist, linear_x=0.2)
                        elif key == 83:  # Right arrow
                            self.move_robot(twist, linear_y=-0.2)
                        elif key == 84:  # Down arrow
                            self.move_robot(twist, linear_x=-0.2)
                        elif key == ord('a'):  # Rotate left
                            self.move_robot(twist, angular_z=0.4)
                        elif key == ord('d'):  # Rotate right
                            self.move_robot(twist, angular_z=-0.4)

                        # Camera servo control
                        elif key == ord('c'):  # Rotate camera up
                            if math.degrees(self.position.data) < 122.5:
                                self.position.data += math.radians(22.5)
                                self.cam_servo_pub.publish(self.position)
                            else:
                                rospy.logwarn("Camera at maximum upward angle.")
                        elif key == ord('x'):  # Rotate camera down
                            if math.degrees(self.position.data) > -122.5:
                                self.position.data -= math.radians(22.5)
                                self.cam_servo_pub.publish(self.position)
                            else:
                                rospy.logwarn("Camera at maximum downward angle.")
                elif not manuel:
                    # If an object is detected and centered information is available
                    if not check_left and math.degrees(self.position.data) != -90: 
                        start_time = time.time()
                        rospy.loginfo(f"Turning camera to Left")
                        radian = math.radians(-90)
                        self.position.data = radian
                        self.cam_servo_pub.publish(self.position)
                        rospy.sleep(0.2)
                    if self.detected_objects:
                        start_time = time.time()
                        # Focus on the closest detected object
                        current_object, self.center, distance = self.detected_objects[0]
                        rospy.loginfo(f"Object detected: {current_object.class_name} at {distance} centimeters.")
                        # Process and adjust based on bounding box edges
                        

                        if abs(math.degrees(self.position.data)) <80:
                            self.navigate(twist)
                            rospy.sleep(0.1)
                            angle = self.calculate_object_angle(self.center, image_width = result_image.shape[1])

                            radian = math.radians(angle)
                            self.position.data += radian
                            self.cam_servo_pub.publish(self.position)
                            rospy.sleep(0.1)

                        if abs(math.degrees(self.position.data)) > 80:
                            if self.position.data < 0:
                                check_left = False
                                radian = math.radians(-90)
                                self.position.data = radian
                                self.cam_servo_pub.publish(self.position)
                                decision_time = time.time() - self.response_time
                                rospy.loginfo(f"decision_time:{decision_time}")
                                rospy.sleep(0.1)
                            else:
                                check_right = False
                                radian = math.radians(90)
                                self.position.data = radian
                                self.cam_servo_pub.publish(self.position)
                                rospy.sleep(0.1)

                            if self.keep_bbox_in_frame_with_direction(
                                detected_object=current_object,
                                frame=image,
                                twist=twist,
                                direction=1 if self.position.data > 0 else -1,
                                distance=distance,
                                ):
                                decision_time = time.time() - self.response_time
                                rospy.loginfo(f"Adjustments made to keep bbox in frame.{decision_time}")
                            else:
                                rospy.loginfo("BBox is within frame. Proceeding with centering and approaching.")
                                direction=1 if self.position.data > 0 else -1
                                self.check_and_create_defect(detected_object =current_object, new_x = self.move_x, new_y = self.move_y*direction + distance, image= image, distance = distance)
                                allinged = self.process_and_approach_object(
                                    detected_object=current_object,
                                    frame=image,
                                    twist=twist,
                                    direction=1 if self.position.data > 0 else -1,
                                    distance=distance
                                    )
                            # Decide next action
                            if allinged:
                                self.save_defect()
                                if self.position.data < 0 :
                                    check_left = True
                                    self.position.data = math.radians(90)
                                else:
                                    check_right = True
                                    self.position.data = math.radians(0)

                            self.cam_servo_pub.publish(self.position)
                            rospy.sleep(0.5)

                    else:
                        if time.time() - start_time > search_timeout:
                            if not check_left:
                                start_time = time.time()
                                if self.defects and not self.defects[-1].recorded:
                                    self.save_defect()
                                rospy.loginfo("No object on the left. Switching to the right.")
                                check_left = True
                                self.position.data = math.radians(90)
                                self.cam_servo_pub.publish(self.position)
                                rospy.sleep(0.5)
                        
                            elif not check_right:
                                start_time = time.time()
                                if self.defects and not self.defects[-1].recorded:
                                    self.save_defect()
                                rospy.loginfo("No object on the right. Switching to forward.")
                                check_right = True
                                self.position.data = math.radians(0)
                                self.cam_servo_pub.publish(self.position)
                                rospy.sleep(0.5)
                            else:
                                rospy.loginfo("No object found. Navigating forward.")
                                if sweep:
                                    if time.time() - start_sweep_time > sweep_timeout:

                                        self.sweep_camera()
                                        start_sweep_time = time.time()
                                        sweep = False
                                else:
                                    self.navigate(twist)
                                    start_sweep_time = time.time()
                                    sweep = True


            except BaseException as e:
                rospy.logerr(f"Error during processing: {e}")
                result_image = cv2.flip(image, 1)




            # Reset the center after processing
            self.center = None            
            
if __name__ == "__main__":
    print('\n******Press "q" or "esc" key to exit!******')
    PoseAdjustmentNode('pose_adjustment')
