import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Image
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import yaml
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf_transformations
from tf2_ros import TransformBroadcaster
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Int32, String
import math
import yaml
import datetime
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile, ReliabilityPolicy

class SimNode(Node):
    def __init__(self):
        super().__init__('ros2_robot_sim_node')
        
        
        self.image_map = cv2.imread("/ros2_humble_sim_ws/src/ros2-humble-sim/ros2_robot_sim/resource/map/map.pgm",0)
        self.image_map = cv2.flip(self.image_map, 0)

        self.map_info_dict = self.load_map_yaml("/ros2_humble_sim_ws/src/ros2-humble-sim/ros2_robot_sim/resource/map/map.yaml")
        
        self.max_lin_vel = 1.0
        self.min_lin_vel = -1.0
        
        self.min_rad_per_second = -0.785398
        self.max_rad_per_second = 0.785398
        
        self.global_frame = 'map'
        self.base_frame  = 'base_link'
        self.odom_frame = 'odom'
        
        self.set_initial_pose()  
        
        self.nav = BasicNavigator()   
        
        # Create callback groups
        self.cmd_vel_group = MutuallyExclusiveCallbackGroup()
        self.nav2_group = MutuallyExclusiveCallbackGroup() 
        self.timer_group = MutuallyExclusiveCallbackGroup()   
        self.cloud_group = MutuallyExclusiveCallbackGroup()   

        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        self.marker_array_subscriber = self.create_subscription(
            MarkerArray,
            '/cogniteam_ros2_sim/markers_input',
            self.markers_callback,
            qos_profile
        )

        # Subscribe to cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.cmd_vel_callback,
            10,
            callback_group=self.cmd_vel_group
        )
        
         # Subscribe goal
        self.goal_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/cogniteam_ros2_sim/goal',
            self.goal_callback,
            10,
            callback_group=self.nav2_group
        )
        
        # Subscribe joy
        self.joy_subscriber = self.create_subscription(
            Joy,
            '/cogniteam_ros2_sim/joy',
            self.joy_callback,
            10,
             callback_group=self.cmd_vel_group
        )

        # Publish Odometry
        self.robot_pose_publisher = self.create_publisher(PoseStamped, 
                                                            '/cogniteam_ros2_sim/robot_pose', 10)

        self.path_publisher = self.create_publisher(Path, '/cogniteam_ros2_sim/robot_path', 10)

        self.battery_voltage_publisher = self.create_publisher(Int32, '/cogniteam_ros2_sim/battery_voltage', 10)

        self.date_publisher = self.create_publisher(String, '/cogniteam_ros2_sim/date', 10)

        self.marker_array_publisher = self.create_publisher(MarkerArray, '/cogniteam_ros2_sim/marker_array',  qos_profile)
        
        
        self.scan_publisher = self.create_publisher(LaserScan, '/cogniteam_ros2_sim/scan', 10)

        self.cloud_publisher = self.create_publisher(PointCloud2, '/cogniteam_ros2_sim/pointcloud', 10)
        
        self.markers_output_publisher = self.create_publisher(MarkerArray, '/cogniteam_ros2_sim/markers_output',  qos_profile)


        # Add a publisher for the compressed image
        self.image_publisher = self.create_publisher(CompressedImage, '/cogniteam_ros2_sim/compressed_image', 10, )
        self.bridge = CvBridge()
        
        # Timer to publish odometry
        self.odom_timer = self.create_timer(0.1, self.publish_odometry, self.timer_group)
        
        self.cloud_timer = self.create_timer(1, self.publish_point_cloud, self.cloud_group)
        

        # self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        # Initialize robot state
       
        self.current_velocity = Twist()
        
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_timer = self.create_timer(0.1, self.tf_timer_callback, callback_group=self.timer_group)


    
    def joy_callback(self, msg: Joy):
        # Right stick vertical axis (axes[4]) controls linear velocity
        right_stick_y = msg.axes[5]  # Forward/Back movement of the right stick
        
        # Normalize to linear velocity range
        if right_stick_y > 0:
            # Moving forward (up), interpolate between 0 and max_lin_vel
            linear_velocity = right_stick_y * self.max_lin_vel
        else:
            # Moving backward (down), interpolate between min_lin_vel and 0
            linear_velocity = right_stick_y * abs(self.min_lin_vel)

        # Left stick horizontal axis (axes[0]) controls angular velocity
        left_stick_x = msg.axes[0]  # Left/Right movement of the left stick
        
        # Normalize to angular velocity range
        if left_stick_x > 0:
            # Moving right, interpolate between 0 and max_rad_per_second
            angular_velocity = left_stick_x * self.max_rad_per_second
        else:
            # Moving left, interpolate between min_rad_per_second and 0
            angular_velocity = left_stick_x * abs(self.min_rad_per_second)

        # Create Twist message and publish
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        
        if self.robot_pose is not None:

            self.apply_twist_to_pose(twist_msg, 0.1)

        
        
    def load_map_yaml(self,yaml_file_path):
        with open(yaml_file_path, 'r') as file:
            try:
                map_data = yaml.safe_load(file)
                return map_data
            except yaml.YAMLError as exc:
                print(f"Error loading YAML file: {exc}")
                return None
        
    def convert_pose_to_pix(self, pose: PoseStamped):
        # Calculate pixel coordinates
        x_pix = (pose.pose.position.x - self.map_info_dict['origin'][0]) / self.map_info_dict['resolution']
        y_pix = (pose.pose.position.y - self.map_info_dict['origin'][1]) / self.map_info_dict['resolution']

        # Create a point (in OpenCV, this would be a tuple)
        p = (int(x_pix), int(y_pix))

        return p

    def convert_pix_to_pose(self,pixel):
   
        pose = PoseStamped()
        
        pose.header.frame_id = self.global_frame

        pose.pose.position.x = (pixel[0] * self.map_info_dict['resolution']) + self.map_info_dict['origin'][0]
        pose.pose.position.y = (pixel[1] * self.map_info_dict['resolution']) + self.map_info_dict['origin'][1]
        pose.pose.position.z = 0.0

        # Assuming `q` is an instance of Quaternion
        pose.pose.orientation.w = 1.0

        return pose
 
    def raycast_to_black_pixel(self, image, robot_pos, yaw, degree, max_range=None):
        
        height, width = image.shape

        # Convert degree to radians and adjust with yaw
        angle = yaw + math.radians(degree)

        # Unit direction vector for the ray
        direction = (math.cos(angle), math.sin(angle))
        
        # Initialize the robot's position
        x_robot, y_robot = robot_pos
        x, y = robot_pos
        
        step_size = 1  # Step by 1 pixel each time (can be adjusted)
        distance = 0

        
        while 0 <= int(x) < width and 0 <= int(y) < height:
            # Check the pixel value at the current ray position
            if image[int(y), int(x)] == 0:  # Check if the pixel is black (0)

                distance_from_robot = math.sqrt((x_robot - x) ** 2 + (y_robot - y) ** 2)

                return (int(x), int(y)), distance_from_robot
            
            # Move along the ray direction
            x += direction[0] * step_size
            y += direction[1] * step_size

            # Increase distance covered
            distance += step_size

            # If a max range is provided, stop if we exceed it
            if max_range and distance >= max_range:
                break
       
        # Return None if no black pixel was found
        return None, None

    def publish_compressed_image(self):
        # Create a blank image with RGB background color (230, 0, 126)
        img = np.zeros((400, 400, 3), np.uint8)
        img[:] = (126, 0, 230)  # Set the entire image to the background color

        # Get the current time
        current_time = datetime.datetime.now().strftime("%d/%m/%Y %H:%M:%S")

        # Add the timestamp to the top-left corner of the image
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        font_color = (255, 255, 255)  # White
        thickness = 2
        position = (10, 30)  # Top-left corner
        cv2.putText(img, current_time, position, font, font_scale, font_color, thickness)

        # Calculate the center of the image and draw the arrow
        center = (200, 200)  # Center of the image
        length = 100  # Length of the arrow

        yaw_angle = 0.0
        if self.robot_pose is not None:
            
            # Extract quaternion from the PoseStamped message
            orientation_q = self.robot_pose.pose.orientation

            # Convert quaternion to Euler angles
            quaternion = (
                orientation_q.x,
                orientation_q.y,
                orientation_q.z,
                orientation_q.w
            )
            euler = tf_transformations.euler_from_quaternion(quaternion)

            # Yaw is the third value in the returned tuple (roll, pitch, yaw)
            yaw_angle = euler[2]
            
        # Calculate the end point of the arrow based on the yaw angle
        arrow_angle = yaw_angle  # Negative to account for coordinate system
        end_point = (int(center[0] + length * np.cos(arrow_angle)),
                     int(center[1] - length * np.sin(arrow_angle)))

        # Draw the arrow (white arrow, thickness of 5)
        cv2.arrowedLine(img, center, end_point, (255, 255, 255), 5)

        # Convert the OpenCV image to a ROS CompressedImage message
        compressed_image_msg = CompressedImage()
        compressed_image_msg.header.stamp = self.get_clock().now().to_msg()
        compressed_image_msg.format = "jpeg"
        compressed_image_msg.data = np.array(cv2.imencode('.jpg', img)[1]).tobytes()

        # Publish the compressed image
        self.image_publisher.publish(compressed_image_msg)
        
    def markers_callback(self, msg):
        
        self.markers_output_publisher.publish(msg)
        
    def goal_callback(self, msg):
        # navigation_launch.py
        print('goal !!')

        self.nav.setInitialPose(self.robot_pose)

        nav_goal = PoseStamped()
        # Copy the header from PoseWithCovarianceStamped to PoseStamped
        nav_goal.header = msg.header
        # Copy the pose from PoseWithCovarianceStamped to PoseStamped (ignoring covariance)
        nav_goal.pose = msg.pose.pose
        wanted_path = self.nav.getPath(self.robot_pose, nav_goal)
        
        if wanted_path == None:
            return
        
        if len(wanted_path.poses) ==  0:
            self.get_logger().info('bad path !!')
            return
            
        smoothed_path = self.nav.smoothPath(wanted_path)
        
        self.path_publisher.publish(wanted_path)


        self.nav.goToPose(nav_goal)
        # while not self.nav.isTaskComplete():   
        #     rclpy.spin_once(self)        
            # feedback = self.nav.getFeedback()
            # if feedback.navigation_time.nanoseconds / 1e9 > 600:
            #     self.nav.cancelTask()      

        # result = self.nav.getResult()
        # if result == TaskResult.SUCCEEDED:
        #     print('Goal succeeded!')
        # elif result == TaskResult.CANCELED:
        #     print('Goal was canceled!')
        # elif result == TaskResult.FAILED:
        #     print('Goal failed!')
        
        self.get_logger().info('roal reached !!')


    def publish_date(self):
        
        msg = String()
        msg.data = str(datetime.datetime.now().strftime("%d/%m/%Y %H:%M:%S"))
        
        self.date_publisher.publish(msg)
    
    
    def publish_battery_voltage(self):
        msg = Int32()
        msg.data = 32

        self.battery_voltage_publisher.publish(msg)
        
    def apply_twist_to_pose(self, twist, delta_time):
        # Extract the current position and orientation
        x = self.robot_pose.pose.position.x
        y = self.robot_pose.pose.position.y
        z = self.robot_pose.pose.position.z
        orientation = self.robot_pose.pose.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]

        # Convert orientation quaternion to Euler angles
        _, _, yaw = tf_transformations.euler_from_quaternion(q)

        # Linear velocities (in the robot's local frame)
        linear_x = twist.linear.x
        linear_y = twist.linear.y
        linear_z = twist.linear.z

        # Angular velocity (yaw)
        angular_z = twist.angular.z

        # Update yaw with angular velocity
        yaw += angular_z * delta_time

        # Calculate the new position in the world frame
        x += (linear_x * math.cos(yaw) - linear_y * math.sin(yaw)) * delta_time
        y += (linear_x * math.sin(yaw) + linear_y * math.cos(yaw)) * delta_time
        z += linear_z * delta_time

        # Convert the updated yaw back to a quaternion
        new_orientation = tf_transformations.quaternion_from_euler(0, 0, yaw)

        # Update the PoseStamped message
        self.robot_pose  = PoseStamped()
        self.robot_pose.header.frame_id = self.global_frame
        self.robot_pose.header.stamp = self.get_clock().now().to_msg()
        self.robot_pose.pose.position.x = x
        self.robot_pose.pose.position.y = y
        self.robot_pose.pose.position.z = z
        self.robot_pose.pose.orientation.x = new_orientation[0]
        self.robot_pose.pose.orientation.y = new_orientation[1]
        self.robot_pose.pose.orientation.z = new_orientation[2]
        self.robot_pose.pose.orientation.w = new_orientation[3]


    def set_initial_pose(self):
        # Initialize PoseStamped message
        self.robot_pose = PoseStamped()

        # Set the header
        self.robot_pose.header.frame_id = self.global_frame
        self.robot_pose.header.stamp = Time()

        # Set the position
        self.robot_pose.pose.position.x = -2.91037
        self.robot_pose.pose.position.y = 0.0
        self.robot_pose.pose.position.z = 0.0

        # Set the orientation (Quaternion)
        self.robot_pose.pose.orientation.x = 0.0
        self.robot_pose.pose.orientation.y = 0.0
        self.robot_pose.pose.orientation.z = 0.711624
        self.robot_pose.pose.orientation.w = 0.702561
        
   

    def cmd_vel_callback(self, twist_msg):
        
        if self.robot_pose is not None:
            self.get_logger().info('cmd_vel_callback !!')

            self.apply_twist_to_pose(twist_msg, 0.1)


    def publish_marker_array(self):
        
        marker_array = MarkerArray()
        
        table_1_marker = Marker()
        table_1_marker.header.frame_id = self.global_frame  
        table_1_marker.header.stamp = self.get_clock().now().to_msg()
        table_1_marker.ns = "cogniteam_sim"
        table_1_marker.id = 0  # Unique ID for each marker
        table_1_marker.type = Marker.CUBE
        table_1_marker.action = Marker.ADD
        table_1_marker.pose.position.x = -2.9
        table_1_marker.pose.position.y = -6.0
        table_1_marker.pose.position.z = 0.01
        table_1_marker.pose.orientation.x = 0.0
        table_1_marker.pose.orientation.y = 0.0
        table_1_marker.pose.orientation.z = 0.700909
        table_1_marker.pose.orientation.w = 0.713251
        table_1_marker.scale.x = 4.0  # Size in meters (length)
        table_1_marker.scale.y = 2.0  # Size in meters (width)
        table_1_marker.scale.z = 0.5  # Size in meters (height)
        table_1_marker.color.r = 150.0/ 255.0
        table_1_marker.color.g = 75.0/ 255.0
        table_1_marker.color.b = 0.0
        table_1_marker.color.a = 1.0  # Fully opaque
        table_1_marker.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
        
        table_2_marker = Marker()
        table_2_marker.header.frame_id = self.global_frame  
        table_2_marker.header.stamp = self.get_clock().now().to_msg()
        table_2_marker.ns = "cogniteam_sim"
        table_2_marker.id = 1  # Unique ID for each marker
        table_2_marker.type = Marker.CUBE
        table_2_marker.action = Marker.ADD
        table_2_marker.pose.position.x = -4.66361
        table_2_marker.pose.position.y = -1.15407
        table_2_marker.pose.position.z = 0.01
        table_2_marker.pose.orientation.x = 0.0
        table_2_marker.pose.orientation.y = 0.0
        table_2_marker.pose.orientation.z = 0.700909
        table_2_marker.pose.orientation.w = 0.713251
        table_2_marker.scale.x = 4.0  # Size in meters (length)
        table_2_marker.scale.y = 1.0  # Size in meters (width)
        table_2_marker.scale.z = 0.5  # Size in meters (height)
        table_2_marker.color.r = 150.0/ 255.0
        table_2_marker.color.g = 75.0/ 255.0
        table_2_marker.color.b = 0.0
        table_2_marker.color.a = 1.0  # Fully opaque
        table_2_marker.lifetime = rclpy.duration.Duration(seconds=1).to_msg()

        # Add the marker to the marker array
        marker_array.markers.append(table_1_marker)
        marker_array.markers.append(table_2_marker)


        # Publish the marker array
        self.marker_array_publisher.publish(marker_array)
    

    def publish_odometry(self):
        
        if self.robot_pose is not None:           
           
            self.robot_pose_publisher.publish(self.robot_pose)

    
    def publish_point_cloud(self):
        
        x_pix_robot ,y_pix_robot  = self.convert_pose_to_pix(self.robot_pose)

        points = []
        for deg in range(360):
            scan_pix, distnace_from_robot = self.raycast_to_black_pixel(self.image_map, (x_pix_robot,y_pix_robot), deg, 0)
            if scan_pix == None:
                continue
            else:
                pose = self.convert_pix_to_pose(scan_pix)
                
                points.append([
                    pose.pose.position.x,
                    pose.pose.position.y,
                    0.3
                ])
                points.append([
                    pose.pose.position.x,
                    pose.pose.position.y,
                    0.4
                ])
                points.append([
                    pose.pose.position.x,
                    pose.pose.position.y,
                    0.5
                ])

        # Convert to numpy array for point cloud creation
        points = np.array(points, dtype=np.float32)

        # Create the PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.global_frame

        pointcloud_msg = pc2.create_cloud_xyz32(header, points)
        
        # Publish the PointCloud2 message
        self.cloud_publisher.publish(pointcloud_msg)
        
        
        # Define parameters for the LaserScan
        laser_scan = LaserScan()
        laser_scan.header = Header()
        laser_scan.header.stamp = self.get_clock().now().to_msg()
        laser_scan.header.frame_id = self.global_frame
        laser_scan.angle_min = -np.pi   # -90 degrees
        laser_scan.angle_max = np.pi    # 90 degrees
        laser_scan.angle_increment = np.pi / 180  # 1 degree
        laser_scan.time_increment = 0.0
        laser_scan.range_min = 0.05
        laser_scan.range_max = 1000.0

        # Filter points to be within a certain angle and distance
        num_readings = int((laser_scan.angle_max - laser_scan.angle_min) / laser_scan.angle_increment) + 1
        laser_scan.ranges = [float('inf')] * num_readings  # Initialize ranges

        for point in points:
            x, y, z = point

            # Calculate angle and range
            angle = np.arctan2(y, x)  # Angle in radians
            range_value = np.sqrt(x**2 + y**2)  # Euclidean distance

            # Check if the point is within the defined angle range
            if laser_scan.angle_min <= angle <= laser_scan.angle_max:
                index = int((angle - laser_scan.angle_min) / laser_scan.angle_increment)
                # Update range only if it is less than the current value
                if range_value < laser_scan.ranges[index]:
                    laser_scan.ranges[index] = range_value

        # Publish the LaserScan message
        self.scan_publisher.publish(laser_scan)
            
    def create_laserscan_from_dict(self, distance_dict, angle_min=-np.pi, angle_max=np.pi, range_min=0.0, range_max=1000.0):
      
        # Initialize LaserScan message
        scan = LaserScan()
        scan.header.frame_id = self.global_frame
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = np.deg2rad(1)  # Assuming 1 degree increments
        scan.range_min = range_min
        scan.range_max = range_max

        # Prepare ranges and intensities
        num_readings = int((angle_max - angle_min) / scan.angle_increment) + 1
        scan.ranges = [float('inf')] * num_readings  # Initialize with inf
        scan.intensities = [0.0] * num_readings  # Assuming no intensities

        # Populate ranges based on the distance dictionary
        for angle, distance in distance_dict.items():
            if angle >= 0 and angle < 360:
                
                index = int((math.radians(angle) - angle_min) / scan.angle_increment)
                
                if distance == 0.0:
                   scan.ranges[index] = 'inf'
                   
                if index > 360:
                    continue
                scan.ranges[index] = min(distance, range_max)  # Clamp to range_max

        return scan

    def tf_timer_callback(self):
        # Convert Pose to TransformStamped
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.odom_frame 
        transform.child_frame_id = self.base_frame  

        # Set translation (position)
        transform.transform.translation.x = self.robot_pose.pose.position.x
        transform.transform.translation.y = self.robot_pose.pose.position.y
        transform.transform.translation.z = self.robot_pose.pose.position.z

        # Set rotation (orientation)
        transform.transform.rotation.x = self.robot_pose.pose.orientation.x
        transform.transform.rotation.y = self.robot_pose.pose.orientation.y
        transform.transform.rotation.z = self.robot_pose.pose.orientation.z
        transform.transform.rotation.w = self.robot_pose.pose.orientation.w

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)
        
        ################################################################
        
        odom_transform = TransformStamped()

        odom_transform.header.stamp = self.get_clock().now().to_msg()
        odom_transform.header.frame_id = self.global_frame 
        odom_transform.child_frame_id = self.odom_frame 

        # Set translation (position) from pose
        odom_transform.transform.translation.x = 0.0
        odom_transform.transform.translation.y = 0.0
        odom_transform.transform.translation.z = 0.0
        odom_transform.transform.rotation.w = 1.0

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(odom_transform)
        
        self.publish_battery_voltage()
        
        self.publish_date()
        
        self.publish_compressed_image()
        
        self.publish_marker_array()

def main(args=None):
    rclpy.init(args=args)
    sim_node = SimNode()
    rclpy.spin(sim_node)
    robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


