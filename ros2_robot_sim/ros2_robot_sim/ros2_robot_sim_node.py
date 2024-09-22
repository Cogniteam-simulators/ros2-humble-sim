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
from std_msgs.msg import Int32
import math
import datetime
from visualization_msgs.msg import Marker, MarkerArray

class SimNode(Node):
    def __init__(self):
        super().__init__('ros2_robot_sim_node')
        
        self.global_frame = 'map'
        self.base_frame  = 'base_link'
        self.odom_frame = 'odom'
        
        self.set_initial_pose()  
        
        self.nav = BasicNavigator()   
        
        # Create callback groups
        self.cmd_vel_group = MutuallyExclusiveCallbackGroup()
        self.nav2_group = MutuallyExclusiveCallbackGroup() 
        self.timer_group = MutuallyExclusiveCallbackGroup()   
  
       

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

        # Publish Odometry
        self.robot_pose_publisher = self.create_publisher(PoseStamped, 
                                                            '/cogniteam_ros2_sim/robot_pose', 10)

        self.path_publisher = self.create_publisher(Path, '/cogniteam_ros2_sim/robot_path', 10)

        self.battery_voltage_publisher = self.create_publisher(Int32, '/cogniteam_ros2_sim/battery_voltage', 10)

        self.marker_array_publisher = self.create_publisher(MarkerArray, '/cogniteam_ros2_sim/marker_array', 10)
        
         # Add a publisher for the compressed image
        self.image_publisher = self.create_publisher(CompressedImage, '/cogniteam_ros2_sim/compressed_image', 10)
        self.bridge = CvBridge()
        
        # Timer to publish odometry
        self.odom_timer = self.create_timer(0.1, self.publish_odometry)

        # self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)

        # Initialize robot state
       
        self.current_velocity = Twist()
        
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_timer = self.create_timer(0.1, self.tf_timer_callback, callback_group=self.timer_group)



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
        
        if len(wanted_path.poses) ==  0:
            self.get_logger().info('bad path !!')
            return
            
        smoothed_path = self.nav.smoothPath(wanted_path)
        
        self.path_publisher.publish(wanted_path)


        self.nav.goToPose(nav_goal)
        while not self.nav.isTaskComplete():   
            continue         
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
           

            # Publish the odometry message
            self.robot_pose_publisher.publish(self.robot_pose)

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
        
        #publish the battery voltage
        self.publish_battery_voltage()
        
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
