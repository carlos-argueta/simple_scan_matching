from math import degrees
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import numpy as np
import open3d as o3d

from scipy.spatial.transform import Rotation

class ScanMatcher:
    def __init__(self):
        self.node = rclpy.create_node('scan_matcher')
        self.subscription = self.node.create_subscription(
            LaserScan,
            '/scan_filtered',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Subscribe to the /odom topic
        self.odom_subscriber = self.node.create_subscription(
            Odometry,
            '/odom_rf2o', #'/fake_odom',  #'/odom', '/wheel_odom'
            self.odom_callback,
            10)
        
        self.odom_msg = Odometry()

        self.prev_cloud = None

        self.scan_count = 0

        # Initialize an identity matrix as the initial odometry
        self.odometry = np.eye(4)
        self.odometry2 = np.eye(4)
        self.odometry3 = np.eye(4)

    def odom_callback(self, msg):

        self.odom_msg = msg

    def listener_callback(self, msg):

        self.scan_count += 1

        cloud = self.scan_to_pointcloud(msg)

        if self.prev_cloud is None:
            self.prev_cloud = cloud
            return
        
        #if self.scan_count % 5 != 0:
        #    return
    
        # Perform ICP or other processing here
        transformation, inlier_rmse = self.perform_icp(self.prev_cloud, cloud)
        transformation2, inlier_rmse2 = self.perform_icp_ransac(self.prev_cloud, cloud)
        transformation3, inlier_rmse3 = self.perform_icp_point_to_plane(self.prev_cloud, cloud)

        # Multiply the current odometry with the new transformation matrix to update the odometry
        self.odometry = np.dot(self.odometry, transformation)
        self.odometry2 = np.dot(self.odometry2, transformation2)
        self.odometry3 = np.dot(self.odometry3, transformation3)

        # Extract rotation matrix
        R = self.odometry[0:3, 0:3]
        euler = self.rotation_to_euler(R)

        R2 = self.odometry2[0:3, 0:3]
        euler2 = self.rotation_to_euler(R2)

        R3 = self.odometry3[0:3, 0:3]
        euler3 = self.rotation_to_euler(R3)

        #self.node.get_logger().info(str(R))
        #self.node.get_logger().info(str(euler))

        # Extract translation vector
        T = self.odometry[0:3, 3]
        T2 = self.odometry2[0:3, 3]
        T3 = self.odometry3[0:3, 3]

        _, _, theta = self.euler_from_quaternion(self.odom_msg.pose.pose.orientation)

        # Log the odometry
        self.node.get_logger().info('Odometry: x: %f, y: %f, yaw: %f' % (T[0], T[1], euler[2]))
        self.node.get_logger().info('Odometry2: "%s" "%s" "%s"' % (self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y, degrees(theta)))
        
        #self.node.get_logger().info('Odometry2: x: %f, y: %f, yaw: %f' % (T2[0], T2[1], euler2[2]))
        #self.node.get_logger().info('Odometry3: x: %f, y: %f, yaw: %f' % (T3[0], T3[1], euler3[2]))

        self.prev_cloud = cloud

    def scan_to_pointcloud(self, msg):
        """ Converts a ROS LaserScan message to an Open3D PointCloud object. """
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        points = np.array([np.cos(angles)*msg.ranges, np.sin(angles)*msg.ranges, np.zeros(len(msg.ranges))]).T
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(points)
        return cloud

    
    def perform_icp(self, cloud1, cloud2):
        """ Performs ICP to align cloud2 to cloud1. Returns the transformation and the inlier RMSE. """
        threshold = 0.05  # distance threshold
        trans_init = np.asarray([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])  # initial transformation
        reg_p2p = o3d.pipelines.registration.registration_icp(cloud1, cloud2, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint())
        return reg_p2p.transformation, reg_p2p.inlier_rmse
    
    # This is a very basic use of RANSAC with ICP. Depending on your specific needs, 
    # you might need to adjust the parameters or use a more complex approach.
    def perform_icp_ransac(self, source, target):
        threshold = 0.02  # Distance threshold for RANSAC
        trans_init = np.eye(4)  # Initial transformation

        # Perform RANSAC followed by ICP refinement
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source, target, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))

        return reg_p2p.transformation, reg_p2p.inlier_rmse
    
    def perform_icp_point_to_plane(self, source, target):

        # Estimate normals
        o3d.geometry.PointCloud.estimate_normals(
            source,
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))

        o3d.geometry.PointCloud.estimate_normals(
            target,
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))

        threshold = 1  # Distance threshold for RANSAC
        trans_init = np.eye(4)  # Initial transformation

        # Perform point-to-plane ICP
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source, target, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())

        return reg_p2p.transformation, reg_p2p.inlier_rmse


    
    def rotation_to_euler(self, R):
        """ Converts a rotation matrix to Euler angles. """

        r = Rotation.from_matrix(R)
        euler = r.as_euler('xyz', degrees=True)
        return euler   
    
    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    

def main(args=None):
    rclpy.init(args=args)

    scan_matcher = ScanMatcher()

    rclpy.spin(scan_matcher.node)

    scan_matcher.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
