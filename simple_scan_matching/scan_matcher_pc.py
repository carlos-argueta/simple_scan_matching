from math import degrees, radians
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

from .conversions import read_points
from .visualization import Visualization


import numpy as np
import open3d as o3d

from scipy.spatial.transform import Rotation

class ScanMatcher:
    def __init__(self):
        self.node = rclpy.create_node('scan_matcher')

        self.publisher_ = self.node.create_publisher(Odometry, 'pointcloud/odom', 10)

        self.subscription = self.node.create_subscription(
            PointCloud2,
            '/rslidar_points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Subscribe to the /odom topic
        self.odom_subscriber = self.node.create_subscription(
            Odometry,
            '/wheel_odom', #'/odom_rf2o', #'/fake_odom',  #'/odom', '/wheel_odom'
            self.odom_callback,
            10)
        
        self.odom_msg = Odometry()
        self.pc_odom_msg = Odometry()

        self.prev_cloud = None
        self.curr_vis_cloud = None
        self.prev_vis_cloud = None

        self.yaw = 0.0

        self.scan_count = 0

        # Initialize an identity matrix as the initial odometry
        self.odometry = np.eye(4)
        self.odometry2 = np.eye(4)
        self.odometry3 = np.eye(4)

        self.tranformation = np.eye(4)

        # Create a Visualizer object
        #self.vis = o3d.visualization.Visualizer()
        #self.vis.create_window()

        # Visualization variables
        #self.viz = Visualization(10, 10, 0.0, 0.0, dt = 1.0, g=0.8, h=0.8)
        timer_period = 0.2  # seconds
        #self.update_viz_timer = self.node.create_timer(timer_period, self.update_viz)

    
    def update_viz(self):
        _, _, theta = self.euler_from_quaternion(self.odom_msg.pose.pose.orientation)
        # Update viz
        self.viz.update_viz(self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y, theta, self.pc_odom_msg.pose.pose.position.x, self.pc_odom_msg.pose.pose.position.y , radians(self.yaw))

    def odom_callback(self, msg):

        self.odom_msg = msg

    def listener_callback(self, msg):

        self.scan_count += 1

        cloud = self.pointcloud2_to_pointcloud(msg)
        
        if self.prev_cloud is None:
            self.prev_cloud = cloud
            
            # Assuming 'pcd' is your initial PointCloud object
            #self.vis.add_geometry(cloud)
            
            return
        
        if self.scan_count % 5 == 0:
            if self.scan_count % 2 == 1:
                #self.vis.remove_geometry(self.curr_vis_cloud)
                #self.vis.remove_geometry(self.prev_vis_cloud)

                self.prev_vis_cloud = cloud
            
                #self.vis.add_geometry(self.prev_vis_cloud)
                

            else:
                #self.vis.remove_geometry(self.curr_vis_cloud)
                self.curr_vis_cloud = cloud
            
                #self.vis.add_geometry(self.curr_vis_cloud)
            
        
        #self.vis.poll_events()
        #self.vis.update_renderer()
                
        #self.vis.update_geometry(cloud)
        

        
        
        #if self.scan_count % 5 != 0:
        #    return
    
        # Perform ICP or other processing here
        #transformation, inlier_rmse = self.perform_icp(self.prev_cloud, cloud)
        #transformation2, inlier_rmse2 = self.perform_icp_ransac(self.prev_cloud, cloud)
        transformation3, inlier_rmse3 = self.perform_icp_point_to_plane(self.prev_cloud, cloud)
        self.tranformation = transformation3

        # Compute the inverse of the transformation
        #inverse_transformation3 = np.linalg.inv(transformation3)

        # Multiply the current odometry with the new transformation matrix to update the odometry
        #self.odometry = np.dot(self.odometry, transformation)
        #self.odometry2 = np.dot(self.odometry2, transformation2)
        self.odometry3 = np.dot(self.odometry3, transformation3)

        # Extract rotation matrix
        #R = self.odometry[0:3, 0:3]
        #euler = self.rotation_to_euler(R)

        #R2 = self.odometry2[0:3, 0:3]
        #euler2 = self.rotation_to_euler(R2)

        R3 = self.odometry3[0:3, 0:3]
        euler3 = self.rotation_to_euler(R3)

        #self.node.get_logger().info(str(R))
        #self.node.get_logger().info(str(euler))

        # Extract translation vector
        #T = self.odometry[0:3, 3]
        #T2 = self.odometry2[0:3, 3]
        T3 = self.odometry3[0:3, 3]

        _, _, theta = self.euler_from_quaternion(self.odom_msg.pose.pose.orientation)

        # Log the odometry
        self.node.get_logger().info('')
        #self.node.get_logger().info('Odometry: x: %f, y: %f, yaw: %f' % (T[0], T[1], euler[2]))
        self.node.get_logger().info('Wheel Odom: x: %f, y: %f, yaw: %f' % (self.odom_msg.pose.pose.position.x, self.odom_msg.pose.pose.position.y, degrees(theta)))
        
        #self.node.get_logger().info('Odometry2: x: %f, y: %f, yaw: %f' % (T2[0], T2[1], euler2[2]))
        self.node.get_logger().info('LiDAR Odom: x: %f, y: %f, yaw: %f' % (-T3[0], T3[1], -euler3[2]))

        self.publish_odometry(-T3[0], T3[1], T3[2], R3, -euler3[2])

        self.prev_cloud = cloud

    def publish_odometry(self, x, y, z, R, yaw):
            quat_x, quat_y, quat_z, quat_w = self.rotation_to_quaternion(R)

            self.pc_odom_msg = Odometry()
            self.pc_odom_msg.header.stamp = self.node.get_clock().now().to_msg()
            self.pc_odom_msg.header.frame_id = "odom"
            self.pc_odom_msg.child_frame_id = "base_link"
            self.pc_odom_msg.pose.pose.position.x = x
            self.pc_odom_msg.pose.pose.position.y = y
            self.pc_odom_msg.pose.pose.position.z = z
            self.pc_odom_msg.pose.pose.orientation.x = quat_x
            self.pc_odom_msg.pose.pose.orientation.y = quat_y
            self.pc_odom_msg.pose.pose.orientation.z = quat_z
            self.pc_odom_msg.pose.pose.orientation.w = quat_w

            self.yaw = yaw
            self.publisher_.publish(self.pc_odom_msg)
        

    def pointcloud2_to_pointcloud(self, msg):

        data = read_points(msg, skip_nans=True, field_names=("y", "x", "z"), x_range=(-10, 10), y_range=(-10, 10), z_range=(-10, 10))
        #for something in data:
        #    print(something)
        """ Converts a ROS LaserScan message to an Open3D PointCloud object. """
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(data)
        #print("Cloud size before voxel: " + str(len(cloud.points)))
        downcloud = cloud.voxel_down_sample(voxel_size=0.02) # Smaller size results in more points
        #print("Cloud size after voxel: " + str(len(downcloud.points)))
        return downcloud
    
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

        # Remove outliers
        #source = self.remove_outliers(source)
        #target = self.remove_outliers(target)

        # Estimate normals
        #search_param = o3d.geometry.KDTreeSearchParamRadius(radius=1)
        #source.estimate_normals(search_param=search_param)
        #target.estimate_normals(search_param=search_param)

        # Estimate normals
        o3d.geometry.PointCloud.estimate_normals(
            source,
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))

        o3d.geometry.PointCloud.estimate_normals(
            target,
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))

        threshold = 1.0  # Distance threshold for RANSAC
        trans_init = self.tranformation#np.eye(4)  # Initial transformation

        # Perform point-to-plane ICP
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source, target, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())

        return reg_p2p.transformation, reg_p2p.inlier_rmse

    def remove_outliers(self, point_cloud):
        cloud, ind = point_cloud.remove_statistical_outlier(nb_neighbors=20,
                                                        std_ratio=2.0)
        return cloud

    
    def rotation_to_euler(self, R):
        """ Converts a rotation matrix to Euler angles. """

        r = Rotation.from_matrix(R)
        euler = r.as_euler('xyz', degrees=True)
        return euler   
    
    def rotation_to_quaternion(self, R):
        """ Converts a rotation matrix to Euler angles. """

        r = Rotation.from_matrix(R)
        # Convert to quaternion
        quaternion = r.as_quat()
        return quaternion
    
    
    
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
