#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import yaml 
import os
from rclpy.node import Node 
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from tf_transformations import quaternion_matrix
from scipy.spatial.transform import Rotation
import numpy as np 
from rclpy.executors import SingleThreadedExecutor
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class ArucoSubPub(Node):
    def __init__(self):
        super().__init__('aruco_sub_pub')
        self.subscription = self.create_subscription(PoseStamped, '/aruco_single/pose',self.aruco_callback,10)
        self.tf_static_broadcaster_ = StaticTransformBroadcaster(self)
        self.subscription
    
    def aruco_callback(self,msg):
        # Estrai posizione
        x_aruco = msg.pose.position.x
        y_aruco = msg.pose.position.y
        z_aruco = msg.pose.position.z

        # Estrai orientamento
        qx_aruco = msg.pose.orientation.x
        qy_aruco = msg.pose.orientation.y
        qz_aruco = msg.pose.orientation.z
        qw_aruco = msg.pose.orientation.w

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id="map"
        t.child_frame_id = "aruco_marker_frame"

      
        #self.get_logger().info(f'Position: [x={x_aruco:.2f}, y={y_aruco:.2f}, z={z_aruco:.2f}]')
        #self.get_logger().info(f'Orientation: [qx={qx_aruco:.2f}, qy={qy_aruco:.2f}, qz={qz_aruco:.2f}, qw={qw_aruco:.2f}]')
        aruco_position = [x_aruco, y_aruco, z_aruco]
        aruco_rotation_matrix = quaternion_matrix([qx_aruco, qy_aruco, qz_aruco, qw_aruco])[:3, :3]

        aruco_trasformation_matrix = np.eye(4)
        aruco_trasformation_matrix[:3,:3] = aruco_rotation_matrix
        aruco_trasformation_matrix[:3, 3] = aruco_position 
        #print(aruco_trasformation_matrix)

        rot_mat = quaternion_matrix([-0.5, 0.5, -0.5, 0.5])
        rot_mat[:3,3]= np.array([0.099, 0.0, 0.135])
        

        aruco_camera_link = np.dot(rot_mat,aruco_trasformation_matrix)
        #print(aruco_camera_link)

        robot_position = np.array([3.46,-0.75, 0.0])
        robot_matrix  = np.eye(4)
        robot_matrix[:3,:3] = quaternion_matrix([0.0, 0.0, 0.0, 1.0])[:3,:3]
        robot_matrix[:3,3] = robot_position

        aruco_map = np.dot(robot_matrix,aruco_camera_link)

        r = Rotation.from_matrix(aruco_map[:3,:3])
        quaternion = r.as_quat()
        aruco_pos_map = aruco_map[:3,3]

        t.transform.translation.x = aruco_pos_map[0]
        t.transform.translation.y = aruco_pos_map[1]
        t.transform.translation.z = aruco_pos_map[2]

        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        self.tf_static_broadcaster_.sendTransform(t)


        #self.get_logger().info(f'Position: [x={aruco_pos_map[0]:.2f}, y={aruco_pos_map[1]:.2f}, z={aruco_pos_map[2]:.2f}]')
        #self.get_logger().info(f'Orientation: [qx={quaternion[0]:.2f}, qy={quaternion[1]:.2f}, qz={quaternion[2]:.2f}, qw={quaternion[3]:.2f}]')

        
yaml_path = get_package_share_directory ('rl_fra2mo_description')
yaml_file = os.path.join(yaml_path,"config", "waypoints.yaml")

with open(yaml_file, 'r') as file:
    waypoints = yaml.safe_load(file)



def main():
    rclpy.init()
    navigator = BasicNavigator()
    
    def create_pose(transform):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = transform["position"]["x"]
        pose.pose.position.y = transform["position"]["y"]
        pose.pose.position.z = transform["position"]["z"]
        pose.pose.orientation.x = transform["orientation"]["x"]
        pose.pose.orientation.y = transform["orientation"]["y"]
        pose.pose.orientation.z = transform["orientation"]["z"]
        pose.pose.orientation.w = transform["orientation"]["w"]
        return pose
    

    # Ordine desiderato dei goal
    goal_order=["goal_aruco1"]
    
    # Riordina i goal in base a goal_order
    ordered_goals = [goal for name in goal_order for goal in waypoints["waypoints"] if goal["goal"] == name]

    # Crea una lista di PoseStamped dai goal ordinati
    goal_poses = list(map(create_pose, ordered_goals))

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer="smoother_server")

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600):
                navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')
    
    node_aruco = ArucoSubPub()
    executor = SingleThreadedExecutor()
    executor.add_node(node_aruco)

    executor.spin_once(timeout_sec=5)
    node_aruco.destroy_node()

     # Ordine desiderato dei goal
    goal_order=["goal_aruco2"]
    
    # Riordina i goal in base a goal_order
    ordered_goals = [goal for name in goal_order for goal in waypoints["waypoints"] if goal["goal"] == name]

    # Crea una lista di PoseStamped dai goal ordinati
    goal_poses = list(map(create_pose, ordered_goals))

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer="smoother_server")

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    while not navigator.isTaskComplete():
        
        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600):
                navigator.cancelTask()
 
    #node_aruco.destroy_node()
    rclpy.shutdown()
    exit(0)


if __name__ == '__main__':
    main()