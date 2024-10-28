#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray, Pose

class MultiTB3PositionPublisher:
    def __init__(self):
        rospy.init_node('multi_tb3_position_publisher', anonymous=True)
        
        self.robots = [
            {"name": "tb3_0", "color": (1.0, 0.0, 0.0)},  # 红色
            {"name": "tb3_1", "color": (0.0, 1.0, 0.0)},  # 绿色
            {"name": "tb3_2", "color": (0.0, 0.0, 1.0)}   # 蓝色
        ]
        
        self.pose_array = PoseArray()
        self.pose_array.header.frame_id = "map"
        
        for robot in self.robots:
            robot["subscriber"] = rospy.Subscriber(f"/{robot['name']}/odom", Odometry, self.odom_callback, callback_args=robot['name'])
            robot["marker_publisher"] = rospy.Publisher(f"/{robot['name']}_position_marker", Marker, queue_size=10)
            robot["marker"] = self.create_marker(robot['color'])
            robot["pose"] = Pose()
        
        self.all_robots_publisher = rospy.Publisher("/all_robot_poses", PoseArray, queue_size=10)

    def create_marker(self, color):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.scale.x = 0.5  # 圆圈直径
        marker.scale.y = 0.5
        marker.scale.z = 0.01  # 圆圈厚度
        marker.color.a = 1.0  # 不透明度
        marker.color.r, marker.color.g, marker.color.b = color
        return marker

    def odom_callback(self, msg, robot_name):
        position = msg.pose.pose.position
        
        for robot in self.robots:
            if robot['name'] == robot_name:
                robot['marker'].pose.position = position
                robot['marker_publisher'].publish(robot['marker'])
                
                robot['pose'].position = position
                robot['pose'].orientation = msg.pose.pose.orientation
                
                rospy.loginfo(f"{robot_name} Position: x={position.x:.2f}, y={position.y:.2f}, z={position.z:.2f}")
                break
        
        self.publish_all_poses()

    def publish_all_poses(self):
        self.pose_array.header.stamp = rospy.Time.now()
        self.pose_array.poses = [robot['pose'] for robot in self.robots]
        self.all_robots_publisher.publish(self.pose_array)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        publisher = MultiTB3PositionPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        pass