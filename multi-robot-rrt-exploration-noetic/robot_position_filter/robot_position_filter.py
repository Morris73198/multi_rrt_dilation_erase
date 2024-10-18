#!/usr/bin/env python3
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from collections import deque

class RobotPositionFilter:
    def __init__(self):
        rospy.init_node('robot_position_filter')
        rospy.sleep(5)  # 等待其他節點初始化
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.robot_names = rospy.get_param('~robot_names', ['tb3_0', 'tb3_1', 'tb3_2'])
        self.global_frame = rospy.get_param('~global_frame', 'map')
        
        self.robot_trajectories = {name: deque(maxlen=1000) for name in self.robot_names}
        
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.filtered_map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
        
        self.update_rate = rospy.Rate(10)  # 10Hz
        rospy.Timer(rospy.Duration(0.1), self.update_robot_positions)
        
    def update_robot_positions(self, event):
        for robot_name in self.robot_names:
            try:
                trans = self.tf_buffer.lookup_transform(self.global_frame, f'{robot_name}/base_footprint', rospy.Time())
                position = (trans.transform.translation.x, trans.transform.translation.y)
                self.robot_trajectories[robot_name].append(position)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"Failed to get transform for {robot_name}: {e}")
    
    def map_callback(self, map_msg):
        filtered_map = self.filter_map(map_msg)
        self.filtered_map_pub.publish(filtered_map)
        
    def filter_map(self, map_msg):
        filtered_map = OccupancyGrid()
        filtered_map.header = map_msg.header
        filtered_map.info = map_msg.info
        
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y
        
        for i in range(len(map_msg.data)):
            x = i % map_msg.info.width
            y = i // map_msg.info.width
            world_x = x * resolution + origin_x
            world_y = y * resolution + origin_y
            
            should_clear = False
            for trajectory in self.robot_trajectories.values():
                for px, py in trajectory:
                    if abs(world_x - px) < resolution and abs(world_y - py) < resolution:
                        should_clear = True
                        break
                if should_clear:
                    break
            
            if should_clear:
                filtered_map.data.append(0)  # Free space
            else:
                filtered_map.data.append(map_msg.data[i])
        
        return filtered_map

if __name__ == '__main__':
    try:
        RobotPositionFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
