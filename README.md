有3種模式

1. 只有對gmapping做膨脹
   roslaunch ros_multi_tb3 multiple_tb3_house_dilation.launch
2. 膨脹之後利用robot_filter直接對 mergemap 之後的全域地圖進行修改
   roslaunch ros_multi_tb3 multiple_tb3_house_robotfilter.launch
3. 在 mapmerge 之前清除掉各自地圖同伴位置附近的障礙物 再進行 mapmerge
   roslaunch ros_multi_tb3 multi_robot.launch




