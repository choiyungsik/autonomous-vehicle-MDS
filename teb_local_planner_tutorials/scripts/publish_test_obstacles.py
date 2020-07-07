#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32
from adaptive_clustering.msg import Bboxes2d

bboxes = []

def bboxes_callback(data):
  global bboxes
  bboxes = data.Bboxes2d

def publish_obstacle_msg():
  pub = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
  #pub = rospy.Publisher('/p3dx/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
  bboxes_sub = rospy.Subscriber('/adaptive_clustering/bboxes_2d',Bboxes2d,bboxes_callback)
  rospy.init_node("test_obstacle_msg")


  obstacle_msg = ObstacleArrayMsg() 
  obstacle_msg.header.stamp = rospy.Time.now()
  obstacle_msg.header.frame_id = "map" # CHANGE HERE: odom/map


  r = rospy.Rate(10) # 10hz

  while not rospy.is_shutdown():

    for i,box in enumerate(bboxes):
      obstacle_msg.obstacles.append(ObstacleMsg())
      obstacle_msg.obstacles[i].id = i
      v1 = Point32()
      v1.x = box.center.x - box.size_x/2
      v1.y = box.center.y - box.size_y/2
      v2 = Point32()
      v2.x = box.center.x - box.size_x/2
      v2.y = box.center.y + box.size_y/2
      v3 = Point32()
      v3.x = box.center.x + box.size_x/2
      v3.y = box.center.y + box.size_y/2
      v4 = Point32()
      v4.x = box.center.x + box.size_x/2
      v4.y = box.center.y - box.size_y/2
      obstacle_msg.obstacles[i].polygon.points = [v1, v2, v3, v4]
      print(len(bboxes))

    pub.publish(obstacle_msg)
    obstacle_msg = ObstacleArrayMsg() 
    obstacle_msg.header.stamp = rospy.Time.now()
    obstacle_msg.header.frame_id = "velodyne" # CHANGE HERE: odom/map
    
    r.sleep()



if __name__ == '__main__': 
  try:
    publish_obstacle_msg()
  except rospy.ROSInterruptException:
    pass

