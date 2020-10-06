#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
import tf
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32
from adaptive_clustering.msg import Bboxes2d
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from geometry_msgs.msg import Point , PoseStamped
import shapely.geometry
import shapely.affinity
import numpy as np

bboxes = []
global_path = Path()
euler = []
path_list = []
path1_pub = rospy.Publisher('/adaptive_clustering/Path_1',Path,queue_size = 1)

class RotatedRect:
    def __init__(self, cx, cy, w, h, angle):
        self.cx = cx
        self.cy = cy
        self.w = w
        self.h = h
        self.angle = angle*(180/math.pi)

    def get_contour(self):
        w = self.w
        h = self.h
        c = shapely.geometry.box(-w/2.0, -h/2.0, w/2.0, h/2.0)
        rc = shapely.affinity.rotate(c, self.angle)
        return shapely.affinity.translate(rc, self.cx, self.cy)

    def intersection(self, other):
        return self.get_contour().intersection(other.get_contour())

def bboxes_callback(data):
  global bboxes
  bboxes = data.Bboxes2d

def path_callback(data):
    global path_list
    global final_path
    TEB_path = data

    TEB_path_ = Path()
    path_length = len(TEB_path.poses)
    const_num = float(path_length)/20
    const_list = list(np.arange(0,path_length,const_num).astype(np.int64))

    for i in const_list:
      TEB_path_.header.stamp = rospy.Time.now()
      TEB_path_.header.frame_id = "map"

      pose = PoseStamped()
      pose.header.stamp = rospy.Time.now()
      pose.header.frame_id = "map"

      pose.pose.position.x = TEB_path.poses[i].pose.position.x - odom_pose.position.x
      pose.pose.position.y = TEB_path.poses[i].pose.position.y - odom_pose.position.y
      pose.pose.position.z = 0

      pose.pose.orientation.x = 0
      pose.pose.orientation.y = 0
      pose.pose.orientation.z = 0
      pose.pose.orientation.w = 1
      if(i == const_list[-1]):
        print(pose.pose.position)

      TEB_path_.poses.append(pose)  
         
    path_list.append(TEB_path_)

    final_path = Path()
    for i in range(20):
      x_ = 0
      y_ = 0

      for j in range(len(path_list)):
        x_ += path_list[j].poses[i].pose.position.x
        y_ += path_list[j].poses[i].pose.position.y

      final_path.header.stamp = rospy.Time.now()
      final_path.header.frame_id = "map"

      pose = PoseStamped()
      pose.header.stamp = rospy.Time.now()
      pose.header.frame_id = "map"

      pose.pose.position.x = float(x_)/len(path_list) + odom_pose.position.x
      pose.pose.position.y = float(y_)/len(path_list) + odom_pose.position.y
      pose.pose.position.z = 0

      pose.pose.orientation.x = 0
      pose.pose.orientation.y = 0
      pose.pose.orientation.z = 0
      pose.pose.orientation.w = 1
      
      if(i == 19):
        print("x",float(x_)/20)
        print("y",float(y_)/20)

      final_path.poses.append(pose)  

    path1_pub.publish(final_path)
      
    if(len(path_list) == 25):
      del path_list[0]

def step_callback(data):
  global step_num
  step_num = data.pose.position.z

def marker_callback(data):
  global container_points
  # rospy.logwarn("1 is obstacle")
  # print(data)
  if(data.ns == "TebContainer"):
    container_points = data.points
  # Marker().ns

def odom_callback(data):
    global odom_pose
    odom_pose = data.pose.pose


def publish_obstacle_msg():
  rospy.init_node("path_")

  pub = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
  marker_pub = rospy.Publisher('visualization_marker', MarkerArray, queue_size = 100)
  bool_pub = rospy.Publisher('/adaptive_clustering/is_bool',Bool,queue_size = 1)

  #pub = rospy.Publisher('/p3dx/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
  bboxes_sub = rospy.Subscriber('/adaptive_clustering/bboxes_2d',Bboxes2d,bboxes_callback)
  path_sub = rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan',Path,path_callback)
  step_sub = rospy.Subscriber('current_step', PoseStamped, step_callback, queue_size=1)
  marker_sub = rospy.Subscriber('/move_base/TebLocalPlannerROS/teb_markers', Marker,marker_callback, queue_size = 100)
  odom_sub = rospy.Subscriber('odom', Odometry, odom_callback, queue_size=1)


  obstacle_msg = ObstacleArrayMsg() 
  obstacle_msg.header.stamp = rospy.Time.now()
  obstacle_msg.header.frame_id = "map" # CHANGE HERE: odom/map

  # listener = tf.TransformListener()
  is_bool = Bool()
  is_bool.data = False
  box_bool = False

  global_path_box_0 = 0
  global_path_box_5 = 0

  r_gpath_list = [0,0,0,0,0]
  r_cpath_list = [0,0,0,0,0]
  
  r = rospy.Rate(144) # 10hz

  final_path = Path()

  while not rospy.is_shutdown():
    # print(len(path_list))
    TEB_path_ = Path()
      
    # for i in range(len(TEB_path.poses)):
    #   TEB_path_.header.stamp = rospy.Time.now()
    #   TEB_path_.header.frame_id = "map"

    #   pose = PoseStamped()
    #   pose.header.stamp = rospy.Time.now()
    #   pose.header.frame_id = "map"

    #   pose.pose.position.x = TEB_path.poses[i].pose.position.x - odom_pose.position.x
    #   pose.pose.position.y = TEB_path.poses[i].pose.position.y - odom_pose.position.y
    #   pose.pose.position.z = 0

    #   pose.pose.orientation.x = 0
    #   pose.pose.orientation.y = 0
    #   pose.pose.orientation.z = 0
    #   pose.pose.orientation.w = 1

    #   TEB_path_.poses.append(pose)      

    # path_list.append(TEB_path_)
    # if(len(path_list) == 50):
    #   del path_list[0]

    # container_path = Path()
    # for point in container_points[:40]:
    #   if(odom_pose.position.x == point.x and odom_pose.position.y == point.y):
    #     print("ok")
    #   container_path.header.stamp = rospy.Time.now()
    #   container_path.header.frame_id = "map"

    #   pose = PoseStamped()
    #   pose.header.stamp = rospy.Time.now()
    #   pose.header.frame_id = "map"

    #   pose.pose.position.x = point.x
    #   pose.pose.position.y = point.y
    #   pose.pose.position.z = 0

    #   pose.pose.orientation.x = 0
    #   pose.pose.orientation.y = 0
    #   pose.pose.orientation.z = 0
    #   pose.pose.orientation.w = 1

    #   container_path.poses.append(pose)
  
    # path1_pub.publish(final_path)

    # try:
    #     (trans,rot) = listener.lookupTransform('/map', '/velodyne', rospy.Time(0))
    #     euler = tf.transformations.euler_from_quaternion(rot)
    # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #     continue

    # marker_array = MarkerArray()
    # box_num = 0
    # if(len(global_path.poses) > 4):
    #   for i in range(0,5):
    #     dx = global_path.poses[i+1].pose.position.x - global_path.poses[i].pose.position.x
    #     dy = global_path.poses[i+1].pose.position.y - global_path.poses[i].pose.position.y
    #     theta = math.atan2(dy,dx)

    #     cx = (global_path.poses[i+1].pose.position.x + global_path.poses[i].pose.position.x)/2
    #     cy = (global_path.poses[i+1].pose.position.y + global_path.poses[i].pose.position.y)/2

    #     height = math.sqrt(dx**2+dy**2)
    #     r_gpath = RotatedRect(cx, cy, height, 2.5, theta)

    #     r_gpath_list[i] = r_gpath
    #     if(step_num < 170):
    #       cx_ = (r_gpath.get_contour().exterior.coords[1][0] + r_gpath.get_contour().exterior.coords[2][0])/2
    #       cy_ = (r_gpath.get_contour().exterior.coords[1][1] + r_gpath.get_contour().exterior.coords[2][1])/2
    #     else:
    #       cx_ = (r_gpath.get_contour().exterior.coords[0][0] + r_gpath.get_contour().exterior.coords[3][0])/2
    #       cy_ = (r_gpath.get_contour().exterior.coords[0][1] + r_gpath.get_contour().exterior.coords[3][1])/2          

    #     r_cpath = RotatedRect(cx_, cy_, height, 5, theta)        

    #     r_cpath_list[i] = r_cpath

    #     marker = Marker()
    #     marker.id = i
    #     marker.header.frame_id = "/map"
    #     marker.type = marker.LINE_STRIP
    #     marker.scale.x = 1
    #     marker.scale.y = 1
    #     marker.scale.z = 1
    #     marker.color.a = 1.0
    #     marker.pose.orientation.w = 1.0
    #     marker.lifetime = rospy.Duration(0.01)
    #     marker.pose.orientation.x = 0
    #     marker.pose.orientation.y = 0
    #     marker.pose.orientation.z = 0
    #     marker.pose.orientation.w = 0

    #     for k in range(len(r_cpath.get_contour().exterior.coords)):
    #       p = Point32()
    #       p.x = r_cpath.get_contour().exterior.coords[k][0]#math.cos(-euler[2])*(r1.get_contour().exterior.coords[k][0]- trans[0])-math.sin(-euler[2])*(r1.get_contour().exterior.coords[k][1]- trans[1])
    #       p.y = r_cpath.get_contour().exterior.coords[k][1]#math.sin(-euler[2])*(r1.get_contour().exterior.coords[k][0]- trans[0])+math.cos(-euler[2])*(r1.get_contour().exterior.coords[k][1]- trans[1])
    #       # print(k,p)
    #       marker.points.append(p)

    #     marker_array.markers.append(marker)  

    #     if(i == 0):
    #       global_path_box_0 = r_gpath
    #     if(i == 4):
    #       global_path_box_5 = r_gpath

    # for box in bboxes:
    #   if(len(global_path.poses) > 4):
        
    #     for j in range(0 , 5):
    #       r_gpath = r_gpath_list[j]

    #       cx_obj = math.cos(euler[2])*box.center.x-math.sin(euler[2])*box.center.y+trans[0]
    #       cy_obj = math.sin(euler[2])*box.center.x+math.cos(euler[2])*box.center.y+trans[1]
    #       r_obj = RotatedRect(cx_obj, cy_obj, box.size_y, box.size_x,euler[2]-math.pi/2)
    #       point_obj = shapely.geometry.Point(cx_obj,cy_obj)

    #       r_cpath = r_cpath_list[j]
          
    #       if(r_cpath.get_contour().contains(point_obj)):
    #         is_bool.data = True
    #         box_bool = True

    #     if(box_bool):
    #       obstacle_msg.obstacles.append(ObstacleMsg())
    #       obstacle_msg.obstacles[box_num].id = box_num
    #       v1 = Point32()
    #       v1.x = box.center.x - box.size_x/2
    #       v1.y = box.center.y - box.size_y/2
    #       v2 = Point32()
    #       v2.x = box.center.x - box.size_x/2
    #       v2.y = box.center.y + box.size_y/2
    #       v3 = Point32()
    #       v3.x = box.center.x + box.size_x/2
    #       v3.y = box.center.y + box.size_y/2
    #       v4 = Point32()
    #       v4.x = box.center.x + box.size_x/2
    #       v4.y = box.center.y - box.size_y/2
    #       obstacle_msg.obstacles[box_num].polygon.points = [v1, v2, v3, v4]

    #       box_num += 1
    #       box_bool = False

    # # for j in range(0 , 5):
    # #   marker = Marker()
    # #   marker.id = j
    # #   marker.header.frame_id = "/map"
    # #   marker.type = marker.LINE_STRIP
    # #   marker.scale.x = 1
    # #   marker.scale.y = 1
    # #   marker.scale.z = 1
    # #   marker.color.a = 1.0
    # #   marker.pose.orientation.w = 1.0
    # #   marker.lifetime = rospy.Duration(0.01)
    # #   marker.pose.orientation.x = 0
    # #   marker.pose.orientation.y = 0
    # #   marker.pose.orientation.z = 0
    # #   marker.pose.orientation.w = 0

    # #   r_gpath = r_gpath_list[j]
    # #   r_cpath = r_cpath_list[j]

    # #   for k in range(len(r_cpath.get_contour().exterior.coords)):
    # #     p = Point32()
    # #     p.x = r_cpath.get_contour().exterior.coords[k][0]#math.cos(-euler[2])*(r1.get_contour().exterior.coords[k][0]- trans[0])-math.sin(-euler[2])*(r1.get_contour().exterior.coords[k][1]- trans[1])
    # #     p.y = r_cpath.get_contour().exterior.coords[k][1]#math.sin(-euler[2])*(r1.get_contour().exterior.coords[k][0]- trans[0])+math.cos(-euler[2])*(r1.get_contour().exterior.coords[k][1]- trans[1])
    # #     # print(k,p)
    # #     marker.points.append(p)

    # #   # marker_array.markers.append(marker)   

    # #   # obstacle_msg.obstacles.append(ObstacleMsg())
    # #   # obstacle_msg.obstacles[box_num].id = box_num
    # #   # v1 = Point32()
    # #   # v1.x = math.cos(-euler[2])*(r1.get_contour().exterior.coords[1][0]- trans[0])-math.sin(-euler[2])*(r1.get_contour().exterior.coords[1][1]- trans[1])
    # #   # v1.y = math.sin(-euler[2])*(r1.get_contour().exterior.coords[1][0]- trans[0])+math.cos(-euler[2])*(r1.get_contour().exterior.coords[1][1]- trans[1])
    # #   # v2 = Point32()
    # #   # v2.x = math.cos(-euler[2])*(r1.get_contour().exterior.coords[2][0]- trans[0])-math.sin(-euler[2])*(r1.get_contour().exterior.coords[2][1]- trans[1])
    # #   # v2.y = math.sin(-euler[2])*(r1.get_contour().exterior.coords[2][0]- trans[0])+math.cos(-euler[2])*(r1.get_contour().exterior.coords[2][1]- trans[1])
    # #   # obstacle_msg.obstacles[box_num].polygon.points = [v1, v2]
    # #   # box_num += 1

    # #   # obstacle_msg.obstacles.append(ObstacleMsg())
    # #   # obstacle_msg.obstacles[box_num].id = box_num
    # #   # v3 = Point32()
    # #   # v3.x = math.cos(-euler[2])*(r1.get_contour().exterior.coords[0][0]- trans[0])-math.sin(-euler[2])*(r1.get_contour().exterior.coords[0][1]- trans[1])
    # #   # v3.y = math.sin(-euler[2])*(r1.get_contour().exterior.coords[0][0]- trans[0])+math.cos(-euler[2])*(r1.get_contour().exterior.coords[0][1]- trans[1])
    # #   # v4 = Point32()
    # #   # v4.x = math.cos(-euler[2])*(r1.get_contour().exterior.coords[3][0]- trans[0])-math.sin(-euler[2])*(r1.get_contour().exterior.coords[3][1]- trans[1])
    # #   # v4.y = math.sin(-euler[2])*(r1.get_contour().exterior.coords[3][0]- trans[0])+math.cos(-euler[2])*(r1.get_contour().exterior.coords[3][1]- trans[1])
    # #   # obstacle_msg.obstacles[box_num].polygon.points = [v3, v4]
    # #   # box_num += 1

    # #   if(j == 0):
    # #     global_path_box_0 = r_gpath
    # #   if(j == 4):
    # #     global_path_box_5 = r_gpath

    # if(step_num > 170):
    #   obstacle_msg.obstacles.append(ObstacleMsg())
    #   obstacle_msg.obstacles[box_num].id = box_num
    #   v1 = Point32()
    #   v1.x = math.cos(-euler[2])*(global_path_box_0.get_contour().exterior.coords[2][0]- trans[0])-math.sin(-euler[2])*(global_path_box_0.get_contour().exterior.coords[2][1]- trans[1])
    #   v1.y = math.sin(-euler[2])*(global_path_box_0.get_contour().exterior.coords[2][0]- trans[0])+math.cos(-euler[2])*(global_path_box_0.get_contour().exterior.coords[2][1]- trans[1])
    #   v2 = Point32()
    #   v2.x = math.cos(-euler[2])*(global_path_box_5.get_contour().exterior.coords[1][0]- trans[0])-math.sin(-euler[2])*(global_path_box_5.get_contour().exterior.coords[1][1]- trans[1])
    #   v2.y = math.sin(-euler[2])*(global_path_box_5.get_contour().exterior.coords[1][0]- trans[0])+math.cos(-euler[2])*(global_path_box_5.get_contour().exterior.coords[1][1]- trans[1])
    #   obstacle_msg.obstacles[box_num].polygon.points = [v1, v2]
    #   box_num += 1
    
    # else:
    #   obstacle_msg.obstacles.append(ObstacleMsg())
    #   obstacle_msg.obstacles[box_num].id = box_num
    #   v3 = Point32()
    #   v3.x = math.cos(-euler[2])*(global_path_box_0.get_contour().exterior.coords[3][0]- trans[0])-math.sin(-euler[2])*(global_path_box_0.get_contour().exterior.coords[3][1]- trans[1])
    #   v3.y = math.sin(-euler[2])*(global_path_box_0.get_contour().exterior.coords[3][0]- trans[0])+math.cos(-euler[2])*(global_path_box_0.get_contour().exterior.coords[3][1]- trans[1])
    #   v4 = Point32()
    #   v4.x = math.cos(-euler[2])*(global_path_box_5.get_contour().exterior.coords[0][0]- trans[0])-math.sin(-euler[2])*(global_path_box_5.get_contour().exterior.coords[0][1]- trans[1])
    #   v4.y = math.sin(-euler[2])*(global_path_box_5.get_contour().exterior.coords[0][0]- trans[0])+math.cos(-euler[2])*(global_path_box_5.get_contour().exterior.coords[0][1]- trans[1])
    #   v3_ = Point32()
    #   v3_.x = v3.x + 0.01
    #   v3_.y = v3.y + 0.01
    #   v4_ = Point32()
    #   v4_.x = v4.x + 0.01
    #   v4_.y = v4.y + 0.01
    #   obstacle_msg.obstacles[box_num].polygon.points = [v3, v4,v3_,v4_]
    #   box_num += 1


    # marker_pub.publish(marker_array)
    # bool_pub.publish(is_bool)
    # is_bool.data = False
    # pub.publish(obstacle_msg)
    # print("obs:",len(obstacle_msg.obstacles))
    # obstacle_msg = ObstacleArrayMsg() 
    # obstacle_msg.header.stamp = rospy.Time.now()
    # obstacle_msg.header.frame_id = "velodyne" # CHANGE HERE: odom/map
    # # print("ok")
    r.sleep()



if __name__ == '__main__': 
  try:
    publish_obstacle_msg()
  except rospy.ROSInterruptException:
    pass