#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
import tf
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PolygonStamped, Point32
from adaptive_clustering.msg import Bboxes2d
from nav_msgs.msg import Path
from std_msgs.msg import Bool
import shapely.geometry
import shapely.affinity


bboxes = []
global_path = Path()
euler = []

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
    global global_path
    global_path = data



def publish_obstacle_msg():
  pub = rospy.Publisher('/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
  marker_pub = rospy.Publisher('visualization_marker', MarkerArray, queue_size = 100)
  bool_pub = rospy.Publisher('/adaptive_clustering/is_bool',Bool,queue_size = 1)
  #pub = rospy.Publisher('/p3dx/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
  bboxes_sub = rospy.Subscriber('/adaptive_clustering/bboxes_2d',Bboxes2d,bboxes_callback)
  path_sub = rospy.Subscriber('/global_path',Path,path_callback)
  
  rospy.init_node("path_obstacle")


  obstacle_msg = ObstacleArrayMsg() 
  obstacle_msg.header.stamp = rospy.Time.now()
  obstacle_msg.header.frame_id = "map" # CHANGE HERE: odom/map

  listener = tf.TransformListener()
  is_bool = Bool()
  is_bool.data = False
  box_bool = False

  global_path_box_0 = 0
  global_path_box_5 = 0
  
  r = rospy.Rate(30) # 10hz

  while not rospy.is_shutdown():
    try:
        (trans,rot) = listener.lookupTransform('/map', '/velodyne', rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    marker_array = MarkerArray()
    box_num = 0

    for box in bboxes:
      if(len(global_path.poses) > 4):
        
        for j in range(0 , 5):

          dx = global_path.poses[j+1].pose.position.x - global_path.poses[j].pose.position.x
          dy = global_path.poses[j+1].pose.position.y - global_path.poses[j].pose.position.y
          theta = math.atan2(dy,dx)

          cx = (global_path.poses[j+1].pose.position.x + global_path.poses[j].pose.position.x)/2
          cy = (global_path.poses[j+1].pose.position.y + global_path.poses[j].pose.position.y)/2

          height = math.sqrt(dx**2+dy**2)
          r1 = RotatedRect(cx, cy, height, 2, theta)
          r2 = RotatedRect(math.cos(euler[2])*box.center.x-math.sin(euler[2])*box.center.y+trans[0], math.sin(euler[2])*box.center.x+math.cos(euler[2])*box.center.y+trans[1], box.size_y, box.size_x,euler[2]-math.pi/2)

          if(r1.intersection(r2).area != 0 and box.center.x > 0):
            is_bool.data = True
            box_bool = True

        if(box_bool):
          obstacle_msg.obstacles.append(ObstacleMsg())
          obstacle_msg.obstacles[box_num].id = box_num
          v1 = Point32()
          v1.x = box.center.x - box.size_x
          v1.y = box.center.y - box.size_y
          v2 = Point32()
          v2.x = box.center.x - box.size_x
          v2.y = box.center.y + box.size_y
          v3 = Point32()
          v3.x = box.center.x + box.size_x
          v3.y = box.center.y + box.size_y
          v4 = Point32()
          v4.x = box.center.x + box.size_x
          v4.y = box.center.y - box.size_y
          obstacle_msg.obstacles[box_num].polygon.points = [v1, v2, v3, v4]

          box_num += 1
          box_bool = False

    for j in range(0 , 5):
      marker = Marker()
      marker.id = j
      marker.header.frame_id = "/velodyne"
      marker.type = marker.LINE_STRIP
      marker.scale.x = 1
      marker.scale.y = 1
      marker.scale.z = 1
      marker.color.a = 1.0
      marker.pose.orientation.w = 1.0
      marker.lifetime = rospy.Duration(0.01)
      marker.pose.orientation.x = 0
      marker.pose.orientation.y = 0
      marker.pose.orientation.z = 0
      marker.pose.orientation.w = 0

      dx = global_path.poses[j+1].pose.position.x - global_path.poses[j].pose.position.x
      dy = global_path.poses[j+1].pose.position.y - global_path.poses[j].pose.position.y
      theta = math.atan2(dy,dx)

      cx = (global_path.poses[j+1].pose.position.x + global_path.poses[j].pose.position.x)/2
      cy = (global_path.poses[j+1].pose.position.y + global_path.poses[j].pose.position.y)/2

      height = math.sqrt(dx**2+dy**2)
      r1 = RotatedRect(cx, cy, height, 2, theta)

      for k in range(len(r1.get_contour().exterior.coords)):
        p = Point32()
        p.x = math.cos(-euler[2])*(r1.get_contour().exterior.coords[k][0]- trans[0])-math.sin(-euler[2])*(r1.get_contour().exterior.coords[k][1]- trans[1])
        p.y = math.sin(-euler[2])*(r1.get_contour().exterior.coords[k][0]- trans[0])+math.cos(-euler[2])*(r1.get_contour().exterior.coords[k][1]- trans[1])
        print(k,p)
        marker.points.append(p)

      marker_array.markers.append(marker)   

      # obstacle_msg.obstacles.append(ObstacleMsg())
      # obstacle_msg.obstacles[box_num].id = box_num
      # v1 = Point32()
      # v1.x = math.cos(-euler[2])*(r1.get_contour().exterior.coords[1][0]- trans[0])-math.sin(-euler[2])*(r1.get_contour().exterior.coords[1][1]- trans[1])
      # v1.y = math.sin(-euler[2])*(r1.get_contour().exterior.coords[1][0]- trans[0])+math.cos(-euler[2])*(r1.get_contour().exterior.coords[1][1]- trans[1])
      # v2 = Point32()
      # v2.x = math.cos(-euler[2])*(r1.get_contour().exterior.coords[2][0]- trans[0])-math.sin(-euler[2])*(r1.get_contour().exterior.coords[2][1]- trans[1])
      # v2.y = math.sin(-euler[2])*(r1.get_contour().exterior.coords[2][0]- trans[0])+math.cos(-euler[2])*(r1.get_contour().exterior.coords[2][1]- trans[1])
      # obstacle_msg.obstacles[box_num].polygon.points = [v1, v2]
      # box_num += 1

      # obstacle_msg.obstacles.append(ObstacleMsg())
      # obstacle_msg.obstacles[box_num].id = box_num
      # v3 = Point32()
      # v3.x = math.cos(-euler[2])*(r1.get_contour().exterior.coords[0][0]- trans[0])-math.sin(-euler[2])*(r1.get_contour().exterior.coords[0][1]- trans[1])
      # v3.y = math.sin(-euler[2])*(r1.get_contour().exterior.coords[0][0]- trans[0])+math.cos(-euler[2])*(r1.get_contour().exterior.coords[0][1]- trans[1])
      # v4 = Point32()
      # v4.x = math.cos(-euler[2])*(r1.get_contour().exterior.coords[3][0]- trans[0])-math.sin(-euler[2])*(r1.get_contour().exterior.coords[3][1]- trans[1])
      # v4.y = math.sin(-euler[2])*(r1.get_contour().exterior.coords[3][0]- trans[0])+math.cos(-euler[2])*(r1.get_contour().exterior.coords[3][1]- trans[1])
      # obstacle_msg.obstacles[box_num].polygon.points = [v3, v4]
      # box_num += 1

      if(j == 0):
        global_path_box_0 = r1
      if(j == 4):
        global_path_box_5 = r1

    # obstacle_msg.obstacles.append(ObstacleMsg())
    # obstacle_msg.obstacles[box_num].id = box_num
    # v1 = Point32()
    # v1.x = math.cos(-euler[2])*(global_path_box_0.get_contour().exterior.coords[2][0]- trans[0])-math.sin(-euler[2])*(global_path_box_0.get_contour().exterior.coords[2][1]- trans[1])
    # v1.y = math.sin(-euler[2])*(global_path_box_0.get_contour().exterior.coords[2][0]- trans[0])+math.cos(-euler[2])*(global_path_box_0.get_contour().exterior.coords[2][1]- trans[1])
    # v2 = Point32()
    # v2.x = math.cos(-euler[2])*(global_path_box_5.get_contour().exterior.coords[1][0]- trans[0])-math.sin(-euler[2])*(global_path_box_5.get_contour().exterior.coords[1][1]- trans[1])
    # v2.y = math.sin(-euler[2])*(global_path_box_5.get_contour().exterior.coords[1][0]- trans[0])+math.cos(-euler[2])*(global_path_box_5.get_contour().exterior.coords[1][1]- trans[1])
    # obstacle_msg.obstacles[box_num].polygon.points = [v1, v2]
    # box_num += 1

    obstacle_msg.obstacles.append(ObstacleMsg())
    obstacle_msg.obstacles[box_num].id = box_num
    v3 = Point32()
    v3.x = math.cos(-euler[2])*(global_path_box_0.get_contour().exterior.coords[3][0]- trans[0])-math.sin(-euler[2])*(global_path_box_0.get_contour().exterior.coords[3][1]- trans[1])
    v3.y = math.sin(-euler[2])*(global_path_box_0.get_contour().exterior.coords[3][0]- trans[0])+math.cos(-euler[2])*(global_path_box_0.get_contour().exterior.coords[3][1]- trans[1])
    v4 = Point32()
    v4.x = math.cos(-euler[2])*(global_path_box_5.get_contour().exterior.coords[0][0]- trans[0])-math.sin(-euler[2])*(global_path_box_5.get_contour().exterior.coords[0][1]- trans[1])
    v4.y = math.sin(-euler[2])*(global_path_box_5.get_contour().exterior.coords[0][0]- trans[0])+math.cos(-euler[2])*(global_path_box_5.get_contour().exterior.coords[0][1]- trans[1])
    v3_ = Point32()
    v3_.x = v3.x + 0.01
    v3_.y = v3.y + 0.01
    v4_ = Point32()
    v4_.x = v4.x + 0.01
    v4_.y = v4.y + 0.01
    obstacle_msg.obstacles[box_num].polygon.points = [v3, v4]
    box_num += 1


    marker_pub.publish(marker_array)
    bool_pub.publish(is_bool)
    is_bool.data = False
    pub.publish(obstacle_msg)
    print(len(obstacle_msg.obstacles))
    obstacle_msg = ObstacleArrayMsg() 
    obstacle_msg.header.stamp = rospy.Time.now()
    obstacle_msg.header.frame_id = "velodyne" # CHANGE HERE: odom/map
    # print("ok")
    r.sleep()



if __name__ == '__main__': 
  try:
    publish_obstacle_msg()
  except rospy.ROSInterruptException:
    pass