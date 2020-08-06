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
from std_msgs.msg import Bool,Int16MultiArray
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



def publish_obstacle_msg():
  marker_pub = rospy.Publisher('visualization_marker_parking', MarkerArray, queue_size = 10)
  empty_pub = rospy.Publisher("empty_space",Int16MultiArray,queue_size=1)
  #pub = rospy.Publisher('/p3dx/move_base/TebLocalPlannerROS/obstacles', ObstacleArrayMsg, queue_size=1)
  bboxes_sub = rospy.Subscriber('/adaptive_clustering/bboxes_2d',Bboxes2d,bboxes_callback)
  
  rospy.init_node("Parking")
  listener = tf.TransformListener()
  
  r = rospy.Rate(30) # 10hz

  parking_spot = [[-5,1],[0,0],[5,-1]]
  empty_spot_array = Int16MultiArray()
  empty_spot_array.data = [0,0,0]

  while not rospy.is_shutdown():
    empty_spot_array.data = [0,0,0]
    try:
        (trans,rot) = listener.lookupTransform('/map', '/velodyne', rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(rot)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    if(1):
      marker_array = MarkerArray()

      for i,box in enumerate(bboxes):
        abs_box_x = math.cos(euler[2])*box.center.x-math.sin(euler[2])*box.center.y+trans[0]
        abs_box_y = math.sin(euler[2])*box.center.x+math.cos(euler[2])*box.center.y+trans[1]

        for j in range(len(parking_spot)):
          marker = Marker()
          marker.id = j
          marker.header.frame_id = "/map"
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

          r2 = RotatedRect(parking_spot[j][0]+trans[0], parking_spot[j][1]+trans[1], 2, 2,+0.338)
          point = shapely.geometry.Point(abs_box_x,abs_box_y)

          if(r2.get_contour().contains(point)):
            empty_spot_array.data[j] = 1
            rospy.logwarn("%d not empty",j)
          else:
            rospy.logwarn("%d is empty",j)

          for k in range(len(r2.get_contour().exterior.coords)):
            p = Point32()
            p.x = r2.get_contour().exterior.coords[k][0]
            p.y = r2.get_contour().exterior.coords[k][1]
            marker.points.append(p)
          
          marker_array.markers.append(marker) 
        
        marker_pub.publish(marker_array)
      empty_pub.publish(empty_spot_array)
    # print("ok")
    r.sleep()



if __name__ == '__main__': 
  try:
    publish_obstacle_msg()
  except rospy.ROSInterruptException:
    pass


