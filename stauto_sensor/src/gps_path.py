#! /usr/bin/env python

import rospy
import tf
import numpy as np
import rospkg
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from gps_common import *
import copy

from math import *

def convert_degree_to_meter(lat,lon):

    alpha = lat*pi/180
    beta = lon*pi/180

    a = 6377397.155
    f = 1/299.1528128
    b = a*(1-f)
    k = 1
    x_plus = 500000
    y_plus = 200000

    e1 = (a**2-b**2)/a**2
    e2 = (a**2-b**2)/b**2
    alpha0 = 38 *pi/180
    beta0 = (125+0.002890277)*pi/180

    T = pow(tan(alpha),2)
    C = e1/(1-e1)*pow(cos(alpha),2)
    AA = (beta-beta0)*cos(alpha)  #both radian

    N = a/sqrt( 1-e1*sin(alpha)**2 )
    M = a*(alpha*(1-e1/4-3*e1**2/64-5*e1**3/256)-sin(2*alpha)*(3*e1/8+3*e1**2/32+45*e1**3/1024)
        +sin(4*alpha)*(15*e1**2/256+45*e1**3/1024)-35*e1**3*sin(6*alpha)/3072)

    M0 = a*(alpha0*(1-e1/4-3*e1**2/64-5*e1**3/256)-sin(2*alpha0)*(3*e1/8+3*e1**2/32+45*e1**3/1024)
         +sin(4*alpha0)*(15*e1**2/256+45*e1**3/1024)-35*e1**3*sin(6*alpha0)/3072)

    Y = y_plus + k*N*(AA+AA**3*(1-T+C)/6+pow(AA,5)*(5-18*T+T*T+72*C-58*e2)/120)
    X = x_plus + k*(M-M0+N*tan(alpha)*(AA*AA/2 + pow(AA,4)*(5-T+9*C+4*C*C)/24 +
        pow(AA,6)*(61-58*T+T*T+600*C-330*e2)/720))

    return [X,Y]

def odom_callback(data):
    global odom_x, odom_y

    #print(odom_x)
    odom_x = data.pose.pose.position.x
    odom_y = data.pose.pose.position.y


def euler_to_quaternion(yaw, pitch, roll):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

def pub_nav(lat,lon):

    gpsmsg.header.stamp = rospy.Time.now()
    gpsmsg.header.frame_id = "world"
    gpsmsg.latitude=lat
    gpsmsg.longitude=lon
    gpsmsg. position_covariance_type=0
    navs_pub.publish(gpsmsg)
    #print(gpsmsg)

def pub_path(theta):
    global step_gps, odom_x, odom_y, init_time
    qx, qy, qz, qw = euler_to_quaternion(theta, 0, 0)

    pose = PoseStamped()
    goal = PoseStamped()

    pathmsg.header.seq = step_gps
    pathmsg.header.stamp = rospy.Time.now()
    pathmsg.header.frame_id = "world"

    pose.header.seq = pathmsg.header.seq
    pose.header.stamp = pathmsg.header.stamp
    pose.header.frame_id = pathmsg.header.frame_id

    pose.pose.position.x = odom_x
    pose.pose.position.y = odom_y
    pose.pose.position.z = 0

    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    pathmsg.poses.append(pose)

    path_pub.publish(pathmsg)


    if (step_gps == len(gps_data)-2):
        goal.header.seq = 0
        goal.header.stamp = init_time
        goal.header.frame_id = "world"

        goal.pose.position.x = odom_x
        goal.pose.position.y = odom_y
        goal.pose.position.z = 0

        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw

        goal_pub.publish(goal)


    #print(pathmsg)

# Lat Long - UTM, UTM - Lat Long conversions

from math import pi, sin, cos, tan, sqrt

#LatLong- UTM conversion..h
#definitions for lat/long to UTM and UTM to lat/lng conversions
#include <string.h>

_deg2rad = pi / 180.0
_rad2deg = 180.0 / pi

_EquatorialRadius = 2
_eccentricitySquared = 3

_ellipsoid = [
#  id, Ellipsoid name, Equatorial Radius, square of eccentricity	
# first once is a placeholder only, To allow array indices to match id numbers
	[ -1, "Placeholder", 0, 0],
	[ 1, "Airy", 6377563, 0.00667054],
	[ 2, "Australian National", 6378160, 0.006694542],
	[ 3, "Bessel 1841", 6377397, 0.006674372],
	[ 4, "Bessel 1841 (Nambia] ", 6377484, 0.006674372],
	[ 5, "Clarke 1866", 6378206, 0.006768658],
	[ 6, "Clarke 1880", 6378249, 0.006803511],
	[ 7, "Everest", 6377276, 0.006637847],
	[ 8, "Fischer 1960 (Mercury] ", 6378166, 0.006693422],
	[ 9, "Fischer 1968", 6378150, 0.006693422],
	[ 10, "GRS 1967", 6378160, 0.006694605],
	[ 11, "GRS 1980", 6378137, 0.00669438],
	[ 12, "Helmert 1906", 6378200, 0.006693422],
	[ 13, "Hough", 6378270, 0.00672267],
	[ 14, "International", 6378388, 0.00672267],
	[ 15, "Krassovsky", 6378245, 0.006693422],
	[ 16, "Modified Airy", 6377340, 0.00667054],
	[ 17, "Modified Everest", 6377304, 0.006637847],
	[ 18, "Modified Fischer 1960", 6378155, 0.006693422],
	[ 19, "South American 1969", 6378160, 0.006694542],
	[ 20, "WGS 60", 6378165, 0.006693422],
	[ 21, "WGS 66", 6378145, 0.006694542],
	[ 22, "WGS-72", 6378135, 0.006694318],
	[ 23, "WGS-84", 6378137, 0.00669438]
]

#Reference ellipsoids derived from Peter H. Dana's website- 
#http://www.utexas.edu/depts/grg/gcraft/notes/datum/elist.html
#Department of Geography, University of Texas at Austin
#Internet: pdana@mail.utexas.edu
#3/22/95

#Source
#Defense Mapping Agency. 1987b. DMA Technical Report: Supplement to Department of Defense World Geodetic System
#1984 Technical Report. Part I and II. Washington, DC: Defense Mapping Agency

#def LLtoUTM(int ReferenceEllipsoid, const double Lat, const double Long, 
#			 double &UTMNorthing, double &UTMEasting, char* UTMZone)

def LLtoUTM(ReferenceEllipsoid, Lat, Long):
#converts lat/long to UTM coords.  Equations from USGS Bulletin 1532 
#East Longitudes are positive, West longitudes are negative. 
#North latitudes are positive, South latitudes are negative
#Lat and Long are in decimal degrees
#Written by Chuck Gantz- chuck.gantz@globalstar.com

    a = _ellipsoid[ReferenceEllipsoid][_EquatorialRadius]
    eccSquared = _ellipsoid[ReferenceEllipsoid][_eccentricitySquared]
    k0 = 0.9996

#Make sure the longitude is between -180.00 .. 179.9
    LongTemp = (Long+180)-int((Long+180)/360)*360-180 # -180.00 .. 179.9

    LatRad = Lat*_deg2rad
    LongRad = LongTemp*_deg2rad

    ZoneNumber = int((LongTemp + 180)/6) + 1
  
    if Lat >= 56.0 and Lat < 64.0 and LongTemp >= 3.0 and LongTemp < 12.0:
        ZoneNumber = 32

    # Special zones for Svalbard
    if Lat >= 72.0 and Lat < 84.0:
        if  LongTemp >= 0.0  and LongTemp <  9.0:ZoneNumber = 31
        elif LongTemp >= 9.0  and LongTemp < 21.0: ZoneNumber = 33
        elif LongTemp >= 21.0 and LongTemp < 33.0: ZoneNumber = 35
        elif LongTemp >= 33.0 and LongTemp < 42.0: ZoneNumber = 37

    LongOrigin = (ZoneNumber - 1)*6 - 180 + 3 #+3 puts origin in middle of zone
    LongOriginRad = LongOrigin * _deg2rad

    #compute the UTM Zone from the latitude and longitude
    UTMZone = "%d%c" % (ZoneNumber, _UTMLetterDesignator(Lat))

    eccPrimeSquared = (eccSquared)/(1-eccSquared)
    N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad))
    T = tan(LatRad)*tan(LatRad)
    C = eccPrimeSquared*cos(LatRad)*cos(LatRad)
    A = cos(LatRad)*(LongRad-LongOriginRad)

    M = a*((1
            - eccSquared/4
            - 3*eccSquared*eccSquared/64
            - 5*eccSquared*eccSquared*eccSquared/256)*LatRad 
           - (3*eccSquared/8
              + 3*eccSquared*eccSquared/32
              + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
           + (15*eccSquared*eccSquared/256 + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad) 
           - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad))
    
    UTMEasting = (k0*N*(A+(1-T+C)*A*A*A/6
                        + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
                  + 500000.0)

    UTMNorthing = (k0*(M+N*tan(LatRad)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
                                        + (61
                                           -58*T
                                           +T*T
                                           +600*C
                                           -330*eccPrimeSquared)*A*A*A*A*A*A/720)))

    if Lat < 0:
        UTMNorthing = UTMNorthing + 10000000.0; #10000000 meter offset for southern hemisphere
    return (UTMEasting, UTMNorthing)


def _UTMLetterDesignator(Lat):
#This routine determines the correct UTM letter designator for the given latitude
#returns 'Z' if latitude is outside the UTM limits of 84N to 80S
#Written by Chuck Gantz- chuck.gantz@globalstar.com

    if 84 >= Lat >= 72: return 'X'
    elif 72 > Lat >= 64: return 'W'
    elif 64 > Lat >= 56: return 'V'
    elif 56 > Lat >= 48: return 'U'
    elif 48 > Lat >= 40: return 'T'
    elif 40 > Lat >= 32: return 'S'
    elif 32 > Lat >= 24: return 'R'
    elif 24 > Lat >= 16: return 'Q'
    elif 16 > Lat >= 8: return 'P'
    elif  8 > Lat >= 0: return 'N'
    elif  0 > Lat >= -8: return 'M'
    elif -8> Lat >= -16: return 'L'
    elif -16 > Lat >= -24: return 'K'
    elif -24 > Lat >= -32: return 'J'
    elif -32 > Lat >= -40: return 'H'
    elif -40 > Lat >= -48: return 'G'
    elif -48 > Lat >= -56: return 'F'
    elif -56 > Lat >= -64: return 'E'
    elif -64 > Lat >= -72: return 'D'
    elif -72 > Lat >= -80: return 'C'
    else: return 'Z'	# if the Latitude is outside the UTM limits

if __name__ == '__main__':

    rospy.init_node('gps_path')
    listener = tf.TransformListener()

    rospy.Subscriber("odom",Odometry,odom_callback)

    navs_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=1)
    path_pub = rospy.Publisher('/path', Path, queue_size=1)
    goal_pub = rospy.Publisher('/pure_pursuit/goal', PoseStamped, queue_size=1)

    rospack = rospkg.RosPack()
    rospack.list()
    arg_name = rospack.get_path('stauto_sensor') + "/src/gps_data/"

    f = open(arg_name + "gps_data_seoultech.txt","r")

    gps_data = f.readlines()
    step_gps=0
    odom_x=0
    odom_y=0
    theta=[]

    gpsmsg=NavSatFix()
    pathmsg=Path()
    rospy.sleep(0.5)

    init_time=rospy.Time.now()

    while not rospy.is_shutdown():
        #s(_,rot) = listener.lookupTransform('pose', 'base_link', rospy.Time(0))
        #yaw = -tf.transformations.euler_from_quaternion(rot)[2]

        if (step_gps<len(gps_data)-1):


            gps_origin = LLtoUTM(-1, 37.6312721, 127.0766235)
            gps_n = gps_data[step_gps].split(',')
            gps_n_1 = gps_data[step_gps+1].split(',')
            # gps_n = [float(gps_n[0]) - float(gps_origin[0]), float(gps_n[1]) - float(gps_origin[1])]
            # gps_n_1 = [float(gps_n_1[0]) - float(gps_origin[0]), float(gps_n_1[1])- float(gps_origin[1])]
            gps_n = [float(gps_n[0]), float(gps_n[1])]
            gps_n_1 = [float(gps_n_1[0]), float(gps_n_1[1])]


            pub_nav(gps_n[0], gps_n[1]) #from LL to UTM
            gps_n = LLtoUTM(-1,gps_n[0], gps_n[1])
            gps_n_1 = LLtoUTM(-1,gps_n_1[0], gps_n_1[1])
            

            # gps_n = convert_degree_to_meter(gps_n[0],gps_n[1])
            # gps_n_1 = convert_degree_to_meter(gps_n_1[0],gps_n_1[1])
            # gps_n[0] = (gps_n[0] - 460000)/100
            # gps_n[1] = (gps_n[1] - 383000)/100
            # gps_n_1[0] = (gps_n_1[0] - 460000)/100
            # gps_n_1[1] = (gps_n_1[1] - 383000)/100

            theta.append(atan2(gps_n_1[1]-gps_n[1], gps_n_1[0]-gps_n[0]))

            pub_path(theta[step_gps])
            rospy.sleep(0.1)
            step_gps=step_gps+1




            #Line = sqrt((gps_n[0]-trans[0])**2 + (gps_n[1]-trans[1])**2)
        else:
            pass


#!/usr/bin/env python

