#! /usr/bin/env python

import serial
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped

port = "/dev/ttyUSB0"
rpy=[0,0,0]
w_speed=[0,0,0]
accel=[0,0,0]

P = [[0]*2 for i in range(2)]
Q_angle = 0.05
Q_bias = 0.001
R_measure = 0.01
angle_kalman = 0
bias = 0
rate = 0

K=[0,0]
y=0
S=0


def kalman(newAngle, prevAngle, dt):
    global bias, Q_angle, Q_bias, R_measure, angle_kalman
    rate = prevAngle - bias
    angle_kalman += dt * rate

    #Update estimation error covariance - Project the error covariance ahead

    # Step 2
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle)
    P[0][1] -= dt * P[1][1]
    P[1][0] -= dt * P[1][1]
    P[1][1] += Q_bias * dt

    #Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    #Calculate Kalman gain - Compute the Kalman gain

    #Step 4
    S = P[0][0] + R_measure

    #Step 5
    K[0] = P[0][0] / S
    K[1] = P[1][0] / S

    #Calculate angle_kalman and bias - Update estimate with measurement zk (newAngle)
    #Step 3
    y = newAngle - angle_kalman;

    #Step 6
    angle_kalman += K[0] * y
    bias += K[1] * y

    #Calculate estimation error covariance - Update the error covariance

    #Step 7
    P[0][0] -= K[0] * P[0][0]
    P[0][1] -= K[0] * P[0][1]
    P[1][0] -= K[1] * P[0][0]
    P[1][1] -= K[1] * P[0][1]

    return angle_kalman
def euler_to_quaternion(roll,pitch,yaw):

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return [qx, qy, qz, qw]

def pub_tf_transform(roll,pitch,yaw,w_speed,accel):


    t = TransformStamped()  # pose of turntable_frame w.r.t. turntable_base
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'base_link'
    t.child_frame_id  = 'imu_link'
    t.transform.translation.x = roll
    t.transform.translation.y = pitch
    t.transform.translation.z = yaw
    q = euler_to_quaternion(roll, pitch, yaw)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node("get_imu")

    port = rospy.get_param("~GPS_PORT",port)
    ser = serial.serial_for_url(port,115200, timeout=0)

    imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=1)

    br = tf2_ros.TransformBroadcaster()

    r=rospy.Rate(1)

    rospy.sleep(0.5)

    imu=Imu()
    # get prevAngle
    IMU_message=ser.readline()
    data=IMU_message.split(",")
    prevAngle = -round(float(data[3]),3)*np.pi/180
    # calculation dt
    prevTime = rospy.Time.now()
    curlTime = rospy.Time.now()
    print(prevAngle)

    while not rospy.is_shutdown():
        IMU_message=ser.readline()

        if (len(IMU_message)>55):
            imu.header.stamp = rospy.Time.now()
            imu.header.frame_id = "imu_link"
            data=IMU_message.split(",")

            rpy[0]=round(float(data[1]),3)
            rpy[1]=round(float(data[2]),3)
            rpy[2]=round(float(data[3]),3)

            roll=-rpy[0]*np.pi/180  #edit
            pitch=rpy[1]*np.pi/180  #edit
            yaw=-rpy[2]*np.pi/180

            curlTime=rospy.Time.now()
            dt=(curlTime-prevTime).to_sec()
            #print(-rpy[2])
            '''
            yaw=kalman(yaw,prevAngle,dt)
            prevTime=curlTime
            #print(yaw*180/np.pi)
            prevAngle=yaw
            '''
            qx,qy,qz,qw = euler_to_quaternion(roll, pitch, yaw)

            imu.orientation.x = qx
            imu.orientation.y = qy
            imu.orientation.z = qz
            imu.orientation.w = qw

            w_speed[0]=round(float(data[4]),3)
            w_speed[1]=round(float(data[5]),3)
            w_speed[2]=round(float(data[6]),3)

            imu.angular_velocity.x = w_speed[0]
            imu.angular_velocity.y = w_speed[1]
            imu.angular_velocity.z = -w_speed[2]  #edit

            accel[0]=round(float(data[7]),3)
            accel[1]=round(float(data[8]),3)
            accel[2]=round(float(data[9]),3)

            imu.linear_acceleration.x = accel[0]
            imu.linear_acceleration.y = accel[1]
            imu.linear_acceleration.z = accel[2]

            battery=round(float(data[10]),3)
            imu_pub.publish(imu)
            pub_tf_transform(roll,pitch,yaw,w_speed,accel)
            print("yaw:",rpy[2], "battery:",battery )

        else:
            pass
    else:
        pass
        #mainloop()
