#! /usr/bin/env python

import rospy
from cv_bridge import CvBridge
import sys, select, termios, tty, math
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2, time
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
from sensor_msgs.msg import Image

def GetKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == '__main__':
    rospy.init_node('camera_node')

    capture1 = cv2.VideoCapture(0)
    capture2 = cv2.VideoCapture(1)
    #capture3 = cv2.VideoCapture(2)
    capture1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    capture2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    capture2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    #capture3.set(cv2.CAP_PROP_FRAME_WIDTH, 400)
    #capture3.set(cv2.CAP_PROP_FRAME_HEIGHT, 300)
    image_b='/video/lane'
    image_a='/video/sign'
    video_pub1 = rospy.Publisher(image_a, Image, queue_size=5)
    video_pub2 = rospy.Publisher(image_b, Image, queue_size=5)
    #video_pub3 = rospy.Publisher('/video/parking', Image, queue_size=5)

    bridge = CvBridge()
    settings = termios.tcgetattr(sys.stdin)
    time.sleep(1)
    prev_time=0
    
    r = rospy.Rate(30)

    while not rospy.is_shutdown():
    #while (capture1.isOpened() and capture2.isOpened() and capture3.isOpened()):
    #compress
        try:
            if(capture1.isOpened()):
                ret, frame1 = capture1.read()
                #cv2.imshow(image_a, frame1)

                a=bridge.cv2_to_imgmsg(frame1, "bgr8")

                video_pub1.publish(a)
            
            if(capture2.isOpened()):
                ret2, frame2 = capture2.read()
                #cv2.imshow(image_b, frame2)

                b=bridge.cv2_to_imgmsg(frame2, "bgr8")

                video_pub2.publish(b)
            '''
            if(capture3.isOpened()):
                ret3, frame3 = capture3.read()
                cv2.imshow("VideoFrame3", frame3)
            
                c=bridge.cv2_to_imgmsg(frame3, "bgr8")

                video_pub3.publish(c)
            '''

            #aa=bridge.imgmsg_to_cv2(a)
            #cv2.imshow("aa",aa)
            #print("FPS: ",1/(time.time()-prev_time))
            prev_time=time.time()

            #key=GetKey()
            #if key == '\x03': #CTRL + C
            #    break

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            r.sleep()

        except rospy.ROSInterruptException:
            pass
        
    capture1.release()
    capture2.release()
    #capture3.release()
    cv2.destroyAllWindows() 
