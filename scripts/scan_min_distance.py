#!/usr/bin/env python
import rospy
import numpy

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from std_msgs.msg import Float32

pub_status = rospy.Publisher('status', Int32, queue_size=10)

pub_min = rospy.Publisher('min', Float32, queue_size=10)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('tower_scan_min_distance', anonymous=True)

    rospy.Subscriber("camera_fwd/depth/color/scan", LaserScan, callback)

    #pub_status = rospy.Publisher('status', Int32, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def callback(msg):

    data = numpy.array(msg.ranges)

    # this part is 20x faster than using the built-in python functions
    index = numpy.argmin(data)

    minval = data[index]
    numpy.isinf

    status = 2

    if(minval < 0.4) or numpy.isinf(minval):
        status = 2
    elif( (minval >= 0.4) and (minval < 0.75) ):
        status = 1
    elif( (minval >= 0.75) and (minval < 100) ):
        status = 0

    pub_status.publish(Int32(status))

    min_msg = 0.5
    pub_min.publish(Float32(minval))

if __name__ == '__main__':
    listener()
