#!/usr/bin/env python
import rospy
import roslaunch
import subprocess
import signal
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
import tf2_ros

last_id = 1
flag = False
x, y, yaw = 0, 0, 0
homogeneous = [0, 0, 0]
package_name = "car_connect"


def init_callback(data):
    global last_id, flag, x, y, yaw
    flag = True 
    rospy.logwarn("homogeneous : {}".format(homogeneous))

    x = data.pose.pose.position.x - homogeneous[0]
    y = data.pose.pose.position.y - homogeneous[1]
    q = data.pose.pose.orientation
    yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2] - homogeneous[2]
    rospy.logwarn("setting pose {:>2f} {:>2f} {:>2f}".format(x,y,yaw))

    # x = round(x, 3)
    # y = round(y, 3)
    # yaw = round(yaw, 3)

    # finish_traj = subprocess.Popen(["rosservice", "call", "/finish_trajectory", str(last_id)])
    # rospy.sleep(0.5)
    start_new_traj = subprocess.Popen(["roslaunch", package_name, "start_trajectory.launch",
                                       "x:={}".format(x),
                                       "y:={}".format(y),
                                       "yaw:={}".format(yaw)])

    # last_id += 1
    # last_id = 0


rospy.init_node("cartographer_initializer")


rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, init_callback)
rate = rospy.Rate(10)
while not rospy.is_shutdown():


    rate.sleep()