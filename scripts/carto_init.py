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
    # x = data.pose.pose.position.x
    # y = data.pose.pose.position.y
    # q = data.pose.pose.orientation
    # yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
    rospy.logwarn("homogeneous : {}".format(homogeneous))

    x = data.pose.pose.position.x - homogeneous[0]
    y = data.pose.pose.position.y - homogeneous[1]
    q = data.pose.pose.orientation
    yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2] - homogeneous[2]
    rospy.logwarn("setting pose {:>2f} {:>2f} {:>2f}".format(x,y,yaw))

    # x = round(x, 3)
    # y = round(y, 3)
    # yaw = round(yaw, 3)

    finish_traj = subprocess.Popen(["rosservice", "call", "/finish_trajectory", str(last_id)])
    rospy.sleep(0.5)
    start_new_traj = subprocess.Popen(["roslaunch", package_name, "start_trajectory.launch",
                                       "x:={}".format(x),
                                       "y:={}".format(y),
                                       "yaw:={}".format(yaw)])
    start_new_traj = subprocess.Popen(["rosrun", "cartographer_ros", "cartographer_start_trajectory", "-configuration_directory ~/catkin_ws/src/cartographer_ros/cartographer_ros/configuration_files -configuration_basename backpack_2d.lua -initial_pose '{to_trajectory_id = 0, relative_pose = { translation = {2.95815134048, 0.165419101715, 0. }, rotation = { 0.0, 0.0, 0.0 } } ,timestamp = 0}' 
"])

    # last_id += 1
    last_id = 0


rospy.init_node("cartographer_initializer")
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
published_frame = rospy.get_param("~published_frame", default="base_footprint")
# tracking_frame = rospy.get_param("~tracking_frame", default="base_footprint")
footprint_frame = rospy.get_param("~footprint_frame", default="base_footprint")
if package_name == "":
    print("Required package_name!")
    exit()

rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, init_callback)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    if published_frame != footprint_frame:
        try:
            trans = tfBuffer.lookup_transform(published_frame, footprint_frame, rospy.Time())
            homogeneous[0] = trans.transform.translation.x
            homogeneous[1] = trans.transform.translation.y
            euler = euler_from_quaternion([trans.transform.rotation.x, trans.transform.rotation.y,
                                           trans.transform.rotation.z, trans.transform.rotation.w])

            homogeneous[2] = euler[2]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass
    else:
        homogeneous = [0, 0, 0]

    rate.sleep()