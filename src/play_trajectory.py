#! /usr/bin/env python3

from Stampede.Utils.ros_utils import *
import rospy
import os
import numpy as np
import sys
import tf
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

rospy.init_node('play_trajectory')
np.set_printoptions(suppress=True)

js_pub = rospy.Publisher('joint_states', JointState, queue_size=5)
marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 3)
tf_pub = tf.TransformBroadcaster()

path_to_src = os.path.dirname(__file__)
print(path_to_src)
f = open(path_to_src + '/Stampede/Config/relaxedik_path')
path_to_relaxedik_src =  f.readline()
path_to_relaxedik_src = path_to_relaxedik_src + '/src'
f.close()

sys.path.append(path_to_relaxedik_src )
from RelaxedIK.Utils.yaml_utils import get_relaxedIK_yaml_obj


y = get_relaxedIK_yaml_obj(path_to_relaxedik_src)
if not y == None:
    urdf_file_name = y['urdf_file_name']
    fixed_frame = y['fixed_frame']
    joint_ordering = y['joint_ordering']
    starting_config = y['starting_config']
    joint_state_define_file_name = y['joint_state_define_func_file']
    joint_state_define_file = open(
        path_to_relaxedik_src + '/RelaxedIK/Config/joint_state_define_functions/' + joint_state_define_file_name, 'r')
    func = joint_state_define_file.read()
    exec (func)

rospy.sleep(0.1)

trajectory_name = rospy.get_param('trajectory_name', default='')
if trajectory_name == '':
    fp = path_to_src + '/Stampede/OutputMotions/last_trajectory.stampede'
else:
    fp = path_to_src + '/Stampede/OutputMotions/{}.stampede'.format(trajectory_name)

with open(fp, 'r') as f:
    first_line = f.readline()
    first_line_arr = first_line.split(',')
    robot_info = first_line_arr[0]
    target_path_name = first_line_arr[1].replace(" ", "")
    target_path_filepath = os.path.join(path_to_src, "Stampede/InputMotions/", target_path_name)
    init_pos = np.array([ float(first_line_arr[3]), float(first_line_arr[4]),  float(first_line_arr[5])])

    lines = []
    line = f.readline()
    while not line == '':
        lines.append(line)
        line = f.readline()



times = []
states = []
goal_positions = []
goal_orientations = []


for i in range(len(lines)):
    line_arr = lines[i].split(';')
    times.append(float(line_arr[0]))

    state_arr = line_arr[1].split(',')
    state = []
    for j in range(len(state_arr)):
        state.append(float(state_arr[j]))
    states.append(state)

    pos_arr = line_arr[2].split(',')
    goal_pos = [ float(pos_arr[0]),float(pos_arr[1]), float(pos_arr[2])   ]
    goal_positions.append(np.array(goal_pos) + init_pos)

    quat_arr = line_arr[3].split(',')
    quat = [ float(quat_arr[0]),float(quat_arr[1]), float(quat_arr[2]), float(quat_arr[3])   ]
    goal_orientations.append(quat)


with open(target_path_filepath, "r") as tpf:

    path_offset__T__path0_dict = {
        ("fetch_info.yaml", "cppflow__circle") : np.array([0.9, 0.25, 0.46]),
        ("fetch_info.yaml", "cppflow__hello") : np.array([0.8, 0.45, 0.25]),
        ("fetch_info.yaml", "cppflow__rotation") : np.array([0.8, 0.0, 0.35]),
        ("fetch_info.yaml", "cppflow__s") : np.array([1.0, 0.3, 0.55]),
        ("fetch_info.yaml", "cppflow__square") : np.array([1.1, 0.0, 0.66]),
        ("fetch_arm_info.yaml", "cppflow__circle") : np.array([0.9, 0.25, 0.46]),
        ("fetch_arm_info.yaml", "cppflow__hello") : np.array([0.8, 0.45, 0.25]),
        ("fetch_arm_info.yaml", "cppflow__rotation") : np.array([0.8, 0.0, 0.35]),
        ("fetch_arm_info.yaml", "cppflow__s") : np.array([1.0, 0.3, 0.55]),
        ("fetch_arm_info.yaml", "cppflow__square") : np.array([1.1, 0.0, 0.66])
        }
    world__T__offset_frame_dict = {
        "fetch_arm_info.yaml" : np.array([-0.086875, 0., 0.37743]),
        "fetch_info.yaml" : np.array([-0.086875, 0., 0.37743]),
    }

    lines = tpf.readlines()
    target_path_positions_full = 2.5 * np.ones((len(lines), 3))
    for idx, line in enumerate(lines):
        time, pos, rot = line.split(";")
        x, y, z = pos.split(",")
        target_path_positions_full[idx, 0] = float(x)
        target_path_positions_full[idx, 1] = float(y)
        target_path_positions_full[idx, 2] = float(z)
        # print(idx, target_path_positions_full[idx])

    target_path_positions_full += (world__T__offset_frame_dict[robot_info] + path_offset__T__path0_dict[(robot_info, target_path_name)])


TARGET_PATH_COLOR = (0.0, 1.0, 0.0, 0.5)
rate = rospy.Rate(40)
idx = 0
while not rospy.is_shutdown():
    tf_pub.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         'common_world',
                         fixed_frame)

    if idx >= len(states):
        rospy.sleep(2.0)
        idx = 0
    xopt = states[idx]
    js = joint_state_define(xopt)

    if js == None:
        js = JointState()
        js.name = joint_ordering
        for x in xopt:
            js.position.append(x)
    now = rospy.Time.now()
    js.header.stamp.secs = now.secs
    js.header.stamp.nsecs = now.nsecs
    js_pub.publish(js)

    # draw_linestrip_in_rviz(marker_pub, 'common_world', goal_positions, color=[0.,0.2,1.0,0.6], width=0.01)
    # draw_linestrip_in_rviz(marker_pub, 'common_world', goal_positions[:idx], color=[0., 0.8, .2, 0.8], width=0.02, id=1001)

    draw_linestrip_in_rviz(marker_pub, 'common_world', target_path_positions_full, color=TARGET_PATH_COLOR, width=0.01, id=1001)


    idx += 1

    rate.sleep()