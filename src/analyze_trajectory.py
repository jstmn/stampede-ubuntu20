#! /usr/bin/env python3

from Stampede.Utils.ros_utils import *
import rospy
import os
import numpy as np
import sys
import tf
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

rospy.init_node('analyze_trajectory')
rospy.sleep(0.1)
np.set_printoptions(suppress=True)

# js_pub = rospy.Publisher('joint_states', JointState, queue_size=5)
# marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 3)
# tf_pub = tf.TransformBroadcaster()

path_to_src = os.path.dirname(__file__)
with open(path_to_src + '/Stampede/Config/relaxedik_path') as f:
    path_to_relaxedik_src =  f.readline()
    path_to_relaxedik_src = path_to_relaxedik_src + '/src'

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


trajectory_name = rospy.get_param('trajectory_name', default='')
if trajectory_name == '':
    fp = path_to_src + '/Stampede/OutputMotions/last_trajectory.stampede'
else:
    fp = path_to_src + '/Stampede/OutputMotions/{}.stampede'.format(trajectory_name)


with open(fp, 'r') as f:
    first_line = f.readline()
    first_line_arr = first_line.split(',')
    planning_time_sec = first_line_arr[6]
    was_successful_str = first_line_arr[7].replace("\n", "").replace(" ", "").upper()
    assert was_successful_str in {"TRUE", "FALSE"}, f"unrecognized was_successful string: '{was_successful_str}'"
    traj_is_correct_length = True if was_successful_str == "TRUE" else False
    robot_info = first_line_arr[0]
    target_path_name = first_line_arr[1].replace(" ", "")
    target_path_filepath = os.path.join(path_to_src, "Stampede/InputMotions/", target_path_name)
    target_path_name = target_path_name.replace("cppflow__", "")
    init_pos = np.array([ float(first_line_arr[3]), float(first_line_arr[4]),  float(first_line_arr[5])])

    lines = []
    line = f.readline()
    while not line == '':
        lines.append(line)
        line = f.readline()


# Get solution joint angles
qpath = []
for i in range(len(lines)):
    line_arr = lines[i].split(';')
    state_arr = line_arr[1].split(',')
    state = []
    for j in range(len(state_arr)):
        state.append(float(state_arr[j]))
    qpath.append(state)
qpath = np.array(qpath)


# Get target path
with open(target_path_filepath, "r") as tpf:

    path_offset__T__path0_dict = {
        ("fetch_info.yaml", "circle") : np.array([0.9, 0.25, 0.46]),
        ("fetch_info.yaml", "hello") : np.array([0.8, 0.45, 0.25]),
        ("fetch_info.yaml", "rotation") : np.array([0.8, 0.0, 0.35]),
        ("fetch_info.yaml", "s") : np.array([1.0, 0.3, 0.55]),
        ("fetch_info.yaml", "square") : np.array([1.1, 0.0, 0.66]),
        ("fetch_arm_info.yaml", "circle") : np.array([0.9, 0.25, 0.46]),
        ("fetch_arm_info.yaml", "hello") : np.array([0.8, 0.45, 0.25]),
        ("fetch_arm_info.yaml", "rotation") : np.array([0.8, 0.0, 0.35]),
        ("fetch_arm_info.yaml", "s") : np.array([1.0, 0.3, 0.55]),
        ("fetch_arm_info.yaml", "square") : np.array([1.1, 0.0, 0.66])
        }
    world__T__offset_frame_dict = {
        "fetch_arm_info.yaml" : np.array([-0.086875, 0., 0.37743]),
        "fetch_info.yaml" : np.array([-0.086875, 0., 0.37743]),
    }

    lines = tpf.readlines()
    target_path = 1000 * np.ones((len(lines), 7))
    for idx, line in enumerate(lines):
        time, pos, rot = line.split(";")
        x, y, z = pos.split(",")
        qw, qx, qy, qz = rot.split(",")
        target_path[idx, 0] = float(x)
        target_path[idx, 1] = float(y)
        target_path[idx, 2] = float(z)
        target_path[idx, 3] = float(qw)
        target_path[idx, 4] = float(qx)
        target_path[idx, 5] = float(qy)
        target_path[idx, 6] = float(qz)
        # print(idx, target_path[idx])

    target_path[:, 0:3] += (world__T__offset_frame_dict[robot_info] + path_offset__T__path0_dict[(robot_info, target_path_name)])

# Analyze trajectory and write results to a csv
from cppflow.planners import Plan, PlanNp, positional_errors, rotational_errors
from jkinpylib.robots import get_robot
import pandas as pd
import shutil
import datetime
import torch

robot_name = robot_info.replace("_info.yaml", "")  # fetch_arm_info.yaml
robot = get_robot(robot_name)

generated_trajectory_poses = torch.tensor(robot.forward_kinematics(qpath), dtype=torch.float32, device="cpu")
qpath = torch.tensor(qpath, dtype=torch.float32, device="cpu")
qpath = torch.remainder(qpath + torch.pi, 2 * torch.pi) - torch.pi

assert robot_name == "fetch_arm", "figure out good way to remap joint angles to [-pi, pi] for fetch_full"

target_path = torch.tensor(target_path[0:generated_trajectory_poses.shape[0], :], dtype=torch.float32, device="cpu")
qpath_revolute, qpath_prismatic = robot.split_configs_to_revolute_and_prismatic(qpath)

print("robot:", robot)
print("target_path:               ", target_path.shape)
print("generated_trajectory_poses:", generated_trajectory_poses.shape)

if robot.name == "fetch" and qpath.shape[0] > 1:
    assert qpath_prismatic.numel() > 0

plan = Plan(
    q_path=qpath,
    q_path_revolute=qpath_revolute,
    q_path_prismatic=qpath_prismatic,
    pose_path=generated_trajectory_poses,
    target_path=target_path,
    robot_joint_limits=robot.actuated_joints_limits,
    n_self_colliding=0,
    n_env_colliding=0,
    positional_errors=positional_errors(generated_trajectory_poses, target_path),
    rotational_errors=rotational_errors(generated_trajectory_poses, target_path),
)
print(plan)
plan = PlanNp(plan)

pos_errors_mm = plan.positional_errors_mm
rot_errors_deg = plan.rotational_errors_deg

min_pos_error_mm = float(np.min(pos_errors_mm))
max_pos_error_mm = float(np.max(pos_errors_mm))
mean_pos_error_mm = float(np.mean(pos_errors_mm))
std_pos_error_mm = float(np.std(pos_errors_mm))
min_rot_error_deg = float(np.min(rot_errors_deg))
max_rot_error_deg = float(np.max(rot_errors_deg))
mean_rot_error_deg = float(np.mean(rot_errors_deg))
std_rot_error_deg = float(np.std(rot_errors_deg))

per_timestep_mjac_deg = plan.mjac_per_timestep_deg
mean_per_timestep_mjac_deg = np.mean(per_timestep_mjac_deg)
std_per_timestep_mjac_deg = np.std(per_timestep_mjac_deg)
max_per_timestep_mjac_deg = np.max(per_timestep_mjac_deg)

per_timestep_mjac_cm = plan.mjac_per_timestep_cm
mean_per_timestep_mjac_cm = np.mean(per_timestep_mjac_cm)
std_per_timestep_mjac_cm = np.std(per_timestep_mjac_cm)
max_per_timestep_mjac_cm = np.max(per_timestep_mjac_cm)

if qpath_prismatic.numel() > 0:
    print("qpath_prismatic:", qpath_prismatic[:, 0])
    print("per_timestep_mjac_cm:", per_timestep_mjac_cm)

if robot.name == "fetch" and qpath.shape[0] > 1:
    # print("per_timestep_mjac_cm")
    print(f"Warning: mean_per_timestep_mjac_cm is {mean_per_timestep_mjac_cm}, should be greater than 0. This means the prismatic joint isn't moving")

filename = f"results__{robot_name}__{target_path_name}"
results_filepath = f"/home/jstm/Projects/cppflowpaper2/benchmarking/stampede/raw/{filename}.csv"
now_str = datetime.datetime.now().strftime("%m:%d:%Y, %H:%M:%S")

if not os.path.isfile(results_filepath):
    column_names = [
        "robot","path","traj_is_correct_length","joint_limits_violated","planning_time_sec","target_path_length","generated_trajectory_length",
        "mean_positional_error_mm","min_positional_error_mm","max_positional_error_mm","std_positional_error_mm",
        "mean_rotational_error_deg", "min_rotational_error_deg", "max_rotational_error_deg", "std_rotational_error_deg",
        "mean_per_timestep_mjac_deg", "std_per_timestep_mjac_deg", "max_per_timestep_mjac_deg", "mean_per_timestep_mjac_cm", "std_per_timestep_mjac_cm", "max_per_timestep_mjac_cm", 
        "timestamp","method"
    ]
    df = pd.DataFrame(columns=column_names)
else:
    # First, backup file
    backup_filepath = results_filepath.replace(filename, f"backups/{filename}-backup {now_str}")
    shutil.copyfile(results_filepath, backup_filepath)
    df = pd.read_csv(results_filepath, index_col=[0]) # https://stackoverflow.com/a/54358758/5191069

print(df)

# <index>,robot,path,traj_is_correct_length,planning_time_sec,target_path_length,generated_trajectory_length,mean_positional_error_mm,min_positional_error_mm,max_positional_error_mm,std_positional_error_mm,mean_rotational_error_deg,min_rotational_error_deg,max_rotational_error_deg,std_rotational_error_deg,timestamp,method
df.loc[len(df.index)] = [
    robot.name,
    target_path_name,
    traj_is_correct_length,
    plan.joint_limits_violated,
    planning_time_sec,
    target_path.shape[0],
    qpath.shape[0],
    mean_pos_error_mm,
    min_pos_error_mm,
    max_pos_error_mm,
    std_pos_error_mm,
    mean_rot_error_deg,
    min_rot_error_deg,
    max_rot_error_deg,
    std_rot_error_deg,
    mean_per_timestep_mjac_deg,
    std_per_timestep_mjac_deg,
    max_per_timestep_mjac_deg,
    mean_per_timestep_mjac_cm,
    std_per_timestep_mjac_cm,
    max_per_timestep_mjac_cm,
    now_str,
    "stampede"
]

df.to_csv(results_filepath)

print(df)