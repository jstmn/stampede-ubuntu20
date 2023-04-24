#!/usr/bin/env julia
println("Julia JIT compilation...")

include("Stampede/Utils_Julia/relaxed_ik_utils.jl")
include("Stampede/Utils_Julia/ik_task.jl")
include("Stampede/stampede2.jl")
include("Stampede/Utils_Julia/stampede_utils.jl")
path_to_src = Base.source_dir()
relaxedik_path = get_relaxedik_path(path_to_src)
using RobotOS
using Dates

init_node("stampede")

#task_name = "spirals"
task_name = get_param("input_motion_file")
scaling_factor = get_param("scaling_factor")



#scaling_factor = 0.8
pos_goals, quat_goals = get_ik_task(path_to_src, task_name, scaling_factor=scaling_factor)

include(relaxedik_path * "/RelaxedIK/relaxedIK.jl")
include(relaxedik_path * "/RelaxedIK/Utils_Julia/joint_utils.jl")
# loaded_robot_file = open(relaxedik_path * "/RelaxedIK/Config/loaded_robot")
# loaded_robot = readline(loaded_robot_file)
# close(loaded_robot_file)
loaded_robot = get_param("robot_info_file")
println("loaded_robot: ", loaded_robot)
@assert loaded_robot == "fetch_arm_info.yaml" || loaded_robot == "fetch_info.yaml"
if loaded_robot == ""
    println("ERROR: loaded_robot must be set in the stampede.launch file!")
    exit(-1)
end

relaxedIK = get_standard(relaxedik_path, loaded_robot) # returns a RelaxedIK object. ee pose goals are relative
baseIK = get_base_ik(relaxedik_path, loaded_robot)
init_pos = relaxedIK.relaxedIK_vars.init_ee_positions[1]

# Update path to be in the end effector frame
# world__T__offset_frame + offset_frame__T__path0 = world__T__end_eff0 + end_eff__T__path0
# --> world__T__offset_frame + offset_frame__T__path0 - world__T__end_eff0 = end_eff__T__path0

# FETCH_ARM
# world__T__torso_lift_link: [-0.086875  0.   0.37743   1.       -0.        0.        0.      ]

path_offset__T__path0_dict = Dict{Tuple{String, String}, SVector}(
    ("fetch_info.yaml", "cppflow__circle") => SVector(0.9, 0.25, 0.46),
    ("fetch_info.yaml", "cppflow__hello") => SVector(0.8, 0.45, 0.25),
    ("fetch_info.yaml", "cppflow__rotation") => SVector(0.8, 0.0, 0.35),
    ("fetch_info.yaml", "cppflow__s") => SVector(1.0, 0.3, 0.55),
    ("fetch_info.yaml", "cppflow__square") => SVector(1.1, 0.0, 0.66),
    ("fetch_arm_info.yaml", "cppflow__circle") => SVector(0.9, 0.25, 0.46),
    ("fetch_arm_info.yaml", "cppflow__hello") => SVector(0.8, 0.45, 0.25),
    ("fetch_arm_info.yaml", "cppflow__rotation") => SVector(0.8, 0.0, 0.35),
    ("fetch_arm_info.yaml", "cppflow__s") => SVector(1.0, 0.3, 0.55),
    ("fetch_arm_info.yaml", "cppflow__square") => SVector(1.1, 0.0, 0.66)
)
 
world__T__offset_frame_dict = Dict{String, SVector}(
    "fetch_arm_info.yaml" => SVector(-0.086875, 0., 0.37743),
    "fetch_info.yaml" => SVector(-0.086875, 0., 0.37743)
)

world__T__offset_frame = world__T__offset_frame_dict[loaded_robot]
offset_frame__T__path0 = path_offset__T__path0_dict[(loaded_robot, task_name)]
world__T__end_eff0 = SVector(init_pos[1], init_pos[2], init_pos[3])
end_eff__T__path0 = world__T__offset_frame + offset_frame__T__path0 - world__T__end_eff0

println("world__T__end_eff0: ", world__T__end_eff0)
println("end_eff__T__path0: ", end_eff__T__path0)

pos_goals_updated = []
for i = 1:length(pos_goals)
    pos = pos_goals[i] + end_eff__T__path0
    push!(pos_goals_updated, pos)
end
@assert length(pos_goals_updated) == length(pos_goals)
pos_goals = pos_goals_updated


# Run the solver
t0 = now()
s = StampedeObj(relaxedIK, baseIK, pos_goals, quat_goals, 15.0)
traj = solve(s)
runtime_ms = now() - t0
runtime_ms = runtime_ms.value
runtime_sec = runtime_ms / 1000
println("runtime millseconds: ", runtime_ms)
println("runtime seconds:     ", runtime_sec)

fp = path_to_src * "/Stampede/OutputMotions/last_trajectory.stampede"
f = open(fp, "w")

successful = length(traj) == length(pos_goals)
write(f, "$loaded_robot, $task_name, $scaling_factor, $(init_pos[1]), $(init_pos[2]), $(init_pos[3]), $(runtime_sec), $(successful)\n")

println("Trajectory length:", length(traj))

for i = 1:length(traj)
    time = s.times[i]
    write(f, "$time; ")
    x = traj[i]
    # print(i, x)
    for j = 1:length(x)-1
        write(f, "$(x[j]), ")
    end
    write(f, "$(x[end]);")
    p = pos_goals[i]
    q = quat_goals[i]

    write(f, "$(p[1]), $(p[2]), $(p[3]);")
    write(f, "$(q[1]), $(q[2]), $(q[3]), $(q[4])\n")
end

close(f)
