from sys import stderr
import tkinter as tk
from tkinter import ttk

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import os
base_dir = os.getcwd()
dll_path = os.path.join(base_dir, "x64")
os.add_dll_directory(dll_path)
import starkit_ik_walk as sk

params = sk.IKWalkParameters()

params.distHipToKnee = 0.093
params.distKneeToAnkle = 0.105
params.distAnkleToGround = 0.032
params.distFeetLateral = 0.092
params.freq = 1.7
params.enabledGain = 0.5
params.supportPhaseRatio = 0.0
params.footYOffset = 0.025
params.stepGain = 0.15
params.riseGain = 0.035
params.turnGain = 0.0
params.lateralGain = 0.0
params.trunkZOffset = 0.02
params.swingGain = 0.02
params.swingRollGain = 0.0
params.swingPhase = 0.25
params.stepUpVel = 4.0
params.stepDownVel = 4.0
params.riseUpVel = 4.0
params.riseDownVel = 4.0
params.swingPause = 0.0
params.swingVel = 4.0
params.trunkXOffset = 0.02
params.trunkYOffset = 0.0
params.trunkPitch = 0.15
params.trunkRoll = 0.0
params.extraLeftX = 0.0
params.extraLeftY = 0.0
params.extraLeftZ = 0.0
params.extraRightX = 0.0
params.extraRightY = 0.0
params.extraRightZ = 0.0
params.extraLeftYaw = 0.0
params.extraLeftPitch = 0.0
params.extraLeftRoll = 0.0
params.extraRightYaw = 0.0
params.extraRightPitch = 0.0
params.extraRightRoll = 0.0

phase = 0.0

labels = {}
button_enabled = None

attribute_ranges = {
    "stepGain": (-0.1, 0.1),
    "lateralGain": (-0.06, 0.06),
    "turnGain": (-0.5, 0.5),
    "freq": (0.1, 5.0),
    "supportPhaseRatio": (0.0, 1.0),
    "footYOffset": (-0.2, 0.2),
    "riseGain": (0.0, 0.1),
    "swingGain": (0.0, 0.1),
    "swingRollGain": (-1.0, 1.0),
    "swingPhase": (0.0, 1.0),
    # "stepUpVel": (0.0, 5.0),
    # "stepDownVel": (0.0, 5.0),
    # "riseUpVel": (0.0, 5.0),
    # "riseDownVel": (0.0, 5.0),
    "swingPause": (0.0, 0.5),
    "swingVel": (0.0, 5.0),
    "trunkXOffset": (-0.2, 0.2),
    "trunkYOffset": (-0.2, 0.2),
    "trunkZOffset": (0.01, 0.2),
    "trunkPitch": (-1.0, 1.0),
    "trunkRoll": (-1.0, 1.0)
}

# def update_value(param_name, value):
    # global params
    # global labels
    # print(float(value))
    # setattr(params, param_name, float(value))
    # if param_name in labels.keys():
        # labels[param_name].configure(text=f"{param_name}: {float(value):.3f}")

# def create_window_1():
    # global params
    # global labels
    # global button_enabled
    # global attribute_ranges
    # window = tk.Tk()
    # window.title("Control robot")
    
    # button_enabled = ttk.Button(window, text="Enable", command=lambda: toggle_enabled())
    # button_enabled.pack(padx=10, pady=10)
    
    # for attr_name, (min_value, max_value) in attribute_ranges.items():
        # if attr_name in ("stepGain", "lateralGain", "turnGain"):
            # label = ttk.Label(window, text=f"{attr_name}: {getattr(params, attr_name):.3f}")
            # labels[attr_name] = label
            # label.pack(pady=5)
        
            # trackbar = ttk.Scale(window, from_=min_value, to=max_value, length=200, orient=tk.HORIZONTAL,
                                 # command=lambda value, param_name=attr_name: update_value(param_name, value))
            # trackbar.set(getattr(params, attr_name))
            # trackbar.pack(pady=5)
    # return window

# def create_window_2():
    # global params
    # global labels
    # global attribute_ranges
    # window = tk.Tk()
    # window.title("Parameter settings")
    
    # attribute_names = vars(params).keys()
    
    # trackbars_per_row = 4
    # current_row = 0
    # current_column = 0

    # for attr_name, (min_value, max_value) in attribute_ranges.items():
        # if attr_name not in ("stepGain", "lateralGain", "turnGain"):
            # label = ttk.Label(window, text=f"{attr_name}: {getattr(params, attr_name):.3f}")
            # label.grid(row=current_row, column=current_column, padx=5, pady=5)
            # labels[attr_name] = label

            # trackbar = ttk.Scale(window, from_=min_value, to=max_value, length=200, orient=tk.HORIZONTAL,
                                 # command=lambda value, param_name=attr_name: update_value(param_name, value))
            # trackbar.set(getattr(params, attr_name))
            # trackbar.grid(row=current_row, column=current_column+1, padx=5, pady=5)

            # current_column += 2
            # if current_column >= trackbars_per_row * 2:
                # current_row += 1
                # current_column = 0
    # return window

def toggle_enabled():
    global params
    global labels
    params.enabledGain = 1.0 if params.enabledGain == 0.0 else 0.0
    button_enabled.configure(text="Disable" if params.enabledGain == 1.0 else "Enable")

# w1 = create_window_1()
# w1.geometry("+1200+200")
# w2 = create_window_2()
# w2.geometry("+100+550")

# create the Robot instance.
robot = Robot()

dof_names = [
    'left_elbow',
    'right_elbow',
    'left_hip_yaw', 
    'left_hip_roll', 
    'left_hip_pitch', 
    'left_knee', 
    'left_ankle_pitch', 
    'left_ankle_roll', 
    'right_hip_yaw', 
    'right_hip_roll', 
    'right_hip_pitch', 
    'right_knee', 
    'right_ankle_pitch', 
    'right_ankle_roll']


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
servos = {}
for name in dof_names:
    servos[name] = robot.getDevice(name)

def send_command(command: sk.IKWalkOutputs):
    for name, motor in servos.items():
        if "elbow" in name:
            motor.setPosition(-2.5)
        else:
            motor.setPosition(getattr(command, name))

while robot.step(timestep) != -1:      
    # w1.update_idletasks()   
    # w1.update()   
    # w2.update_idletasks() 
    # w2.update() 
    outputs = sk.IKWalkOutputs()                                
    if sk.IKWalk.walk(params, timestep / 1000.0, phase, outputs):
        send_command(outputs)
        phase = outputs.phase
    else:
        print(" Inverse Kinematics error. Position not reachable.", file=stderr)
        
        
        
        
        
        
        
"""keypoint controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import json
import cv2
import numpy as np
from scipy.interpolate import interp1d
from controller import Robot

dofs = [
    "head_pitch",
    "head_yaw",
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "left_elbow",
    "right_elbow",
    "left_hip_pitch",
    "left_hip_roll",
    "left_hip_yaw",
    "right_hip_pitch",
    "right_hip_roll",
    "right_hip_yaw",
    "left_knee",
    "right_knee",
    "left_ankle_pitch",
    "left_ankle_roll",
    "right_ankle_pitch",
    "right_ankle_roll"
]

def deg2rad(deg):
    return deg / 180 * math.pi
def rad2deg(rad):
    return rad / math.pi * 180


def read_json(file_path):
    with open(file_path, 'r') as file:
        data = json.load(file)
    refactored_data = {}
    for joint_name in data.keys():
        if joint_name not in ("over", "remap", "head_yaw", "head_pitch"):
            if "left" not in joint_name:
                refactored_data["left_" + joint_name] = data[joint_name]
            if "right" not in joint_name:
                refactored_data["right_" + joint_name] = data[joint_name]
        else:
            refactored_data[joint_name] = data[joint_name]
    return refactored_data
    
def interpolate(motion):
    motion_trajectory = {}
    for joint in motion.items():
        joint_name = joint[0] 
        x = []
        y = []
        points = joint[1]
        for point in points:
            x.append(point[0])
            y.append(point[1])
        if joint_name not in ("over", "remap"):
            joint_trajectory = interp1d(x, y, kind='linear', fill_value='extrapolate')
        else:
            joint_trajectory = interp1d(x, y, kind='zero', fill_value='extrapolate')
        motion_trajectory[joint_name] = joint_trajectory       
    return motion_trajectory
    
def send_commands(commands):
    for joint_name, value in commands.items():
        if joint_name not in ("over", "remap"):
            servos[joint_name].setPosition(deg2rad(value))

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

motion = read_json("fallonface.json")

motion_timer = 0
motion_trajectory = interpolate(motion)

servos = {}

for joint_name in motion_trajectory.keys():
    if joint_name not in ("over", "remap"):
        servos[joint_name] = robot.getDevice(joint_name)
    
while robot.step(timestep) != -1:
    time_factor = motion_trajectory["remap"](motion_timer)
    motion_timer += time_factor * timestep / 1000 
    if not motion_trajectory["over"](motion_timer):
        commands = {}
        for joint_name, trajectory in motion_trajectory.items(): 
            target = trajectory(motion_timer)
            commands[joint_name] = target
    send_commands(commands)