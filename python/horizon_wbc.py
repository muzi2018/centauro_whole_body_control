from horizon.problem import Problem
from horizon.rhc.model_description import FullModelInverseDynamics
from horizon.rhc.taskInterface import TaskInterface
from horizon.ros import replay_trajectory

import phase_manager.pymanager as pymanager
import phase_manager.pyphase as pyphase

from xbot_interface import config_options as co
from xbot_interface import xbot_interface as xbot

import casadi_kin_dyn.py3casadi_kin_dyn as casadi_kin_dyn

import casadi as cs
import numpy as np
import rospy
import subprocess
import os


rospy.init_node('horizon_wbc_node')

# load urdf and srdf to create model
urdf = rospy.get_param('robot_description', default='')
if urdf == '':
    raise print('urdf not set')

srdf = rospy.get_param('robot_description_semantic', default='')
if srdf == '':
    raise print('urdf not set')

T = 5.
ns = 100
dt = T / ns

prb = Problem(ns, receding=True, casadi_type=cs.SX)
prb.setDt(dt)

# try construct RobotInterface
cfg = co.ConfigOptions()
cfg.set_urdf(urdf)
cfg.set_srdf(srdf)
cfg.generate_jidmap()
cfg.set_string_parameter('model_type', 'RBDL')
cfg.set_string_parameter('framework', 'ROS')
cfg.set_bool_parameter('is_model_floating_base', True)

# robot = xbot.RobotInterface(cfg)
# robot.sense()

# q_init = robot.getJointPosition()
q_init = {
    "ankle_pitch_1": -0.301666,
    "ankle_pitch_2": 0.301666,
    "ankle_pitch_3": 0.301667,
    "ankle_pitch_4": -0.30166,
    "ankle_yaw_1": 0.746874,
    "ankle_yaw_2": -0.746874,
    "ankle_yaw_3": -0.746874,
    "ankle_yaw_4": 0.746874,
    "d435_head_joint": 0,
    "hip_pitch_1": -1.25409,
    "hip_pitch_2": 1.25409,
    "hip_pitch_3": 1.25409,
    "hip_pitch_4": -1.25409,
    "hip_yaw_1": -0.746874,
    "hip_yaw_2": 0.746874,
    "hip_yaw_3": 0.746874,
    "hip_yaw_4": -0.746874,
    "j_arm1_1": 0.520149,
    "j_arm1_2": 0.320865,
    "j_arm1_3": 0.274669,
    "j_arm1_4": -2.23604,
    "j_arm1_5": 0.0500815,
    "j_arm1_6": -0.781461,
    "j_arm2_1": 0.520149,
    "j_arm2_2": -0.320865,
    "j_arm2_3": -0.274669,
    "j_arm2_4": -2.23604,
    "j_arm2_5": -0.0500815,
    "j_arm2_6": -0.781461,
    "knee_pitch_1": -1.55576,
    "knee_pitch_2": 1.55576,
    "knee_pitch_3": 1.55576,
    "knee_pitch_4": -1.55576,
    "torso_yaw": 3.56617e-13,
    "velodyne_joint": 0,
    "dagana_2_claw_joint": 0.
}

base_init = np.array([0, 0, 0, 0, 0, 0, 1])

urdf = urdf.replace('continuous', 'revolute')

fixed_joints_map = dict()
fixed_joints_map.update({'j_wheel_1': 0., 'j_wheel_2': 0., 'j_wheel_3': 0., 'j_wheel_4': 0.})

kin_dyn  = casadi_kin_dyn.CasadiKinDyn(urdf, fixed_joints=fixed_joints_map)

model = FullModelInverseDynamics(problem=prb,
                                 kd=kin_dyn,
                                 q_init=q_init,
                                 base_init=base_init,
                                 fixed_joint_map=fixed_joints_map)

bashCommand = 'rosrun robot_state_publisher robot_state_publisher'
process = subprocess.Popen(bashCommand.split(), start_new_session=True)

ti = TaskInterface(prb=prb, model=model)
ti.setTaskFromYaml(os.getcwd() + '/centauro_wbc_config.yaml')

pm = pymanager.PhaseManager(ns+1)

c_phases = dict()
for c in model.getContactMap():
    c_phases[c] = pm.addTimeline(f'{c}_timeline')

stance_duration = 10
for c in model.getContactMap():
    stance_phase = pyphase.Phase(stance_duration, f'stance_{c}')
    if ti.getTask(f'{c}') is not None:
        stance_phase.addItem(ti.getTask(f'{c}'))
    else:
        raise Exception(f'Task {c} not found')
    
    c_phases[c].registerPhase(stance_phase)

for c in model.getContactMap():
    stance = c_phases[c].getRegisteredPhase(f'stance_{c}')
    while c_phases[c].getEmptyNodes() > 0:
        c_phases[c].addPhase(stance)

q0 = np.hstack((base_init, list(q_init.values())))
model.q.setBounds(q0, q0, nodes=0)
model.q.setBounds(q0, q0, nodes=ns)
model.v.setBounds(np.zeros(model.nv), np.zeros(model.nv), nodes=0)
model.v.setBounds(np.zeros(model.nv), np.zeros(model.nv), nodes=ns)

f0 = [0, 0, kin_dyn.mass() / 4 * 9.81]
for cname, cforces in model.getContactMap().items():
    for c in cforces:
        c.setInitialGuess(f0)

ti.finalize()
# ti.load_initial_guess()
ti.bootstrap()

solution = ti.solution
