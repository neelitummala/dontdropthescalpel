#!/usr/bin/env python

import rospy
import moveit_commander
import baxter_interface
from baxter_interface import CHECK_VERSION
from std_msgs.msg import String, Int16MultiArray
from geometry_msgs.msg import Pose,Vector3,Quaternion,PoseStamped


rospy.init_node('master', anonymous=True)
moveit_commander.roscpp_initialize('/joint_states:=/robot/joint_states')
robot = moveit_commander.RobotCommander()
# scene = moveit_commander.PlanningSceneInterface()
Arm_right = baxter_interface.Limb('right')
# right_arm = moveit_commander.MoveGroupCommander('right_arm')
# right_arm.set_goal_position_tolerance(0.01)
# right_arm.set_goal_orientation_tolerance(0.01)
print("Getting robot state... ")
self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
self._init_state = self._rs.state().enabled
print("Enabling robot... ")
self._rs.enable()
print("Running. Ctrl-c to quit")

['head_nod', 'head_pan', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2', 'torso_t0']
home = [3.055306234270773, 1.0128108152013444, 0.6902913545484362, 1.048092373322709, 3.0595246814374577, -0.09242234247009617, 1.7130730448710358]

def main():
    names = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    commands = {}
    print(home)
    for t, n in zip(home, names):
        commands[n] = t
    print(commands)

    limb = Arm_right
    current_position = [limb.joint_angle(n) for n in names]
    diffs = [abs(c - w) for c, w in zip(current_position, angles)]
    while sum(diffs) > 0.1:
        limb.set_joint_positions(commands)
        current_position = [limb.joint_angle(n) for n in names] 
        diffs = [abs(c - w) for c, w in zip(current_position, angles)]


if __name__ == "__main__":
    main()