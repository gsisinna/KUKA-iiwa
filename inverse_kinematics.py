#!/usr/bin/env/python
import numpy as np
from numpy import array
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

cos = np.cos
sin = np.sin
pi = np.pi

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('inverse_kinematics', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "iiwa_arm"
group = moveit_commander.MoveGroupCommander(group_name)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

print "============ Printing robot state"
print robot.get_current_state()
print ""

#Per ora setto tutto a zero, ma i valori mi arriveranno dal topic /joint_states (O da robot.get_current_state() ? )
q_old = np.array([pi/4, pi/2, -pi/2, pi/4, pi/2, pi/2, -pi/2])
q = q_old

#end-effector position
ee_pos_old = [0.0, 0.0, 0.0]
ee_pos = ee_pos_old

#end-effector velocity
ee_vel_old = [0.0, 0.0, 0.0]
ee_vel = ee_vel_old

#dt
h=0.1

#jacobiano di posizione --> la i-esima colonna di Jp è uguale a d(ee_pos)/d(qi)
Jp_old = np.zeros((6, 7))
Jp = np.zeros((6, 7))

#jacobiano di orientazione
Jo_old = np.zeros((6, 7))
Jo = np.zeros((6, 7))

Ke = 2/h * np.ones((6,7)) #Stabilità per valori minori di 2/h

#errore
err_old = np.Inf
err = err_old

#threshold
th = 0.001

while err >= th:
    # inserire qui calcolo jacobiano posizione
    for j in range(7):
        for i in range(3):
            J((i,j)) = ((ee_pos(i)-ee_pos_old(i)/h)*( h / (q(j)-q_old(j) ) )

    for j in range(7):
        for i in range(3):



    



    q = q_old + h*np.linalg.inv(J_old)*(ee_vel_old + Ke)
    #aggiorno errore
    err = (np.identity() - Ke*h)*err_old