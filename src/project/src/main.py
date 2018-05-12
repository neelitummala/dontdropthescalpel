#!/usr/bin/env python
# Some code is from takktile_example_code.py

from math import pi, cos
import matplotlib.pyplot as plt
import numpy as np

import rospy
from std_srvs.srv import Empty

from reflex_msgs.msg import Command
from reflex_msgs.msg import PoseCommand
from reflex_msgs.msg import VelocityCommand
from reflex_msgs.msg import ForceCommand
from reflex_msgs.msg import Hand
from reflex_msgs.msg import FingerPressure
from reflex_msgs.srv import SetTactileThreshold, SetTactileThresholdRequest
from std_msgs.msg import String
import subprocess
import sys
import calibrate

hand_state = Hand()

rospy.init_node('TakktileHandNode')

# Services can set tactile thresholds and enable tactile stops
enable_tactile_stops = rospy.ServiceProxy('/reflex_takktile/enable_tactile_stops', Empty)
disable_tactile_stops = rospy.ServiceProxy('/reflex_takktile/disable_tactile_stops', Empty)
set_tactile_threshold = rospy.ServiceProxy('/reflex_takktile/set_tactile_threshold', SetTactileThreshold)

# This collection of publishers can be used to command the hand
command_pub = rospy.Publisher('/reflex_takktile/command', Command, queue_size=1)
pos_pub = rospy.Publisher('/reflex_takktile/command_position', PoseCommand, queue_size=1)
vel_pub = rospy.Publisher('/reflex_takktile/command_velocity', VelocityCommand, queue_size=1)
force_pub = rospy.Publisher('/reflex_takktile/command_motor_force', ForceCommand, queue_size=1)
# classify_pub = rospy.Publisher('classify', String, queue_size=1)

def main():

    # Constantly capture the current hand state
    rospy.Subscriber('/reflex_takktile/hand_state', Hand, hand_state_cb)

    ##################################################################################################################

    # Open hand
    rospy.sleep(1)
    calibrate.open_hand()
    rospy.sleep(1)

    subprocess.call("./black.sh")

    raw_input("== When ready to calibrate tactile sensors and close until contact, hit [Enter]\n")
    calibrate.calibrate_tactile()
    pos_pub.publish(PoseCommand(preshape=0.0))
    
    # crush()
    # threshold_control(15)
    print(str(sys.argv))
    if(len(sys.argv) == 1):
        print("no command given")
    elif (sys.argv[1] == 'c'):
        classify(50)
    elif sys.argv[1] == 't':
        tug_of_war(15);
    elif sys.argv[1] == 's':
        threshold_control(15,5,[2,2])
    elif sys.argv[1] == 'd':
        shake_demo()
    else:
        print("bye")
        return


    # raw_input("... [Enter]\n")
    # pos_pub.publish(PoseCommand(preshape=0.5))
    # disable_tactile_stops()
    # tug_of_war(15)
    # touch(3)
    # plot_pressures(1)

def crush():
    touch(1000)

def shake_demo():
    subprocess.call("./home.sh")
    threshold_control(15,5,[2,2])
    print("Starting shake")
    subprocess.call("./big_shake.sh")
    

def touch(val):
    enable_tactile_stops()
    f1 = FingerPressure([0, 0, 0, 0, 0, 0, 0, 0, 0])
    f2 = FingerPressure([val, val, val, val, val, 1000, 1000, 1000, 1000])
    f3 = FingerPressure([val, val, val, val, val, 1000, 1000, 1000, 1000])
    threshold = SetTactileThresholdRequest([f1, f2, f3])
    set_tactile_threshold(threshold)
    vel_pub.publish(VelocityCommand(f1=0.0, f2=1, f3=1, preshape=0.0))
    rospy.sleep(5.0)
    disable_tactile_stops()

def classify(threshold):
    # touch(1) initial_angle = np.array([hand_state.motor[1].joint_angle,
    # hand_state.motor[2].joint_angle]) print("initial angle: " +
    # str(initial_angle)) touch(threshold) final_angle =
    # np.array([hand_state.motor[1].joint_angle,
    # hand_state.motor[2].joint_angle]) print("final angle: "+
    # str(final_angle)) print("difference: "+ str(list( np.abs(final_angle -
    # initial_angle)))) if abs(initial_angle[0]-final_angle[0])<= 0.03:
    # print("Hard object") else:     print("Soft boi")
    subprocess.call("./black.sh")
    touch(1)
    vals1 = []
    vals2 = []
    for i in range(0, 20):
        vals1.append(hand_state.motor[1].load)
        vals2.append(hand_state.motor[2].load)
    initial_vals = np.array([np.mean(vals1), np.mean(vals2)])
    print(initial_vals)

    touch(threshold)
    vals1 = []
    vals2 = []
    for i in range(0, 20):
        vals1.append(hand_state.motor[1].load)
        vals2.append(hand_state.motor[2].load)
    final_vals = np.array([np.mean(vals1), np.mean(vals2)])
    print(final_vals)
    print(np.abs(final_vals - initial_vals))
    difference = np.abs(final_vals - initial_vals)
    if difference[0] < 10 or difference[1] < 10:
        print("RIGID")
        subprocess.call("./pic_rigid.sh")

        # classify_pub.publish("RIGID")
    else:
        print("DEFORMABLE")
        subprocess.call("./pic_deform.sh")

        # classify_pub.publish("DEFORMABLE")

def threshold_control(sim_time, threshold, slope_threshold=[3, 3]):
    # Control the finger pressures based on threshold. Most naive approach.
    print('Starting threshold controller')
    f1 = [0, 0, 0, 0, 0, 0, 0, 0, 0] # don't move this finger because it's broken
    f2 = [threshold, threshold, threshold, threshold, threshold, 1000, 1000, 1000, 1000]
    f3 = [threshold, threshold, threshold, threshold, threshold, 1000, 1000, 1000, 1000]

    enable_tactile_stops()
    set_tactile_threshold(SetTactileThresholdRequest([FingerPressure(f1), FingerPressure(f2), FingerPressure(f3)]))
    vel_pub.publish(VelocityCommand(f1=0.0, f2=1.5, f3=1.5, preshape=0.0))
    rospy.sleep(3)
    disable_tactile_stops()
    print("Start grasp")
    start_time = rospy.get_time()
    now = start_time
    prev_values_2 = np.array(hand_state.finger[1].pressure)[:5]
    prev_values_3 = np.array(hand_state.finger[2].pressure)[:5]
    while (now - start_time) < sim_time:
        now = rospy.get_time()
        curr_values_2 = np.array(hand_state.finger[1].pressure)
        curr_values_2 = curr_values_2[:5]
        curr_values_3 = np.array(hand_state.finger[2].pressure)
        curr_values_3 = curr_values_3[:5]


        f1_list = []
        f2_list = []

        for x in (curr_values_2 - prev_values_2):
            if (x > slope_threshold[0]):
                f1_list.append(x)

        for x in (curr_values_3 - prev_values_3):
            # print list[curr_values_3 - prev_values_3])
            if (x > slope_threshold[1]):
                f2_list.append(x)


        if len(f1_list) > 0 or len(f2_list) > 0:
            print("Slipping, increasing threshold")
            f2 = [x+2 for x in f2]
            f3 = [x+2 for x in f3] 
            enable_tactile_stops()
            set_tactile_threshold(SetTactileThresholdRequest([FingerPressure(f1), FingerPressure(f2), FingerPressure(f3)]))
            vel_pub.publish(VelocityCommand(f1=0.0, f2=1.0, f3=1.0, preshape=0.0))
            disable_tactile_stops()
        prev_values_2 = curr_values_2
        prev_values_3 = curr_values_3

    # print('starting big_shake')
    # subprocess.call("./big_shake.sh")

def plot_pressures(finger):
    state = []
    start_time = rospy.get_time()
    now = start_time
    while (now - start_time) < 10:
        now = rospy.get_time()
        state.append(hand_state.finger[finger].pressure)
    fig, ax = plt.subplots()
    ax.plot(state)
    ax.legend(["1", "2", "3", "4", "5", "6", "7", "8", "9"])
    plt.show()

def plot_frequencies(sim_time, finger, sensor):
    print('Measuring frequencies')
    raw_input("... [Enter]\n")
    num_samples = 0
    state = []
    start_time = rospy.get_time()
    now = start_time
    while (now - start_time) < sim_time:
        now = rospy.get_time()
        state.append(hand_state.finger[finger].pressure[sensor])
        num_samples += 1
    dft = np.fft.fft(state)
    N = len(dft)
    fs = num_samples / (now-start_time)
    print('Sampling rate = '+str(fs))
    freqs = [i*fs/N for i in range(N)]
    plt.plot(freqs[1:2000], np.abs(dft)[1:2000])
    plt.show()

def get_frequencies(sim_time, finger, sensor):
    print('Measuring frequencies')
    raw_input("... [Enter]\n")
    num_samples = 0
    start_time = rospy.get_time()
    now = start_time
    state = []
    while (now - start_time) <= sim_time:
        now = rospy.get_time()
        state.append(hand_state.finger[finger].pressure[sensor])
        num_samples += 1
    dft = np.fft.fft(state)
    # print(np.mean(np.abs(dft)[1:2000]))
    print(np.amax(np.abs(dft)[0:2000]))

def tug_of_war(sim_time, slope_threshold=[2, 2]):
    # Control the finger pressures based on threshold. Most naive approach.
    print('Starting threshold controller')
    f1 = [0, 0, 0, 0, 0, 0, 0, 0, 0] # don't move this finger because it's broken
    f2 = [10, 10, 10, 10, 10, 1000, 1000, 1000, 1000]
    f3 = [10, 10, 10, 10, 10, 1000, 1000, 1000, 1000]

    enable_tactile_stops()
    set_tactile_threshold(SetTactileThresholdRequest([FingerPressure(f1), FingerPressure(f2), FingerPressure(f3)]))
    vel_pub.publish(VelocityCommand(f1=0.0, f2=1.0, f3=1.0, preshape=0.0))
    rospy.sleep(5)
    disable_tactile_stops()
    print("Start grasp")

    start_time = rospy.get_time()
    now = start_time
    prev_values_2 = np.array(hand_state.finger[1].pressure)[:5]
    prev_values_3 = np.array(hand_state.finger[2].pressure)[:5]
    while (now - start_time) < sim_time:
        now = rospy.get_time()
        curr_values_2 = np.array(hand_state.finger[1].pressure)
        curr_values_2 = curr_values_2[:5]
        curr_values_3 = np.array(hand_state.finger[2].pressure)
        curr_values_3 = curr_values_3[:5]


        f1_list = []
        f2_list = []

        for x in (curr_values_2 - prev_values_2):
            if (x > slope_threshold[0]):
                f1_list.append(x)

        for x in (curr_values_3 - prev_values_3):
            # print list[curr_values_3 - prev_values_3])
            if (x > slope_threshold[1]):
                f2_list.append(x)


        if len(f1_list) > 0 or len(f2_list) > 0:
            print("Slipping, increasing threshold")
            f2 = [x+80 for x in f2]
            f3 = [x+80 for x in f3] 
            print(f2)
            print(f3)
            enable_tactile_stops()
            set_tactile_threshold(SetTactileThresholdRequest([FingerPressure(f1), FingerPressure(f2), FingerPressure(f3)]))
            vel_pub.publish(VelocityCommand(f1=0.0, f2=1.0, f3=1.0, preshape=0.0))
            disable_tactile_stops()
        prev_values_2 = curr_values_2
        prev_values_3 = curr_values_3

def hand_state_cb(data):
    global hand_state
    hand_state = data

if __name__ == '__main__':
    main()
