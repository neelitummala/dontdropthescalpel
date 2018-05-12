#!/usr/bin/env python
import calibrate
import rospy

def main():
	rospy.sleep(1)
	calibrate.open_hand()
	rospy.sleep(1)

if __name__ == '__main__':
    main()