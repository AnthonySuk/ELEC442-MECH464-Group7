#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from rospy import Time

# Task states
FINDINGTARGET = 0
FINDINGTARGET_2_MOVE = 1
MOVE = 2
DONE = 3

class TargetPositionPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('target_position_publisher', anonymous=True)

        # Publisher for the target_position topic
        self.pub = rospy.Publisher('/target_position', Float32MultiArray, queue_size=10)

        # Initialize state and variables
        self.curr_taskState = FINDINGTARGET
        self.targetPosition = [0.6, 0, 0.5]  # Target position (x, y, z)
        self.R_target = np.eye(3)  # Target rotation matrix (identity for now)
        self.initial_pose_ = np.eye(4)  # 4x4 identity matrix (for example)
        self.move_start_time_ = None
        self.motion_duration_ = 15.0
        self.current_time = 0.0
        self.new_pose = None  # To store updated pose

    def run(self):
        rate = rospy.Rate(10)  # Loop rate (10 Hz)

        while not rospy.is_shutdown():
            self.current_time = rospy.get_time()  # Get the current ROS time
            self.update_task_state()  # Update task state
            self.handle_state_calculations()  # Perform calculations for the current state
            self.publish_target_position()  # Publish the updated target position

            rate.sleep()

    def update_task_state(self):
        if self.curr_taskState == FINDINGTARGET:
            # Transition to FINDINGTARGET_2_MOVE
            rospy.loginfo("Transitioning from FINDINGTARGET to FINDINGTARGET_2_MOVE.")
            self.curr_taskState = FINDINGTARGET_2_MOVE

        elif self.curr_taskState == FINDINGTARGET_2_MOVE:
            # Set the initial pose and start time
            rospy.loginfo("Setting initial pose and starting motion.")
            self.initial_pose_ = np.array([[1, 0, 0, 0],
                                           [0, 1, 0, 0],
                                           [0, 0, 1, 0],
                                           [0, 0, 0, 1]])  # Starting pose
            self.move_start_time_ = rospy.get_time()  # Capture the start time
            self.curr_taskState = MOVE

        elif self.curr_taskState == MOVE:
            # Perform interpolation based on time
            t = self.current_time - self.move_start_time_
            #rospy.loginfo(t)
            if t > self.motion_duration_:
                t = self.motion_duration_

            s = 0.5 * (1 - np.cos(np.pi * t / self.motion_duration_))
            rospy.loginfo(s)
            # Interpolate position
            pstart = self.initial_pose_[:3, 3]
            self.new_pose = self.initial_pose_.copy()
            self.new_pose[0, 3] = (1 - s) * pstart[0] + s * self.targetPosition[0]
            self.new_pose[1, 3] = (1 - s) * pstart[1] + s * self.targetPosition[1]
            self.new_pose[2, 3] = (1 - s) * pstart[2] + s * self.targetPosition[2]

            rospy.loginfo("Target pose updated: %.4f, %.4f, %.4f", self.new_pose[0, 3], self.new_pose[1, 3], self.new_pose[2, 3])

            # Transition to DONE state after motion
            if t >= self.motion_duration_:
                rospy.loginfo("Motion completed, transitioning to DONE state.")
                self.curr_taskState = DONE

    def handle_state_calculations(self):
        # This method is now used to perform any state-specific calculations
        # (e.g., pose interpolation) when entering or staying in a state.
        pass  # Currently not doing anything else since logic is in update_task_state

    def publish_target_position(self):
        if self.new_pose is not None:
            # Convert the 4x4 matrix to a 1D list for ROS message
            target_pose_flat = self.new_pose.flatten().tolist()

            # Create the ROS message
            msg = Float32MultiArray()
            msg.data = target_pose_flat

            # Publish the message
            self.pub.publish(msg)

            rospy.loginfo("Published target position to /target_position.")

if __name__ == '__main__':
    try:
        target_position_publisher = TargetPositionPublisher()
        target_position_publisher.run()
    except rospy.ROSInterruptException:
        pass
