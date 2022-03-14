#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryFeedback
from std_msgs.msg import UInt8MultiArray
from industrial_msgs.msg import RobotStatus

import collections
import math


class amber_robot_link():

    def __init__(self):
        rospy.init_node('Amber_linking_node', anonymous=True)

        self.rate = rospy.Rate(10)

        self.joint_subscriber = rospy.Subscriber("/move_group/fake_controller_joint_states", JointState,
                                                 self.read_moveit_joint_callback)
        self.joint_publisher = rospy.Publisher("/position_trajectory_controller/joint_trajectory", JointTrajectory, queue_size=1)


        self.fake_joint_names = ['Rev2', 'Rev3', 'Rev6', 'Rev7', 'Rev9', 'Rev10', 'Rev12']
        self.robot_joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        self.count = 0
        self.send_to_robot_JointTrajectory = JointTrajectory()
        self.points=JointTrajectoryPoint()

        print("Amber_linking_node started!!")

    def read_moveit_joint_callback(self, Joints_state):
        """Retrieves live position feedback and publishes the data
        to its corresponding topic. (infinite loop)
        """
        while not rospy.is_shutdown():
            try:
                # Robot Status Feedback
                if (len(Joints_state.position) > 0):
                    self.send_to_robot_JointTrajectory = JointTrajectory()
                    self.points = JointTrajectoryPoint()
                    print(Joints_state)
                    #print("Joint values", Joints_state)
                    #print("#############################################")
                    #print("Joint position 0", Joints_state.position[5])
                    #print("Joint position 1", Joints_state.position[6])
                    #print("Joint position 2", Joints_state.position[0])
                    #print("Joint position 3", Joints_state.position[1])
                    #print("Joint position 4", Joints_state.position[2])
                    #print("Joint position 5", Joints_state.position[3])
                    #print("Joint position 6", Joints_state.position[4])
                    #rospy.loginfo("Joint position values", Joints_state.position)
                    #print("########### 1 ###########",self.send_to_robot_JointTrajectory.points)

                    self.points.positions.append(Joints_state.position[2])
                    print("################### 2 #######################")
                    self.points.positions.append(Joints_state.position[3])
                    self.points.positions.append(Joints_state.position[4])
                    self.points.positions.append(Joints_state.position[5])
                    self.points.positions.append(Joints_state.position[6])
                    self.points.positions.append(Joints_state.position[0])
                    self.points.positions.append(Joints_state.position[1])
                    print("#################### 3 ######################")
                    self.send_to_robot_JointTrajectory.joint_names.append(self.robot_joint_names[0])
                    self.send_to_robot_JointTrajectory.joint_names.append(self.robot_joint_names[1])
                    self.send_to_robot_JointTrajectory.joint_names.append(self.robot_joint_names[2])
                    self.send_to_robot_JointTrajectory.joint_names.append(self.robot_joint_names[3])
                    self.send_to_robot_JointTrajectory.joint_names.append(self.robot_joint_names[4])
                    self.send_to_robot_JointTrajectory.joint_names.append(self.robot_joint_names[5])
                    self.send_to_robot_JointTrajectory.joint_names.append(self.robot_joint_names[6])
                    """self.send_to_robot_JointTrajectory.joint_names[0]=self.robot_joint_names[0]
                    self.send_to_robot_JointTrajectory.joint_names[1] = self.robot_joint_names[1]
                    self.send_to_robot_JointTrajectory.joint_names[2] = self.robot_joint_names[2]
                    self.send_to_robot_JointTrajectory.joint_names[3] = self.robot_joint_names[3]
                    self.send_to_robot_JointTrajectory.joint_names[4] = self.robot_joint_names[4]
                    self.send_to_robot_JointTrajectory.joint_names[5] = self.robot_joint_names[5]"""
                    #self.send_to_robot_JointTrajectory.header.stamp= rospy.Time.now()
                    self.points.time_from_start.nsecs=600000000  #.rospy.Time.now()

                    #self.send_to_robot_JointTrajectory.points = self.points
                    #self.joint_publisher.publish(self.send_to_robot_JointTrajectory)
                    print("##################### 4 ######################")
                    #self.joint_publisher.publish(self.send_to_robot_JointTrajectory)
                    #test=JointTrajectory()
                    #test.header.stamp = rospy.Time.now()
                    #test.points=JointTrajectoryPoint()
                    #self.joint_publisher.publish(test)
                    #print("Joint values to Robot", self.points)
                    #print("##################### 5 ######################")
                    #print("send_to_robot_JointTrajectory values to Robot", self.send_to_robot_JointTrajectory)
                    #print("##################### 6 ######################")
                    self.send_to_robot_JointTrajectory.points.append(self.points)
                    print("all values to Robot", self.send_to_robot_JointTrajectory)
                    print("##################### 7 ######################")
                    self.joint_publisher.publish(self.send_to_robot_JointTrajectory)

                    print("##################### 8 ######################")
                    break
            except:
                #print("read_moveit_joint_callback error.")
                break

if __name__ == "__main__":
    # robot_ctrl = MecademicRobot_Controller()
    robot_linking = amber_robot_link()

    rospy.spin()

    # robot_ctrl.set_target_pose(pose_ctrl)
