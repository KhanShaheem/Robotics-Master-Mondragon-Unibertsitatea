#!/usr/bin/env python

import sys

import math
import rospy
import moveit_commander
import geometry_msgs.msg


RADIANS_TO_DEGREES = 180.0 / math.pi
DEGREES_TO_RADIANS = math.pi / 180.0


class GroupInterface(object):
    def __init__(self):
        """
        Setup a MoveIt interface for all groups 
        """
        super(GroupInterface, self).__init__()

        # initialize moveit_commander and a rospy node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_interface", anonymous=False)
    
        # instantiate a RobotCommander object (information about robot)
        self.robot = moveit_commander.RobotCommander()

        # instantiate MoveGroupCommander objects (interface to planning groups)
        self.groups = {}
        for name in self.robot.get_group_names():
            self.groups[name] = moveit_commander.MoveGroupCommander(name, ns=rospy.get_namespace())

        # get basic information
        print("")
        print("Available planning groups: {}".format(self.robot.get_group_names()))
        for group in self.groups:
            print("  |-- {}".format(group))
            print("  |     |-- planning frame: {}".format(self.groups[group].get_planning_frame()))
            print("  |     |-- end effector link: {}".format(self.groups[group].get_end_effector_link()))
            print("  |     |-- has end effector link: {}".format(self.groups[group].has_end_effector_link()))
            print("  |     |-- links:")
            for link_name in self.robot.get_link_names(group):
                print("  |     |     |-- {}".format(link_name))
            print("  |     |-- joints")
            joints_name = self.robot.get_active_joint_names(group)
            for joint_name in joints_name:
                print("  |     |     |-- {}".format(joint_name))
                print("  |     |     |     |-- bounds: {}".format(self.robot.get_joint(joint_name).bounds()))
        print("")

    def reach_joint_state(self, group_name, joints, degrees=False):
        """
        Reach target joint positions
        
        Parameters
        ----------
        group_name: str
            Name of the planning group
        joints: list
            Target joint positions
        degrees: bool, optional
            If true, returns the angles in degrees (radians by default).
            Note: This parameter only makes sense for revolute joints
        """
        try:
            # set joints
            if degrees:
                joints = [joint * DEGREES_TO_RADIANS for joint in joints]
            self.groups[group_name].go(joints, wait=True)

            # ensure that there is no residual movement
            self.groups[group_name].stop()
        except moveit_commander.exception.MoveItCommanderException as e:
            print("[ERROR] {}".format(str(e)))
            return geometry_msgs.msg.Pose()

    def reach_cartesian_pose(self, group_name, pose, end_effector_link=None):
        """
        Reach the target pose
        
        Parameters
        ----------
        group_name: str
            Name of the planning group
        pose: geometry_msgs.msg.Pose
            Target pose
        end_effector_link: str or None, optional
            End effector link on which to plan the trajectory.
            If None, the default end effector link is used
        """
        try:
            if end_effector_link is not None:
                self.groups[group_name].set_end_effector_link(end_effector_link)
            
            # set pose
            self.groups[group_name].set_pose_target(pose)
            self.groups[group_name].go(wait=True)
            
            # ensure that there is no residual movement
            self.groups[group_name].stop()
            
            self.groups[group_name].clear_pose_targets()
        except moveit_commander.exception.MoveItCommanderException as e:
            print("[ERROR] {}".format(str(e)))
            return geometry_msgs.msg.Pose()

    def reach_named_pose(self, group_name, name):
        """
        Reach the named pose by setting the predefined joint values

        Parameters
        ----------
        group_name: str
            Name of the planning group
        name: str
            Named pose
        """
        try:
            # set named pose
            self.groups[group_name].set_named_target(name)
            self.groups[group_name].go(wait=True)
            
            # ensure that there is no residual movement
            self.groups[group_name].stop()
            
            self.groups[group_name].clear_pose_targets()
        except moveit_commander.exception.MoveItCommanderException as e:
            print("[ERROR] {}".format(str(e)))

    def get_cartesian_pose(self, group_name):
        """
        Get the current pose of the group's end-effector 

        Parameters
        ----------
        group_name: str
            Name of the planning group

        Returns
        -------
        geometry_msgs.msg.Pose
            Current pose of the group's end-effector
        """
        try:
            return self.groups[group_name].get_current_pose().pose
        except moveit_commander.exception.MoveItCommanderException as e:
            print("[ERROR] {}".format(str(e)))
            return geometry_msgs.msg.Pose()

    def get_joint_state(self, group_name, degrees=False):
        """
        Get the current position of the joints

        Parameters
        ----------
        group_name: str
            Name of the planning group
        degrees: bool, optional
            If true, returns the angles in degrees (radians by default)
            Note: This parameter only makes sense for revolute joints

        Returns
        -------
        list
            Current position of the joints
        """
        try:
            return [joint * RADIANS_TO_DEGREES if degrees else joint for joint in self.groups[group_name].get_current_joint_values()]
        except moveit_commander.exception.MoveItCommanderException as e:
            print("[ERROR] {}".format(str(e)))
            return []
        
