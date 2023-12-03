#!/usr/bin/env python
import sys
import copy
from time import sleep
import rospy
import moveit_commander
import numpy as np

from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

from copy import deepcopy

from gazebo_msgs.srv import GetModelState
from fake_vision.srv import GetModelList, GetModelListResponse, GetModelListRequest

import tf_conversions
import PyKDL


CLOSE_GRIPPER=-1
OPEN_GRIPPER=1
NEUTRAL_GRIPPER=0

class PickAndPlace:
    def __init__(self,move_group_name):
        
        rospy.wait_for_service('/gazebo/get_model_state')
        self.get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        rospy.wait_for_service('get_model_list')
        self.get_target_list = rospy.ServiceProxy('get_model_list', GetModelList)


        #just to safely wait until moveit_config has started
        rospy.wait_for_service('/apply_planning_scene')
        # Instantiate a RobotCommander object
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm = moveit_commander.MoveGroupCommander(move_group_name)
        self.add_collision_objects()



        self.gripper_pub = rospy.Publisher("/gripper_joint_position/command",Float64, queue_size=1)

        #release the grip and return to the home configuration at takeoff.
        self.command_gripper(OPEN_GRIPPER)
        self.home_target_joint_values = self.arm.get_named_target_values("home_configuration")
        self.home()

        #define the desired orientation for place subtask
        identity_rotation =PyKDL.Rotation()
        desired_orientation=PyKDL.Rotation(-identity_rotation.UnitZ(), 
                              identity_rotation.UnitY(), 
                              identity_rotation.UnitX())
        self.desired_pose= tf_conversions.toMsg(PyKDL.Frame(desired_orientation,PyKDL.Vector()))

        #evaluate the transformation matrix to get targets' pose with respect to robot base frame
        robot_pose = self.get_robot_pose()
        self.robot_matrix = self.create_transformation_matrix(robot_pose)


        rospy.on_shutdown(self.cleanup)

    def call_model_list_service(self,target_str):
        request = GetModelListRequest(target_str)
        response = self.get_target_list(request)
        return response.model_names


    def home(self):
        self.arm.set_joint_value_target(self.home_target_joint_values)
        self.arm.plan()
        self.arm.go(wait=True)

    def command_gripper(self,value):
        cnt=0

        rate = rospy.Rate(20)
        while(cnt<10):
            self.gripper_pub.publish(value)
            cnt+=1
            rate.sleep()
        


        
        
    def create_collision_object(self,id, dimensions, pose):
        object = CollisionObject()
        object.id = id
        object.header.frame_id = "base_link"

        solid = SolidPrimitive()
        solid.type = solid.BOX
        solid.dimensions = dimensions
        object.primitives = [solid]

        object_pose = Pose()
        object_pose.position.x = pose[0]
        object_pose.position.y = pose[1]
        object_pose.position.z = pose[2]

        object.primitive_poses = [object_pose]
        object.operation = object.ADD
        return object

    
    def add_collision_objects(self):

        table_1 = self.create_collision_object(id='cafe_table_1',
                                        dimensions=[0.91, 0.91, 0.775],
                                        pose=[0, -0.6, -0.3125])
        table_2 = self.create_collision_object(id='cafe_table_2',
                                        dimensions=[0.91, 0.91, 0.775],
                                        pose=[0, 0.6, -0.3125])

            
        self.scene.add_object(table_1)
        self.scene.add_object(table_2)
        
    def create_transformation_matrix(self,pose):
        
        # Crea una matrice di trasformazione usando TF
        transformation_matrix = tf_conversions.fromMsg(pose)

        return transformation_matrix
    
    def apply_transformation(self,transformation_frame, pose):
        # Convert geometry_msgs/Pose to PyKDL Frame
        pose_frame = tf_conversions.fromMsg(pose)

        # Apply transformation
        transformed_frame = transformation_frame.Inverse() * pose_frame

        # Convert back to geometry_msgs/Pose
        transformed_pose = tf_conversions.toMsg(transformed_frame)

        return transformed_pose

    def get_robot_pose(self):
        try:
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp = get_model_state('robot', '') 
            return resp.pose
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
    
    def get_block_pose(self, block_name):
        # Call the service to get the model state
        block_state = self.get_model_state(block_name, "world")

        return self.apply_transformation(self.robot_matrix,block_state.pose)
    
    def compute_pick_cartesian_path(self,start_pose,final_pose):
        waypoints = []

        # Set the first waypoint to be the starting pose
        # Append the pose to the waypoints list
        midway_pose = deepcopy(final_pose)
        midway_pose.position.x = (final_pose.position.x + start_pose.position.x)/2 
        midway_pose.position.y = (final_pose.position.y +start_pose.position.y)/2 
        midway_pose.position.z = (final_pose.position.z +start_pose.position.z)/2
        waypoints.append(midway_pose)
        approach_pose = deepcopy(final_pose)
        approach_pose.position.z+=0.02
        waypoints.append(approach_pose)
        waypoints.append(final_pose)
        self.arm.set_start_state_to_current_state()
        (plan, fraction) = self.arm.compute_cartesian_path(
            waypoints, 0.01, 0.0, True)
        return plan, fraction

    def define_grasp(self,model_name):


        block_pose = self.get_block_pose(model_name)
        # Get the rotation part of the frame
        block_frame=tf_conversions.fromMsg(block_pose)



        rotation = PyKDL.Rotation(-block_frame.M.UnitZ(), 
                              block_frame.M.UnitY(), 
                              block_frame.M.UnitX())
        new_block_frame=PyKDL.Frame(rotation, block_frame.p)
        block_pose=tf_conversions.toMsg(new_block_frame)
        return block_pose

    def pick(self,block_pose):


        # Get the current pose so we can add it as a waypoint
        start_pose = self.arm.get_current_pose().pose

        # Initialize the waypoints list
        (cartesian_plan, fraction) = self.compute_pick_cartesian_path(start_pose,block_pose)
        if fraction == 1.0:
            print("plan is feasible")
            self.arm.execute(cartesian_plan, wait=True)
            self.command_gripper(CLOSE_GRIPPER)
        else:
            raise Exception("[PICK TASK] object unreachable")

    def place(self,final_pose):
        final_pose.orientation=self.desired_pose.orientation

        start_pose = self.arm.get_current_pose().pose
        (cartesian_plan, fraction) = self.compute_pick_cartesian_path(start_pose,final_pose)
        if fraction == 1.0:
            print("plan is feasible")
            self.arm.execute(cartesian_plan, wait=True)
            self.command_gripper(OPEN_GRIPPER)
        else:

            print("planning error %d"+ str(fraction))
            raise Exception("[PLACE TASK] desired place position is unreachable")

    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
    
    def loop_routine(self,target):
        rate=rospy.Rate(1)
        y_offset = 0.3
        x_offset = -0.2
        x_step = 0.2
        y_step = y_offset
        max_y = 0.7
        max_x = 0.5
        ith_object=0
        while not rospy.is_shutdown():
            model_names = self.call_model_list_service(target)
            if model_names: #there is at least model_names[0]
                if ith_object >= len(model_names): #there are no other objects, continue trying
                    ith_object = 0
                des_pose=self.define_grasp(model_names[ith_object])
                try:
                    self.pick(des_pose)
                    self.home()
                    
                    # Define the target pose for placing the block
                    try:
                        place_pose = Pose()
                        place_pose.position.x = x_offset
                        place_pose.position.y = y_step
                        place_pose.position.z = 0.1

                        self.place(place_pose)
                        self.home()
                    except Exception as e:
                        rospy.loginfo(f"Operation failed: {e}")
                        #we will put it back and try again later
                        place_pose=des_pose
                        self.place(place_pose)
                        self.home()


                    # Aggiorna y_offset per il prossimo ciclo
                    y_step += 0.1

                    # Controlla se y_offset ha superato il valore massimo
                    if y_step > max_y:
                        y_step = y_offset  # Reset y_offset
                        x_offset += x_step  # Increm x_offset
                        if(x_offset>max_x):
                            rospy.signal_shutdown("table 2 has no empty spaces")

                except Exception as e:
                    ith_object+=1
                    rospy.loginfo(f"Operation failed: {e}")

                    
            else:
                rate.sleep()



if __name__ == '__main__':

    rospy.init_node('pick_and_place', anonymous=False)

    try:
        moveit_commander.roscpp_initialize(sys.argv)
        # Create an instance of the PickAndPlace class
        pick_and_place = PickAndPlace('manipulator')
        pick_and_place.loop_routine("block")
        
    except rospy.ROSInterruptException:
        print("Shutting down PickAndPlace node.")
