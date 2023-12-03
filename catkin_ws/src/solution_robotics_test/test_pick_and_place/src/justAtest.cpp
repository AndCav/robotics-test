// apt-get install psmisc ti serve per il killall




def define_grasp(self,model_name):


        block_pose = self.get_block_pose(model_name)
        # Get the rotation part of the frame
        block_frame=tf_conversions.fromMsg(block_pose)

        # Calculate roll, pitch, and yaw from the rotation
        # (roll, pitch, block_yaw) = block_frame.M.GetRPY()

        rotation = PyKDL.Rotation(-block_frame.M.UnitZ(), 
                              block_frame.M.UnitY(), 
                              block_frame.M.UnitX())
        new_block_frame=PyKDL.Frame(rotation, block_frame.p)
        block_pose=tf_conversions.toMsg(new_block_frame)
        return block_pose

    def pick(self,block_pose):

        # print("Roll (rad): ", roll )
        # print("Pitch (rad):", pitch)
        # print("Yaw (rad):  ", yaw )
        # Get the current pose so we can add it as a waypoint
        start_pose = self.arm.get_current_pose().pose

        # Initialize the waypoints list
        (cartesian_plan, fraction) = self.compute_pick_cartesian_path(start_pose,block_pose)
        if fraction == 1.0:
            print("plan is feasible")
            self.arm.execute(cartesian_plan, wait=True)
            self.command_gripper(CLOSE_GRIPPER)
        else:
            print("object unreachable")
