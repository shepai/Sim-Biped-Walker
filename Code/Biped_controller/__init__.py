import time
class Biped:
    def __init__(self,engine,filepath,position=[0, 0, 1]):
        self.p=engine
        self.robot_id = self.p.loadURDF(filepath, position)  # Load URDF at position [0, 0, 0]
        for _ in range(100):  # Run the simulation for 1000 steps (adjust as needed)
            self.p.stepSimulation()
            time.sleep(1./240.)
        self.orientation=[-7.403932178333501e-18, -7.822274398480377e-17, 2.2034421452109355e-18]
    def positions(self):
        num_joints = self.p.getNumJoints(self.robot_id)

        angular_positions = []
        for i in range(num_joints):
            joint_info = self.p.getJointInfo(self.robot_id, i)
            joint_type = joint_info[2]
            if joint_type == self.p.JOINT_REVOLUTE:
                # For revolute joints, retrieve the angular position
                joint_state = self.p.getJointState(self.robot_id, i)
                angular_position = joint_state[0]
                angular_positions.append(angular_position)
        
        return angular_positions
    def set_positions(self,positions):
        num_joints = self.p.getNumJoints(self.robot_id)

        if len(positions) != num_joints:
            print("Error: Number of angular positions does not match the number of joints.")
            return

        for i in range(num_joints):
            joint_info = self.p.getJointInfo(self.robot_id, i)
            joint_type = joint_info[2]
            if joint_type == self.p.JOINT_REVOLUTE:
                # For revolute joints, set the angular position
                self.p.setJointMotorControl2(self.robot_id, i, self.p.POSITION_CONTROL, targetPosition=positions[i])
    def get_orientation(self):
        position, orientation = self.p.getBasePositionAndOrientation(self.robot_id)

        # Convert quaternion to euler angles
        orientation_euler = self.p.getEulerFromQuaternion(orientation)
        
        return orientation_euler
    def get_position(self):
        position, orientation = self.p.getBasePositionAndOrientation(self.robot_id)
        return position
        