import time
import math as maths
class Biped:
    def __init__(self,engine,id,floor=None):
        self.p=engine
        self.robot_id = id
        self.orientation=[-7.403932178333501e-18, -7.822274398480377e-17, 2.2034421452109355e-18]
        self.motors=[0,0,0,0]
        self.neutral=[0,0,0,0]
        self.offset=90-np.array(self.motors)
        self.start=self.getPos()
        self.start_orientation=self.getOrientation()[0:3]
        self.floor=floor
        self.positions=[]
    def reset(self):
        self.positions=[]
        for joint_index in range(len(self.neutral)): 
            self.p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=joint_index,
                controlMode=self.p.POSITION_CONTROL,
                targetPosition=maths.radians(self.neutral[joint_index])
            )
    def getPos(self):
        position, orientation = self.p.getBasePositionAndOrientation(self.robot_id)
        return position
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
    def setPositions(self,positions):
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
    def get_self_collision_count(self):
        # Get all contact points where the robot is in contact with itself
        contact_points = self.p.getContactPoints(bodyA=self.robot_id, bodyB=self.robot_id)
        return len(contact_points)
    def getOrientation(self):
        position, orientation = self.p.getBasePositionAndOrientation(self.robot_id)
        # Convert quaternion to euler angles
        orientation_euler = self.p.getEulerFromQuaternion(orientation)
        return orientation_euler
    def getContact(self):
        if type(self.floor)==type(None):
            raise TypeError("No floor ")
        contact_points = self.p.getContactPoints(bodyA=self.robot_id, bodyB=self.floor)
        return contact_points
    def hasFallen(self):
        contacts=self.getContact()
        base_touching = any(contact[3] == -1 for contact in contacts)
        if base_touching: return True
        return False
import pybullet as p
import pybullet_data
import sys
import numpy as np
import uuid
path="/its/home/drs25/Documents/GitHub/Sim-Biped-Walker/URDF_files/walker_assembly/"
if sys.platform.startswith('win'):
    path="C:/Users/dexte/Documents/GitHub/Sim-Biped-Walker/URDF_files/walker_assembly/"
robot_urdf_path = path+"urdf/walker_assembly.urdf"  # Replace with the actual path to your URDF file

sys.path.append("/its/home/drs25/Documents/GitHub/Sim-Biped-Walker/Code/")
def demo(variable,history={}):
    return 0
class environment:
    def __init__(self,show=False,record=False,filename="",friction=0.5):
        self.show=show
        if show: p.connect(p.GUI)
        else: p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)  # Ensure GUI is enabled
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)  # Hide Explorer
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)  # Hide RGB view
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)  # Hide Depth view
        self.friction=friction
        self.robot_id=None
        self.plane_id=None
        self.quad=None
        self.record=record
        self.filename=filename
        self.recording=0
        self.history={}
        if self.show:
            self.x_slider = p.addUserDebugParameter("dt", -5, 5, 0.1)
    def take_agent_snapshot(self,p, agent_id, alpha=0.1, width=640, height=480):
        # Make all objects except the agent transparent
        num_bodies = p.getNumBodies()
        for i in range(num_bodies):
            body_id = p.getBodyUniqueId(i)
            if body_id != agent_id:
                visual_shapes = p.getVisualShapeData(body_id)
                for visual in visual_shapes:
                    p.changeVisualShape(body_id, visual[1], rgbaColor=[1, 1, 1, alpha])  # Set transparency
        # Capture snapshot from the current camera view
        _, _, img_arr, _, _ = p.getCameraImage(width, height)
        # Convert the image array to a NumPy array
        return np.array(img_arr, dtype=np.uint8)
    def reset(self):
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        self.plane_id = p.loadURDF('plane.urdf')
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)  # Ensure GUI is enabled
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)  # Hide Explorer
        p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)  # Hide RGB view
        p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)  # Hide Depth view
        p.changeDynamics(self.plane_id, -1, lateralFriction=self.friction)
        p.setPhysicsEngineParameter(enableConeFriction=0)
        p.changeDynamics(self.plane_id, -1, lateralFriction=self.friction)
        initial_position = [0, 0, 5.8]  # x=1, y=2, z=0.5
        initial_orientation = p.getQuaternionFromEuler([0, 0, 0])  # No rotation (Euler angles to quaternion)
        flags = p.URDF_USE_SELF_COLLISION
        self.robot_id = p.loadURDF(robot_urdf_path, initial_position, initial_orientation,flags=flags)
        p.changeDynamics(self.robot_id, -1, lateralFriction=self.friction)
        self.quad=Biped(p,self.robot_id,self.plane_id)
        self.quad.neutral=[0,0,0,0]
        self.quad.reset()
        for i in range(500):
            p.stepSimulation()
            p.setTimeStep(1./240.)
        if self.record:
            self.video_log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, self.filename)
            self.recording=1
    def runTrial(self,agent,generations,delay=False,fitness=demo,photos=-1):
        history={}
        history['positions']=[]
        history['orientations']=[]
        history['motors']=[]
        history['accumalitive_reward']=[]
        history['self_collisions']=[]
        history['feet']=[]
        self.reset()
        a=[]
        photos_l=[]
        for i in range(generations*10):
            pos=self.step(agent,0,delay=delay)
            if photos>-1 and i%photos==0:
                print("snap")
                photos_l.append(self.take_agent_snapshot(p,self.robot_id))

            basePos, baseOrn = p.getBasePositionAndOrientation(self.robot_id) # Get model position
            history['positions'].append(basePos)
            history['orientations'].append(baseOrn[0:3])
            history['motors'].append(pos)
            history['accumalitive_reward'].append(fitness(self.quad,history=history))
            history['self_collisions'].append(self.quad.get_self_collision_count())
            #history['feet'].append(self.quad.getFeet())
            p.resetDebugVisualizerCamera( cameraDistance=0.3, cameraYaw=75, cameraPitch=-20, cameraTargetPosition=basePos) # fix camera onto model
            if self.quad.hasFallen():
                break
            if self.quad.hasFallen():
                break
            a.append(pos)
        history['positions']=np.array(history['positions'])
        history['orientations']=np.array(history['orientations'])
        history['motors']=np.array(history['motors'])
        history['accumalitive_reward']=np.array(history['accumalitive_reward'])
        history['self_collisions']=np.array(history['self_collisions'])
        history['feet']=np.array(history['feet'])
        filename = str(uuid.uuid4())
        #np.save("/its/home/drs25/Documents/GitHub/Quadruped/Code/data_collect_proj/trials_all/"+str(filename),history)
        return fitness(self.quad,history=history),history,photos_l
    def step(self,agent,action,delay=False,gen=0):
        if self.show:
            agent.dt=p.readUserDebugParameter(self.x_slider)
        motor_positions=agent.get_positions(np.array(self.quad.motors))
        self.quad.setPositions(motor_positions)
        for k in range(10): #update simulation
            p.stepSimulation()
            if delay: time.sleep(1./240.)
            else: p.setTimeStep(1./240.)
            basePos, baseOrn = p.getBasePositionAndOrientation(self.robot_id) # Get model position
            p.resetDebugVisualizerCamera( cameraDistance=0.3, cameraYaw=75, cameraPitch=-20, cameraTargetPosition=basePos) # fix camera onto model
            if self.quad.hasFallen():
                
                break
        return motor_positions
    def stop(self):
        if self.recording and self.record:
            p.stopStateLogging(self.video_log_id)
            self.recording=0
    def close(self):
        p.disconnect()

if __name__ == "__main__":
    e=environment(1)
    e.reset()
    time.sleep(5)
    e.close()