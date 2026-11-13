import pybullet as p
import pybullet_data
import time
import random
import keyboard  # using module keyboard
# Initialize PyBullet
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
path="C:/Users/dexte/Documents/GitHub/Sim-Biped-Walker/URDF_files/walker_assembly/"
p.setAdditionalSearchPath(path)  # Set the path for urdf files
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Set the path for urdf files
# Load ground plane
p.loadURDF("plane.urdf",[0, 0, 0])  # Assuming there's a plane.urdf file in the pybullet data path
# Load URDF file
robot_urdf_path = "urdf/walker_assembly.urdf"  # Replace with the actual path to your URDF file
try:
    robot_id = p.loadURDF(path+robot_urdf_path, [0, 0, 0])  # Load URDF at position [0, 0, 0]

except p.error as e:
    print("error loading file",e)
    p.disconnect()
try:
    file=open(path+robot_urdf_path,"r")
    r=file.read()
    file.close()
    print("File exists")
except:
    print("cannot find file")
gravity = [0, 0, -9.8]
p.setGravity(gravity[0], gravity[1], gravity[2])
# Move joint at index 0 (assuming it's a revolute joint)
joint_index_to_move = 0


# Set the initial joint position to avoid a sudden jump
#p.resetJointState(robot_id, joint_index_to_move, targetValue=desired_joint_position)

# Simulation loop
for _ in range(5000):  # Run the simulation for 1000 steps (adjust as needed)
    p.stepSimulation()
    if keyboard.is_pressed('1'):  # if key 'q' is pressed qqqqq q
        joint_state = p.getJointState(robot_id, 0)
        joint_position = joint_state[0]
        p.setJointMotorControl2(robot_id,
                                0,
                                p.POSITION_CONTROL,
                                targetPosition=joint_position+0.1,
                                force=500)
    elif keyboard.is_pressed('2'):  # if key 'q' is pressed qqqqq q
        joint_state = p.getJointState(robot_id, 1)
        joint_position = joint_state[0]
        p.setJointMotorControl2(robot_id,
                                1,
                                p.POSITION_CONTROL,
                                targetPosition=joint_position+0.1,
                                force=500)
    elif keyboard.is_pressed('3'):  # if key 'q' is pressed qqqqq q
        joint_state = p.getJointState(robot_id, 2)
        joint_position = joint_state[0]
        p.setJointMotorControl2(robot_id,
                                2,
                                p.POSITION_CONTROL,
                                targetPosition=joint_position+0.1,
                                force=500)
    elif keyboard.is_pressed('4'):  # if key 'q' is pressed qqqqq q
        joint_state = p.getJointState(robot_id, 3)
        joint_position = joint_state[0]
        p.setJointMotorControl2(robot_id,
                                3,
                                p.POSITION_CONTROL,
                                targetPosition=joint_position+0.1,
                                force=500)
    elif keyboard.is_pressed('5'):  # if key 'q' is pressed qqqqq q
        joint_state = p.getJointState(robot_id, 0)
        joint_position = joint_state[0]
        p.setJointMotorControl2(robot_id,
                                0,
                                p.POSITION_CONTROL,
                                targetPosition=joint_position-0.1,
                                force=500)
    elif keyboard.is_pressed('6'):  # if key 'q' is pressed qqqqq q
        joint_state = p.getJointState(robot_id, 1)
        joint_position = joint_state[0]
        p.setJointMotorControl2(robot_id,
                                1,
                                p.POSITION_CONTROL,
                                targetPosition=joint_position-0.1,
                                force=500)
    elif keyboard.is_pressed('7'):  # if key 'q' is pressed qqqqq q
        joint_state = p.getJointState(robot_id, 2)
        joint_position = joint_state[0]
        p.setJointMotorControl2(robot_id,
                                2,
                                p.POSITION_CONTROL,
                                targetPosition=joint_position-0.1,
                                force=500)
    elif keyboard.is_pressed('8'):  # if key 'q' is pressed qqqqq q
        joint_state = p.getJointState(robot_id, 3)
        joint_position = joint_state[0]
        p.setJointMotorControl2(robot_id,
                                3,
                                p.POSITION_CONTROL,
                                targetPosition=joint_position-0.1,
                                force=500)                      
    time.sleep(1./240.)

p.disconnect()
print("Finished")
