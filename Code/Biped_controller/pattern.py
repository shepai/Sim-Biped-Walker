import numpy as np

class sinBot:
    def __init__(self, num_legs=2, num_motors_per_leg=2, dt=0.1,imu=False):
        self.num_legs = num_legs
        self.num_motors = num_legs * num_motors_per_leg  # 12 motors total
        self.dt = dt  # Time step for integration
        self.leg_geno=np.random.uniform(-1,1,(3))
        self.hip_geno=np.random.uniform(-1,1,(3))
        self.phase=np.random.uniform(-1,1,(4))
        self.geno=np.concatenate([self.hip_geno,self.leg_geno,self.phase])
        self.t=0
    def get_positions(self,inputs,motors=None):
        degrees=np.degrees(self.step(imu_feedback=inputs, velocity_feedback=0))*10
        degrees=np.clip(degrees,0,40)
        #degrees[[2,5,8,11]]=degrees[[1,4,7,10]]
        #degrees[3:9]=-degrees[3:9] #try running this 
        #print(degrees)
        return degrees
    def step(self, imu_feedback, velocity_feedback=0):
        motor_positions=[]
        for i in range(self.num_legs):
            H=self.hip_geno[0]*np.sin(self.hip_geno[1]*self.t - self.phase[i]) + self.hip_geno[2]
            L=self.leg_geno[0]*np.sin(self.leg_geno[1]*self.t - self.phase[i]) + self.leg_geno[2]
            motor_positions.append([H,L])
        self.t+=self.dt
        motor_positions=np.array(motor_positions).flatten()
        return np.array([motor_positions[1],motor_positions[0],motor_positions[3],motor_positions[2]]).flatten()
    def mutate(self,rate=0.2):
        probailities=np.random.random(self.geno.shape)
        self.geno[np.where(probailities<rate)]+=np.random.normal(0,1,self.geno[np.where(probailities<rate)].shape)
        self.set_genotype(self.geno)
    def sex(self,geno1,geno2,prob_winning=0.6):
        probabilities=np.random.random(len(self.geno))
        geno2.geno[np.where(probabilities<prob_winning)]=geno1.geno[np.where(probabilities<prob_winning)]
        geno2.set_genotype(geno2.geno)
        return geno2
    def set_genotype(self, values):
        self.t=0
        self.hip_geno=values[0:4]
        self.leg_geno=values[4:8]
        self.phase=values[8:]
        self.hip_geno=np.clip(self.hip_geno,-2,2)
        self.leg_geno=np.clip(self.leg_geno,-2,2)
        self.phase=np.clip(self.hip_geno,-1,1)

class CTRNNBiped:
    def __init__(self, num_legs=2, num_motors_per_leg=2, dt=0.1,imu=False):
        self.num_legs = num_legs
        self.num_motors = num_legs * num_motors_per_leg  # 12 motors total
        self.dt = dt  # Time step for integration
        self.num_neurons=6
        #initialize CTRNN parameters
        self.tau = np.ones(self.num_neurons) * 0.5  # Time constants (modifiable via evolution)
        self.weights = np.random.normal(-1, 1, (self.num_neurons, self.num_neurons)) # Synaptic weights .normal(0,2,(self.num_neurons, self.num_neurons))#
        self.biases = np.zeros(self.num_neurons)  # Bias terms
        self.activations = np.zeros(self.num_neurons)  # Neuron activations
        self.outputs = np.zeros(self.num_neurons)  # Motor output (joint angles)

        #frequency and phase offsets for oscillatory behavior
        self.omega = np.random.normal(0.8, 1.2, (self.num_motors,))  # Oscillation frequencies
        self.phases = np.linspace(0, 2 * np.pi, self.num_motors, endpoint=False)  # Initial phase
        self.geno=np.concatenate((self.weights.flatten(),self.biases.flatten(),self.tau.flatten(),self.omega.flatten()))

        #IMU Feedback Gains (Proportional control for stability)
        self.Kp_imu = 0.5  # Adjusts hip based on tilt
        self.Kp_vel = 0.3  # Adjusts knee based on forward velocity
        self.height=1
        self.imu=imu
    def sigmoid(self, x):
        x = np.clip(x, -500, 500)
        return 1 / (1 + np.exp(-x))
    def get_positions(self,inputs,motors=None):
        degrees=np.degrees(self.step())/1.5
        degrees=np.clip(degrees,0,180)
        #degrees[[2,5,8,11]]=degrees[[1,4,7,10]]
        #degrees[3:9]=-degrees[3:9] #try running this 
        return degrees
    def step(self,):
        """Update the CTRNN for one timestep."""
        #imu_feedback = np.array(imu_feedback).flatten()
        # Create fixed input weights if not already done (move this to __init__ ideally)
        input_weights = np.random.uniform(-1, 1, (self.num_neurons, 3)) if self.imu else np.zeros((self.num_neurons, 3))

        # Project IMU feedback into neuron space
        sensory_drive = 0#input_weights @ imu_feedback  # shape: (num_neurons,)

        # Compute neural activations (discrete update of CTRNN)
        net_input = np.dot(self.weights,self.outputs) + self.biases + sensory_drive
        net_input = np.clip(net_input, -500, 500)
        self.activations += self.dt / self.tau * (-self.activations + net_input)
        #arr = np.array([1.0, float('nan'), float('inf'), -float('inf'), 2.0])
        #nan_mask = arr != arr
        #self.activations = arr * (~nan_mask)
        self.outputs = self.sigmoid(self.activations)

        # Add oscillatory gait modulation
        self.phases += self.dt * self.omega
        oscillation = np.sin(self.phases)

        # Compute motor commands (combine CTRNn output and oscillation)
        motor_commands = np.concatenate((self.outputs[0:2],self.outputs[0:2])) + 0.5 * oscillation
        return np.clip(motor_commands, 0, 1)*self.height  # Return motor positions (normalized)
    def set_genotype(self, values):
        """Set CTRNN parameters from an evolutionary genotype."""
        num_weights = len(self.weights.flatten())
        num_biases = len(self.biases.flatten())
        num_tau = len(self.tau.flatten())
        num_omega = len(self.omega.flatten())
        #assign genotype values to weights, biases, and time constants
        self.weights = values[0:num_weights].reshape(self.weights.shape)
        self.biases = values[num_weights:num_weights + num_biases]
        self.tau = values[num_weights + num_biases:num_weights + num_biases + num_tau].reshape(self.tau.shape)
        self.omega = values[num_weights + num_biases + num_tau: num_weights + num_biases + num_tau + num_omega].reshape(self.omega.shape)
        #apply value constraints
        self.biases = np.clip(self.biases, -16, 16)  # Cap bias values
        self.tau = np.maximum(self.tau, self.dt)  # Ensure time constants are above dt
        self.weights = np.clip(self.weights, -4, 4)  # Cap weight values
        self.omega = np.clip(self.omega, -1, 1)  # Cap weight values
        self.geno=np.concatenate([self.weights.flatten(),self.biases.flatten(),self.tau.flatten(),self.omega.flatten()])
    def mutate(self,rate=0.2):
        probailities=np.random.random(self.geno.shape)
        self.geno[np.where(probailities<rate)]+=np.random.normal(0,4,self.geno[np.where(probailities<rate)].shape)
        self.set_genotype(self.geno)
    def sex(self,geno1,geno2,prob_winning=0.6):
        probabilities=np.random.random(len(self.geno))
        geno2.geno[np.where(probabilities<prob_winning)]=geno1.geno[np.where(probabilities<prob_winning)]
        geno2.set_genotype(geno2.geno)
        return geno2