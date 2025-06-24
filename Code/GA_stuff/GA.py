if __name__=="__main__":
    import sys
    sys.path.insert(1,"/its/home/drs25/Documents/GitHub/Sim-Biped-Walker/Code/")
datapath="/its/home/drs25/Documents/GitHub/Sim-Biped-Walker/"
#datapath="C:/Users/dexte/Documents/GitHub/Quadruped/"
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
        degrees=np.degrees(self.step(imu_feedback=inputs, velocity_feedback=0))/1.5
        degrees=np.clip(degrees,0,180)
        #degrees[[2,5,8,11]]=degrees[[1,4,7,10]]
        #degrees[3:9]=-degrees[3:9] #try running this 
        return degrees
    def step(self, imu_feedback, velocity_feedback=0):
        motor_positions=[]
        for i in range(self.num_legs):
            H=self.hip_geno[0]*np.sin(self.hip_geno[1]*self.t - self.phase[i]) + self.hip_geno[2]
            L=self.leg_geno[0]*np.sin(self.leg_geno[1]*self.t - self.phase[i]) + self.leg_geno[2]
            motor_positions.append([H,L])
        self.t+=self.dt
        return np.array(motor_positions).flatten()
    def mutate(self,rate=0.2):
        probailities=np.random.random(self.geno.shape)
        self.geno[np.where(probailities<rate)]+=np.random.normal(0,4,self.geno[np.where(probailities<rate)].shape)
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
        self.hip_geno=np.clip(self.hip_geno,-4,4)
        self.leg_geno=np.clip(self.leg_geno,-4,4)
        self.phase=np.clip(self.hip_geno,-1,1)

import time
from copy import deepcopy
import pickle
from Biped_controller import environment
import numpy as np
import os
from genetic_algorithms import *

    
    
def F1(robot,history={}): 
    fitness=0
    #look at behaviour over time
    if len(history.get('motors',[]))>0:
        distances=np.array(history['positions'])-np.array([robot.start])#euclidean_distance(np.array(history['positions']),np.array([robot.start]))
        distancesX=distances[-1][0]
        distancesY=distances[-1][1]
        distancesZ=distances[-1][2]
        fitness+=distancesX - (distancesY+distancesZ)/10 #np.sum(distances)
    if robot.hasFallen(): fitness=-1000
    if type(fitness)!=type(0): 
        try:
            if type(fitness)==type([]): fitness=float(fitness[0])
            else:fitness=float(fitness)
        except:
            print("shit",fitness,np.array(history['motors']).shape,np.array(history['positions']).shape,np.array(history['orientations']).shape)
            fitness=-1000
    return fitness

def F2(robot,history={}): 
    fitness=0
    #look at behaviour over time
    if len(history.get('motors',[]))>0:
        alpha=10
        beta=0.05
        gamma=0.5
        positions = np.array(history['positions'])  # Shape (T, 3)
        X, Y, Z = positions[:, 0], positions[:, 1], positions[:, 2]
        forward_movement = np.abs(X[-1] - X[0])
        deviation_penalty = np.sum(np.sqrt(Y**2 + Z**2))
        velocities = np.diff(positions, axis=0)
        smoothness_penalty = np.sum(np.linalg.norm(np.diff(velocities, axis=0), axis=1))
        #print(forward_movement, deviation_penalty, smoothness_penalty)
        fitness = (alpha * forward_movement) - (beta * deviation_penalty) - (gamma * smoothness_penalty)
    if robot.hasFallen(): fitness=-1000
    if type(fitness)!=type(0): 
        try:
            if type(fitness)==type([]): fitness=float(fitness[0])
            else:fitness=float(fitness)
        except:
            print("shit",fitness,np.array(history['motors']).shape,np.array(history['positions']).shape,np.array(history['orientations']).shape)
            fitness=-1000
    return fitness

def F3(robot,history={}): 
    fitness=0
    #look at behaviour over time
    if len(history.get('motors',[]))>0:
        positions = np.array(history['positions'])
        Y, X, Z = positions[:, 0], positions[:, 1], positions[:, 2]
        orientations=history['orientations']
        magnitude=np.sqrt(np.sum(np.square(orientations),axis=1))
        forward_movement = np.abs(X[-1] - X[0])
        fitness=np.abs(np.sum(np.diff(Y))) - np.sum(np.abs(magnitude))/10
    if robot.hasFallen(): fitness=-1000
    if type(fitness)!=type(0): 
        try:
            if type(fitness)==type([]): fitness=float(fitness[0])
            else:fitness=float(fitness)
        except:
            print("shit",fitness,np.array(history['motors']).shape,np.array(history['positions']).shape,np.array(history['orientations']).shape)
            fitness=-1000
    return fitness
def euclidean_distance(point1, point2):
    return np.sqrt(np.sum((np.array(point1) - np.array(point2)) ** 2,axis=1))

def RUN_hillclimber(dt=0.1,sho=0,trial=0,generations=300,fit=F3,fric=0.5):
    env=environment(sho,generations,friction=fric)
    env.dt=dt
    population_size=50
    mutation_rate=0.2
    ga=Hillclimbers(population_size, generations, 0.2)
    ga.initialize_population(sinBot, [2,2,0.1,0])
    history,fitnesses=ga.evolve(env, fit, 20,outputs=1)
    t_start=time.time()
    #get fitnesses
    with open(datapath+'/models/genotypes_dt'+str(dt)+"_"+str(trial)+str(fric)+'.pkl', 'wb') as f:
        pickle.dump(ga.pop, f)
    np.save(datapath+'/models/fitnesses_dt'+str(dt)+"_"+str(trial)+str(fric),fitnesses)
    np.save(datapath+'/models/history_dt'+str(dt)+"_"+str(trial)+str(fric),history)

    env.runTrial(ga.pop[np.where(fitnesses==np.max(fitnesses))[0][0]],150,fitness=fit)
    print("top fitness:",np.max(fitnesses))
    p.disconnect()

    t_passed=time.time()-t_start
    print("********************************\n\n\n\nTIME IT TOOK:",t_passed/(60*60),"Hours")
if __name__=="__main__":
    for i in range(20):
        #RUN_microbial(dt=0.1,sho=0,generations=300,trial="LongMicrobial_"+str(i)+"_friction",fit=F3,fric=0.5)
        RUN_hillclimber(dt=0.1,sho=0,generations=300,trial="Hillclimber_F1_"+str(i)+"_friction",fit=F1,fric=0.5)
        RUN_hillclimber(dt=0.1,sho=0,generations=300,trial="Hillclimber_F2_"+str(i)+"_friction",fit=F2,fric=0.5)
        RUN_hillclimber(dt=0.1,sho=0,generations=300,trial="Hillclimber_F3_"+str(i)+"_friction",fit=F3,fric=0.5)