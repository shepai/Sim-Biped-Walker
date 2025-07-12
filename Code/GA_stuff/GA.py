if __name__=="__main__":
    import sys
    sys.path.insert(1,"/its/home/drs25/Documents/GitHub/Sim-Biped-Walker/Code/")
    sys.path.insert(1,"C:/Users/dexte/Documents/GitHub/Sim-Biped-Walker/Code")
datapath="/its/home/drs25/Documents/GitHub/Sim-Biped-Walker/"
datapath="C:/Users/dexte/Documents/GitHub/Sim-Biped-Walker/"

from Biped_controller.pattern import sinBot
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
    #ga.delay=1
    history,fitnesses=ga.evolve(env, fit, 20,outputs=1)
    t_start=time.time()
    #get fitnesses
    with open(datapath+'/models/genotypes_dt'+str(dt)+"_"+str(trial)+str(fric)+'.pkl', 'wb') as f:
        pickle.dump(ga.pop, f)
    np.save(datapath+'/models/fitnesses_dt'+str(dt)+"_"+str(trial)+str(fric),fitnesses)
    np.save(datapath+'/models/history_dt'+str(dt)+"_"+str(trial)+str(fric),history)

    env.runTrial(ga.pop[np.where(fitnesses==np.max(fitnesses))[0][0]],150,fitness=fit)
    print("top fitness:",np.max(fitnesses))
    env.close()

    t_passed=time.time()-t_start
    print("********************************\n\n\n\nTIME IT TOOK:",t_passed/(60*60),"Hours")
if __name__=="__main__":
    for i in range(20):
        #RUN_microbial(dt=0.1,sho=0,generations=300,trial="LongMicrobial_"+str(i)+"_friction",fit=F3,fric=0.5)
        RUN_hillclimber(dt=0.1,sho=0,generations=200,trial="Hillclimber_F1_"+str(i)+"_friction",fit=F1,fric=0.5)
        #RUN_hillclimber(dt=0.1,sho=0,generations=300,trial="Hillclimber_F2_"+str(i)+"_friction",fit=F2,fric=0.5)
        #RUN_hillclimber(dt=0.1,sho=0,generations=300,trial="Hillclimber_F3_"+str(i)+"_friction",fit=F3,fric=0.5)