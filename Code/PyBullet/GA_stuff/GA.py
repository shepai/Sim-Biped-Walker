if __name__=="__main__":
    import sys
    sys.path.insert(1,"/its/home/drs25/Documents/GitHub/Sim-Biped-Walker/Code/")
    sys.path.insert(1,"C:/Users/dexte/Documents/GitHub/Sim-Biped-Walker/Code")
datapath="/its/home/drs25/Documents/GitHub/Sim-Biped-Walker/"
#datapath="C:/Users/dexte/Documents/GitHub/Sim-Biped-Walker/"

from PyBullet.Biped_controller.pattern import sinBot, CTRNNBiped, simple
import time
from copy import deepcopy
import pickle
from PyBullet.Biped_controller import environment
import numpy as np
import os
from genetic_algorithms import *

def fitness_(robot,history={}): 
    fitness=0
    #look at behaviour over time
    if len(history.get('motors',[]))>0:
        alpha=100
        beta=0.05
        gamma=0.5
        #F1
        """distances=np.array(history['positions'])-np.array([robot.start])#euclidean_distance(np.array(history['positions']),np.array([robot.start]))
        distancesX=distances[-1][0]
        distancesY=distances[-1][1]
        distancesZ=distances[-1][2]
        fitness+=distancesX - (distancesY+distancesZ)/10 #np.sum(distances)"""
        #F2
        """positions = np.array(history['positions'])  # Shape (T, 3)
        X, Y, Z = positions[:, 0], positions[:, 1], positions[:, 2]
        forward_movement = np.abs(X[-1] - X[0])
        deviation_penalty = np.sum(np.sqrt(Y**2 + Z**2))
        velocities = np.diff(positions, axis=0)
        smoothness_penalty = np.sum(np.linalg.norm(np.diff(velocities, axis=0), axis=1))
        #print(forward_movement, deviation_penalty, smoothness_penalty)
        fitness = (alpha * forward_movement) - (beta * deviation_penalty) - (gamma * smoothness_penalty)"""
        #F3
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
    #if fitness<0: fitness=0
    return fitness
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
def RUN(dt=0.1,sho=0,trial=0,generations=300,fit=fitness_,fric=0.5,fitnum="Def"):
    env=environment(sho,friction=fric)
    #agent goes in population generation
    #initial
    population_size=50
    #g=geno_gen(6,population_size)
    population=[CTRNNBiped(dt=dt,imu=0) for _ in range(population_size)]#np.random.choice([50, 20, 0,0,0,0,-20],(150,15,12)) #12 motors, 15 steps
        
    fitnesses=np.zeros((population_size,))
    
    t_start=time.time()
    #get fitnesses
    for i in range(len(fitnesses)):
        fitnesses[i],_,_2=env.runTrial(population[i],100,delay=0,fitness=fit)
        #print(i,"/",len(fitnesses), fitnesses[i])
    history=np.zeros((generations,))
    for gen in range(generations):
        
        if gen%50==0:
            print("Generation ",gen+1,"Best fitness",np.max(fitnesses))
        ind1=np.random.randint(0,len(fitnesses)-1)
        ind2=np.random.randint(0,len(fitnesses)-1)
        if fitnesses[ind1]>fitnesses[ind2]: #selection
            geno=deepcopy(population[ind1])
            mutated=deepcopy(geno)
            mutated.mutate()
            fitnesses[ind2],motors,_=env.runTrial(mutated,100,delay=False,fitness=fit)
            population[ind2]=deepcopy(mutated)
        elif fitnesses[ind2]>fitnesses[ind1]:
            geno=deepcopy(population[ind2])
            mutated=deepcopy(geno)
            mutated.mutate()
            fitnesses[ind1],motors,_=env.runTrial(mutated,100,delay=False,fitness=fit)
            population[ind1]=deepcopy(mutated)
        history[gen]=np.max(fitnesses)
    #play the trials on reapeat
        if gen%10==0:
            with open(datapath+'/models/F2_Friction//genotypes_dt'+str(dt)+"_"+str(trial)+str(fric)+"_"+fitnum+'.pkl', 'wb') as f:
                pickle.dump(population, f)
            np.save(datapath+'/models//F2_Friction/fitnesses_dt'+str(dt)+"_"+str(trial)+str(fric)+"_"+fitnum,fitnesses)
            np.save(datapath+'/models//F2_Friction/history_dt'+str(dt)+"_"+str(trial)+str(fric)+"_"+fitnum,history)


    env.runTrial(population[np.where(fitnesses==np.max(fitnesses))[0][0]],150,fitness=fit)
    print("top fitness:",np.max(fitnesses))
    p.disconnect()


    t_passed=time.time()-t_start
    print("********************************\n\n\n\nTIME IT TOOK:",t_passed/(60*60),"Hours")

if __name__=="__main__":
    DT=0.1
    start=time.time()
    """for _ in range(20):
        os.system('cls' if os.name == 'nt' else 'clear')
        print("PROGRESS:",_,"/ 20")
        RUN(dt=DT,sho=0,trial="_6_neurons_"+str(_)+"_F1",fit=F1)
        RUN(dt=DT,sho=0,trial="_6_neurons_"+str(_)+"_F2",fit=F2)
        RUN(dt=DT,sho=0,trial="_6_neurons_"+str(_)+"_F3",fit=F3)"""

    """for trial in range(3,4):
        for i in np.arange(0.05,1.5,0.05):
            RUN(dt=i,sho=0,trial=trial)"""
    c=0
    for i in range(0,20):
        for j in np.arange(0.05,1,0.05):
            os.system('cls' if os.name == 'nt' else 'clear')
            calc=len(range(0,40))*len( np.arange(0.05,1,0.05))
            print(i,j,c/calc *100,"%")
            RUN(dt=0.1,sho=0,trial="trial_"+str(i)+"_friction",fit=F2,fric=j)
            c+=1
    """for i in range(20):
        os.system('cls' if os.name == 'nt' else 'clear')
        print(i,"/",i,(i/20 *100),"%","\nTIME PASSED:",round((time.time()-start)/60,2),"minutes")
        RUN(dt=0.1,sho=0,trial="trial_"+str(i)+"_friction",fit=F3,fric=0.5,fitnum="F3")
        RUN(dt=0.1,sho=0,trial="trial_"+str(i)+"_friction",fit=F1,fric=0.5,fitnum="F1")
        RUN(dt=0.1,sho=0,trial="trial_"+str(i)+"_friction",fit=F2,fric=0.5,fitnum="F2")"""
    """for i in range(0,20):
        RUN(dt=0.1,sho=0,trial="6_neurons_"+str(i)+"_rocky",fit=F3,fric=j)"""
