import numpy as np
from copy import deepcopy

class GA:
    def __init__(self,population_size,generations,mutation_rate,sex=False):
        """
        Genetic algorithm class for running the GA
        It will need some changes to better match the environment you make
        """
        self.pop_zise=population_size
        self.rate=mutation_rate
        #self.initialize_population(controller,[_INPUT_SIZE_,[_H1_,_H2_],_OUTPUTSIZE_])
        self.sex=sex
        self.best_genos_time=[]
        self.pop=[]
        self.generations=generations
    def initialize_population(self,contr,params):
        self.contr=contr
        self.params=params
        population=[]
        for i in range(self.pop_zise):
            population.append(contr(*params))   #@seyi this is where the network sizes go
        self.pop=population

class Hillclimbers(GA):
    def evolve(self,environment,fitness,timesteps,outputs=False):
        fitness_matrix=np.zeros((self.pop_zise))
        history=[]
        for i in range(self.pop_zise): #calculate current fitness of all genotypes
            fitness__,history_,_=environment.runTrial(self.pop[i],timesteps,fitness=fitness)
            fitness_matrix[i]=fitness__
        for gen in range(self.generations): #begin actual evolution
            if outputs:
                print("Generation",gen,"best fitness:",np.max(fitness_matrix))
            for j in range(self.pop_zise):
                geno=deepcopy(self.pop[j])
                geno.mutate()
                f,history_,_=environment.runTrial(geno,timesteps,fitness=fitness)
                if f>fitness_matrix[j]:
                    self.pop[j]=deepcopy(geno)
                    fitness_matrix[j]=f
            self.best_genos_time.append(deepcopy(self.pop[np.argmax(fitness_matrix)]))
            history.append(np.max(fitness_matrix))
        return history,fitness_matrix
class Microbial_GA(GA):
    def evolve(self,environment,fitness,timesteps,outputs=False):
        history=[]
        fitness_matrix=np.zeros((self.pop_zise))
        for i in range(self.pop_zise): #calculate current fitness of all genotypes
            fitness__,history_,_=environment.runTrial(self.pop[i],timesteps,fitness=fitness)
            fitness_matrix[i]=fitness__

        for gen in range(self.generations): #begin actual evolution
            if outputs:
                print("Generation",gen,"best fitness:",np.max(fitness_matrix))
            ind1=np.random.randint(0,self.pop_zise-1)
            ind2=ind1
            while ind2==ind1: #make sure second geno is not first
                ind2=np.random.randint(0,self.pop_zise-1)
            
            if fitness_matrix[ind2]>=fitness_matrix[ind1]: #selection
                self.pop[ind1]=deepcopy(self.pop[ind2])
                if self.sex: self.pop[ind1].sex(self.pop[ind1],self.pop[ind2])
                self.pop[ind1].mutate() #mutate
                fitness__,history_,_=environment.runTrial(self.pop[ind1],timesteps,fitness=fitness)
                fitness_matrix[ind1]=fitness__
            elif fitness_matrix[ind1]>fitness_matrix[ind2]: #selection
                self.pop[ind2]=deepcopy(self.pop[ind1])
                self.pop[ind2].mutate() #mutate
                if self.sex: self.pop[ind2].sex(self.pop[ind2],self.pop[ind1])
                fitness__,history_,_=environment.runTrial(self.pop[ind2],timesteps,fitness=fitness)
                fitness_matrix[ind2]=fitness__
            
            history.append(np.max(fitness_matrix))
            self.best_genos_time.append(deepcopy(self.pop[np.argmax(fitness_matrix)]))
        return history,fitness_matrix

class Differential(GA):
    def evolve(self,environment,fitness,timesteps,outputs=False,rate=0.2):
        history=[0]
        fitness_matrix=np.zeros((self.pop_zise))
        for i in range(self.pop_zise): #calculate current fitness of all genotypes
            fitness__,history_,_=environment.runTrial(self.pop[i],timesteps,fitness=fitness)
            fitness_matrix[i]=fitness__
        for gen in range(self.generations):
            if outputs:
                print("Generation",gen,"best fitness:",np.max(history))
            for i in range(self.pop_zise): #select three distinct individuals
                indices = [idx for idx in range(self.pop_zise) if idx != i]
                r1, r2, r3 = np.random.choice(indices, 3, replace=False)
                x1, x2, x3 = self.pop[r1].geno, self.pop[r2].geno, self.pop[r3].geno
                dummy=self.contr(*self.params) #create dummy object
                mutant = x1 + 0.5 * (x2 - x3)
                dummy.geno=mutant #set the geno to the object 
                #crossover
                mutant=self.pop[i].sex(self.pop[i],dummy)
                #selection
                trial_fitness,history_,_=environment.runTrial(mutant,timesteps,fitness=fitness)
                if trial_fitness>fitness_matrix[i]:
                    fitness_matrix[i]=trial_fitness
                    self.pop[i]=deepcopy(mutant)
            self.best_genos_time.append(deepcopy(self.pop[np.argmax(fitness_matrix)]))
            history.append(np.max(fitness_matrix))
        return np.array(history), fitness_matrix

class NSGA_II(GA):
    def evolve(self,environment,fitness,timesteps,outputs=False,rate=0.2):
        history=[0]
        fitness_matrix=np.zeros((self.pop_zise))
        for i in range(self.pop_zise): #calculate current fitness of all genotypes
            fitness__,history_,_=environment.runTrial(self.pop[i],timesteps,fitness=fitness)
            fitness_matrix[i]=fitness__
        for gen in range(self.generations):
            #make offspring popoulation from P_t using selection crossover and mutation 
            #combine populations
            #perform non dominated sorrting on the combined population
            # create new population
            #assign crowding distance to individuals 
            #sort by crowding distance
            pass