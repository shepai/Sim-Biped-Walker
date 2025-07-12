import numpy as np
import matplotlib.pyplot as plt
if __name__=="__main__":
    import sys
    sys.path.insert(1,"/its/home/drs25/Documents/GitHub/Sim-Biped-Walker/Code/")
    sys.path.insert(1,"C:/Users/dexte/Documents/GitHub/Sim-Biped-Walker/Code")
datapath="/its/home/drs25/Documents/GitHub/Sim-Biped-Walker/"
datapath="C:/Users/dexte/Documents/GitHub/Sim-Biped-Walker/"

from Biped_controller.pattern import sinBot
from Biped_controller import environment, get_total_mass
import numpy as np
import pickle
from genetic_algorithms import *



file="models/genotypes_dt0.1_Hillclimber_F1_0_friction0.5.pkl"

with open(datapath + file, "rb") as f:
    model = pickle.load(f)

fitnesses=np.load(file.replace("genotypes","fitnesses").replace(".pkl",".npy"))
top=np.argmax(fitnesses)
#print("@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n\n\n",fitnesses)
env=environment(1,300,friction=0.5)
#print(model)
env.reset()
print("\n\n\n",get_total_mass(env.robot_id))
env.runTrial(model[top],100,delay=1)
env.close()