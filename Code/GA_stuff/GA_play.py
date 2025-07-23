import numpy as np
import matplotlib.pyplot as plt
if __name__=="__main__":
    import sys
    sys.path.insert(1,"/its/home/drs25/Documents/GitHub/Sim-Biped-Walker/Code/")
    sys.path.insert(1,"C:/Users/dexte/Documents/GitHub/Sim-Biped-Walker/Code")
datapath="/its/home/drs25/Documents/GitHub/Sim-Biped-Walker/"
#datapath="C:/Users/dexte/Documents/GitHub/Sim-Biped-Walker/"

from Biped_controller.pattern import sinBot
from Biped_controller import environment, get_total_mass
import numpy as np
import pickle
from genetic_algorithms import *


error=False
top_geno=None
top=-np.inf
i=1
while not error:
    try:
        file="models/genotypes_dt0.01_Microbial_F3_"+str(i)+"_friction0.5.pkl"

        with open(datapath + file, "rb") as f:
            model = pickle.load(f)

        fitnesses=np.load(datapath +file.replace("genotypes","fitnesses").replace(".pkl",".npy"))
        if np.max(fitnesses)>top:
            top=np.max(fitnesses)
            top_geno=model[np.argmax(fitnesses)]
    except FileNotFoundError:
        print(i)
        error=True
    i+=1
print(top)

#print("@@@@@@@@@@@@@@@@@@@@@@@@@@@\n\n\n\n",fitnesses)
env=environment(1,300,friction=0.5,filename=datapath+"/assets/filerecord.mp4")
#print(model)
env.reset()
print("\n\n\n",get_total_mass(env.robot_id))
env.runTrial(top_geno,1000,delay=0)
env.close()

print(list(top_geno.geno))