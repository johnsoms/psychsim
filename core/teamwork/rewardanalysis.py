from Tkinter import *
import os
import itertools
import random
from teamwork import *

trials = 10

positions = [["0,0", "0,1", "1,0"],
             ["0,6", "0,5", "1,6"],
             ["6,0", "6,1", "5,0"],
             ["2,1", "5,2", "6,3"],
             ["3,3", "4,3", "4,2"],
             ["1,5", "1,1", "5,1"],
             ["3,6", "3,2", "6,2"],
             ["0,2", "0,3", "0,4"],
             ["2,0", "3,0", "4,0"],
             ["4,4", "5,3", "6,4"],]
r1 = 1.0
for r2 in {0.25,0.5,1.,2.,4.}:
    for r3 in {0.25,0.5,1.,2.,4.}:
        total_score = 0
        print("1-"+str(r2)+"-"+str(r3))
        for trial in range(trials):
            print(positions[trial])
            run = Scenario(
                MAP_SIZE_X=10,
                MAP_SIZE_Y=10,
                F_ACTORS=3,
                F_START_LOC=positions[trial],
                F_GOAL_LOC=["5,5", "5,5", "5,5"],
                F_ENERGY=[10.0],
                E_ACTORS=3,
                E_START_LOC=["5,4", "4,5", "6,5"],
                E_PATROL_RANGE=5,
                ENEMY=[0.7, 0.7, -1.0],
                AGENT=[[r1, r2, -1.0*r3] for i in range(3)])

            total_score += run.run_without_visual()[0]
            print(total_score)
            print(trial)
            print("________________________\n\n\n")
        path = os.getcwd()+"/rewardtrials4/"
        scorefile = open(path + str(r2) +"_"+ str(r3) + ".txt", "w")
        scorefile.write(str(total_score))
        scorefile.close()

