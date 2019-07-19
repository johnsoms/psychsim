from Tkinter import *
import os
import itertools
import random
from teamwork import *

trials = 10

positions = [["0,0", "0,1", "1,0"],  # All agents in bottom left corner
             ["0,6", "0,5", "1,6"],  # All agents in top left corner
             ["6,0", "6,1", "5,0"],  # All agents in bottom right corner
             ["4,6", "5,6", "6,6"],  # All agents in top right corner
             ["3,3", "4,3", "4,2"],  # All agents in center
             ["0,6", "0,0", "6,0"],  # Agents spread out in bottom left, top left, and bottom right corners
             ["3,6", "3,2", "6,2"],  # Agents spread out in center, top middle, and middle right
             ["0,2", "0,3", "0,4"],  # All agents on middle of left side
             ["2,0", "3,0", "4,0"],  # All agents on middle of bottom side
             ["4,4", "5,3", "6,4"], # Agents directly below opposing agents
            ]
r1 = 1.0
for r2 in {0.25,0.5,1.,2.,4.}:
    for r3 in {0.25,0.5,1.,2.,4.}:
        # total_score = 0
        total = ""
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
                ENEMY=[0.7, -0.1],
                AGENT=[[r1, r2, -1.0*r3] for i in range(3)])

            # total_score += run.run_without_visual()[0]
            turns = str(run.run_without_visual()[0])+"\n"
            total += turns
            print(turns)
            print(trial)
            print("________________________\n\n\n")
        path = os.getcwd()+"/rewardtrials7/"
        scorefile = open(path + str(r2) +"_"+ str(r3) + ".txt", "w")
        scorefile.write(str(total))
        scorefile.close()

