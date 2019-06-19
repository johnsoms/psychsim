from Tkinter import *
import os
import itertools
import random
from teamwork import *

trials = 100

positions = [str(i)+","+str(j) for i in range(7) for j in range(7)]
starts = list(itertools.combinations(positions, 3))
random.shuffle(starts)
r1 = 1.0
for r2 in {4.}:
    for r3 in {4.}:
        total_score = 0
        pos_100 = random.sample(starts, trials)
        for trial in range(trials):
            run = Scenario(
                MAP_SIZE_X=10,
                MAP_SIZE_Y=10,
                F_ACTORS=3,
                F_START_LOC=list(starts[trial]),
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
        path = os.getcwd()+"/rewardtrials3/"
        scorefile = open(path + str(r2) +"_"+ str(r3) + ".txt", "w")
        scorefile.write(str(total_score))
        scorefile.close()

