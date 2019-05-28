import Tkinter
from Tkinter import *
import os
from teamwork import *


r1 = 1.0
for r2 in {0.25,0.5,1.0,2.0,4.0}:
    for r3 in {0.25,0.5,1.0,2.0,4.0}:
        total_score = 0
        for trial in range(10):
            run = Scenario(
                MAP_SIZE_X=10,
                MAP_SIZE_Y=10,
                F_ACTORS=3,
                F_START_LOC=[str(random.randint(0, 6)) + "," + str(random.randint(0, 6)),
                             str(random.randint(0, 6)) + "," + str(random.randint(0, 6)),
                             str(random.randint(0, 6)) + "," + str(random.randint(0, 6))],
                F_GOAL_LOC=["5,5", "5,5", "5,5"],
                F_ENERGY=[10.0],
                E_ACTORS=3,
                E_START_LOC=["5,4", "4,5", "6,5"],
                E_PATROL_RANGE=5,
                # D_ACTORS=1,
                # D_START_LOC=["2,3"],
                # S_ACTORS=1,
                # S_ENERGY=[10.0],
                # S_START_LOC=["1,4"],
                # BASE=[b1, b2],
                # DISTRACTOR=[h1, h2],
                # SUPPLIER=[d1, d2],
                ENEMY=[0.7, 0.7, -1.0],
                AGENT=[[r1, r2, -1.0*r3] for i in range(3)])

            total_score += run.run_without_visual()[0]
        # path = os.getcwd()+"/rewardtrials/"
        # scorefile = open(path + str(r2) +"_"+ str(r3) + ".txt", "w")
        # scorefile.write(str(total_score))
        # scorefile.close()

