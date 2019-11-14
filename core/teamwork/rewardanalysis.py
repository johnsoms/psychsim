from Tkinter import *
import os, sys, glob
import itertools
import random
from teamwork import *
import moviepy.editor as mpy

trials = 1
r1 = 1.0
enemy=[1.,0.25,0.25]
for r2 in {0.25,0.5,1.,2.,4.}:
    for r3 in {0.5,1.,2.,4.}:
        total_score = 0
        total = ""
        print("1-"+str(r2)+"-"+str(r3))
        for trial in range(trials):
            file_list = glob.glob('*.png')
            for x in file_list:
                os.remove(x)
            run = Scenario(
                MAP_SIZE_X=7,
                MAP_SIZE_Y=7,
                F_ACTORS=3,
                F_START_LOC=["1,2", "2,2", "2,1"],
                F_GOAL_LOC=["5,5", "5,5", "5,5"],
                F_ENERGY=[10.0],
                E_ACTORS=3,
                E_START_LOC=["5,4", "4,4", "4,5"],
                E_GOAL_LOC=["1,1", "1,1", "1,1"],
                E_PATROL_RANGE=5,
                ENEMY=enemy,
                AGENT=[[r1, r2, r3] for i in range(3)],
                ABILITY=[[1.0, 1.0], [1.0, 1.0], [1.0, 1.0]],
                ASYNC = False)
            run.file_num = 0
            if sys.argv[1] == "v":
                run.run_with_visual()
                gif_name = "1-"+str(r2)+"-"+str(r3)+"_"+"BlueStart"
                fps = 6
                file_list = glob.glob('*.png')  # Get all the pngs in the current directory
                list.sort(file_list, key=lambda x: int(
                    x.split('-')[1].split('.png')[
                        0]))  # Sort the images by #, this may need to be tweaked for your use case
                short_file_list = file_list[:-1]
                try:
                    clip = mpy.ImageSequenceClip(short_file_list, fps=fps)
                    clip.write_gif('{}.gif'.format(gif_name), fps=fps)
                except:
                    short_file_list = file_list[:-2]
                    clip = mpy.ImageSequenceClip(short_file_list, fps=fps)
                    clip.write_gif('{}.gif'.format(gif_name), fps=fps)
                for x in file_list:
                    os.remove(x)
            else:
                run.run_without_visual()
            result = run.return_score()
            del run
            total_score += result['team']
            turns = str(result['turns'])+"\n"
            total += turns
            print(turns)
            print(trial)
            print("Score: "+str(total_score))
            print("________________________\n\n\n")
        path = os.getcwd()+"/rewardtrials10/"
        scorefile = open(path + str(r2) +"_"+ str(r3) +"_"+"BlueStartVs"+str(enemy[0])+str(enemy[1])+str(enemy[2])+".txt", "w")

        scorefile.write(str(total_score))
        scorefile.close()
