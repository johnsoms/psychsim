from Tkinter import *
import os, sys, glob
import subprocess


r1 = 1.0
targets=["max","random","group"]
enemy=[1.,1.,1.]
for (Zr2,Zr3) in {(4.,0.25),(1,0.25),(0.5,1.),(2.,2.)}:
    for (Or2, Or3) in {(4., 0.25), (1, 0.25), (0.5, 1.), (2., 2.)}:
        for (Tr2, Tr3) in {(4., 0.25), (1, 0.25), (0.5, 1.), (2., 2.)}:
            for target in targets:
                # os.execl("python","teamwork.py", "-t ",target,"-x ",str(r2),"-y ",str(r3))
                print("python teamwork.py -t "+target+" -x "+str(Zr2)+" -y "+str(Zr3)+" -z "+str(Or2)+" -w "+str(Or3)+" -j "+str(Tr2)+" -k "+str(Tr3))
                # subprocess.call("python teamwork.py -t "+target+" -x "+str(Zr2)+" -y "+str(Zr3)+" -z "+str(Or2)+" -w "+str(Or3)+" -j "+str(Tr2)+" -k "+str(Tr3))