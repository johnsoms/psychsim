# Team of agents that attempts to capture a flag without being caught by enemies
# Agents:
# Explorer - minimize distance between self and goal location
# Distractor - maximize distance between explorer and enemy (new)
# Enemy - minimize distance between self and explorer and distractor
# Base - deploy distractor when explorer in danger (new)

from __future__ import print_function, division
from psychsim.reward import *
from psychsim.pwl import *
from psychsim.action import *
from psychsim.world import *
from psychsim.agent import *
import pyglet
from pyglet.window import key
from threading import Thread
from time import time
import os
import random
import copy



class Scenario:
    def __init__(self,
                 S_ACTORS=0,
                 S_START_R=[[0.0]],
                 RESULT = 0.0,
                 AGENT=[[0.0]],
                 BEST=[0.0],
                 TUTOR=[0.0],
                 MAX = 4.0):

        self.S_ACTORS = S_ACTORS
        self.S_START_R = S_START_R
        self.RESULT = RESULT
        self.AGENT = AGENT
        self.BEST = BEST
        self.TUTOR = TUTOR
        self.MAX = MAX

        self.world = World()
        self.world.defineState(None, 'Max_R', float)
        self.world.setState(None, 'Max_R', self.MAX)
        for i in range(len(self.BEST)):
            self.world.defineState(None,'Best_R'+str(i),float)
            self.world.setState(None,'Best_R'+str(i),self.BEST[i])

        self.create_tutor_agent()

        self.paused = False

        # Parallel action
        # self.world.setOrder([set(self.world.agents.keys())])
        # Sequential action
        self.world.setOrder(self.world.agents.keys())


def create_tutor_agent(self):

    actor = Agent('Tutor')
    self.world.addAgent(actor)
    actor.setHorizon(5)

    self.create_student_agents()

    for index in range(self.S_ACTORS):
        for i in range(len(self.BEST)):
            actor.setReward(minimizeDifference(stateKey('Actor'+str(index),'R'+str(i)),stateKey(None,"Best_R"+str(i))),self.TUTOR[i])

    self.set_tutor_actions()

def create_student_agents(self):
    for index in range(self.S_ACTORS):
        actor = Agent('Student'+str(index))
        self.world.addAgent(actor)
        actor.setHorizon(5)
        for i in range(len(self.BEST)):
            self.world.defineState(actor, 'Best_R' + str(i), float)
            self.world.setState(actor, 'Best_R' + str(i), self.S_START_R[index][i])

    set_student_actions()


def set_tutor_actions(self):



def set_student_actions(self,actor):
    #Maintain Rewards
    action = actor.addAction({'verb': 'No_Change'})
    for i in range(len(self.BEST)):
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'R'+str(i)), 0.))
        self.world.setDynamics(stateKey(action['subject'], ''R'+str(i)'), action, tree)
    for i in range(len(self.BEST)):
        for val in [0.25,0.5,1.,2.,4.]:
            action = actor.addAction({'verb': 'Modify_R'+str(i)+'Accept_'+val})
            tree = makeTree(incrementMatrix(stateKey(action['subject'], 'R'+str(i)), val))
            self.world.setDynamics(stateKey(action['subject'], 'R'+str(i)), action, tree)

            dict = {'if': differenceRow(stateKey(actor.name, 'R'+str(i)),stateKey(None, 'Max_R'),val),
                    True:True,
                    False:False}
            tree = makeTree(dict)
            actor.setLegal(action, tree)

            action = actor.addAction({'verb': 'Modify_R' + str(i) + 'Reject_' + val})
            tree = makeTree(incrementMatrix(stateKey(action['subject'], 'R' + str(i)), -1.*val))
            self.world.setDynamics(stateKey(action['subject'], 'R' + str(i)), action, tree)

            dict = {'if': differenceRow(stateKey(actor.name, 'R' + str(i)), "0.0", val),
                    True: True,
                    False: False}
            tree = makeTree(dict)
            actor.setLegal(action, tree)

def run_without_visual(self):
    while not self.world.terminated():
        result = self.world.step()
        # self.world.explain(result, 2)
    return self.return_score()

def return_score(self):
    result = 0.0
    for index in range(self.S_ACTORS):
        for k in range(len(self.AGENT)):
            student_r = int(self.world.getState('Actor' + str(index), 'R'+str(k)).domain()[0])
            best_r = self.BEST[k]
            score = 10.0 - abs(student_r - best_r)
            result += score
    result /= (self.S_ACTORS*len(self.AGENT))
    return result

def run():
    resultsFile = open("results.txt",'r')
    s_actors = int(resultsFile.readline())
    s_start_r = resultsFile.readline().split(",")
    s_start_r = [float(s) for s in s_start_r]
    result = float(resultsFile.readline())
    agent = resultsFile.readline().split(",")
    agent = [float(a) for a in agent]
    bestFile = open("best.txt",'r')
    best = bestFile.readline().split(",")
    best = [float(b) for b in best]
    max = float(bestFile.readline().split(","))
    tutorFile = open("tutor.txt", 'r')
    tutor = tutorFile.readline().split(",")
    tutor = [float(t) for t in tutor]
    run = Scenario(
                S_ACTORS= s_actors,
                S_START_R= s_start_r,
                RESULT = result,
                AGENT= agent,
                BEST = best,
                TUTOR = tutor,
                MAX = max_r)
    score = run.run_without_visual()
    print(score)
    return score

