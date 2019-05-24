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
from teamwork import *
import pyglet
from pyglet.window import key
from threading import Thread
from time import time
import os
import random
import copy



class Tutoring:
    def __init__(self,
                 S_ACTORS=0,
                 S_START_R=[[0.0]],
                 S_TRUST_T=[[0.0]],
                 S_TRUST_S=[[0.0]],
                 BEST=[[0.0]],
                 TUTOR=[0.0],
                 MAX = 4.0):

        self.S_ACTORS = S_ACTORS
        self.S_START_R = S_START_R
        self.BEST = BEST
        self.TUTOR = TUTOR
        self.MAX = MAX

        self.world = World()
        self.world.defineState(None, 'Max_R', float)
        self.world.setState(None, 'Max_R', self.MAX)
        self.world.defineState(None, 'Last_Result', float)
        self.world.setState(None, 'Last_Result', 0.)
        self.world.addTermination(makeTree({'if': thresholdRow(stateKey(None, 'Last_Result'), 10.),
                                            True: True, False: False}))
        for i in range(len(self.BEST)):
            for j in range(len(self.BEST[i])):
                self.world.defineState(None,'Best_R'+str(j)+"_"+str(i),float)
                self.world.setState(None,'Best_R'+str(j)+"_"+str(i),self.BEST[i])

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
            for j in range(len(self.BEST[0])):
                actor.setReward(minimizeDifference(stateKey('Actor'+str(index),'R'+str(j)),stateKey(None,"Best_R"+str(j)+"_"+str(i))),self.TUTOR[i])

    self.set_tutor_actions(actor)


def create_student_agents(self):
    for index in range(self.S_ACTORS):
        actor = Agent('Student'+str(index))
        self.world.addAgent(actor)
        actor.setHorizon(5)
        for i in range(len(self.BEST[0])):
            self.world.defineState(actor, 'R' + str(i), float)
            self.world.setState(actor, 'R' + str(i), self.S_START_R[index][i])
            self.world.defineState(actor, 'Suggest_R' + str(i), float)
            self.world.setState(actor, 'Suggest_R' + str(i), self.S_START_R[index][i])

        student_tutor_trust = self.S_TRUST_T[index]
        key = world.defineRelation(actor.name, 'Tutor', 'trusts')
        world.setFeature(key, student_tutor_trust)

        student_self_trust = self.S_TRUST_S[index][index]
        world.defineState(actor.name, 'Self-Trust', float)
        world.setState(actor.name, 'Self-Trust', student_self_trust)
        others = [str(k) for k in range(self.S_ACTORS)]
        others.remove([str(index)])
        student_student_trust = {s:0.0 for s in others}
        for j in range(len(self.BEST[0])):
            actor.setReward(minimizeDifference(stateKey(actor.name, 'R' + str(j)),
                                               stateKey(actor.name, "R" + str(j))),
                            student_self_trust)
            for s in others:
                student_student_trust[s] = self.S_TRUST_S[index][int(s)]
                key = world.defineRelation(actor.name, 'Actor'+s, 'trusts')
                world.setFeature(key, student_student_trust[s])
                actor.setReward(minimizeDifference(stateKey(actor.name, 'R' + str(j)),
                                                   stateKey('Actor' + s, "R" + str(j))),
                                student_student_trust[s])
            for i in range(len(self.BEST)):
                actor.setReward(minimizeDifference(stateKey(actor.name, 'R' + str(j)),
                                                   stateKey(None, "Suggest_R" + str(j) + "_" + str(i))), student_tutor_trust)


        set_student_actions(actor)


def update_student_agents(self):
    for index in range(self.S_ACTORS):
        actor = Agent('Student'+str(index))

        student_tutor_trust = self.calculate_trust_tutor()
        key = binaryKey(actor.name, 'Tutor', 'trusts')
        world.setFeature(key, student_tutor_trust)

        student_self_trust = self.calculate_trust_self()
        world.setState(actor.name, 'Self-Trust', student_self_trust)
        others = [str(k) for k in range(self.S_ACTORS)]
        others.remove([str(index)])
        student_student_trust = {s:0.0 for s in others}
        for j in range(len(self.BEST[0])):
            actor.setReward(minimizeDifference(stateKey(actor.name, 'R' + str(j)),
                                               stateKey(actor.name, "R" + str(j))),
                            student_self_trust)
            for s in others:
                student_student_trust[s] = self.calculate_trust_other(int(s))
                key = world.defineRelation(actor.name, 'Actor'+s, 'trusts')
                world.setFeature(key, student_student_trust[s])
                actor.setReward(minimizeDifference(stateKey(actor.name, 'R' + str(j)),
                                                   stateKey('Actor' + s, "R" + str(j))),
                                student_student_trust[s])
            for i in range(len(self.BEST)):
                actor.setReward(minimizeDifference(stateKey(actor.name, 'R' + str(j)),
                                                   stateKey(None, "Suggest_R" + str(j) + "_" + str(i))), student_tutor_trust)


def calculate_trust_tutor(self):



def calculate_trust_self(self):



def calculate_trust_other(self,index):



def set_tutor_actions(self, actor):
    for index in range(self.S_ACTORS):
        action = actor.addAction({'verb': 'No_Tutor_Actor'+str(index)})
        for i in range(len(self.BEST[0])):
            tree = makeTree(incrementMatrix(stateKey('Actor'+str(index), 'Suggest_R' + str(i)), 0.))
            self.world.setDynamics(stateKey('Actor'+str(index), 'Suggest_R'+str(i)), action, tree)
        # Modify Rewards by either accepting or rebelling against instruction
        for i in range(len(self.BEST[0])):
            for val in [0.25,0.5,0.75,1.,1.25,1.5,1.75,2.,2.25,2.5,2.75,3,3.25,3.5,3.75,4.]:
                action = actor.addAction({'verb': 'Suggest_R' + str(i) + 'Increase_' + val})
                tree = makeTree(incrementMatrix(stateKey('Actor'+str(index), 'Suggest_R' + str(i)), val))
                self.world.setDynamics(stateKey('Actor'+str(index), 'Suggest_R' + str(i)), action, tree)

                dict = {'if': differenceRow(stateKey('Actor'+str(index), 'Suggest_R' + str(i)), stateKey(None, 'Max_R'), val),
                        True: True,
                        False: False}
                tree = makeTree(dict)
                actor.setLegal(action, tree)

                action = actor.addAction({'verb': 'Suggest_R' + str(i) + 'Decrease_' + val})
                tree = makeTree(incrementMatrix(stateKey('Actor'+str(index), 'Suggest_R' + str(i)), -1. * val))
                self.world.setDynamics(stateKey('Actor'+str(index), 'Suggest_R' + str(i)), action, tree)

                dict = {'if': differenceRow(stateKey('Actor'+str(index), 'Suggest_R' + str(i)), "0.0", val),
                        True: True,
                        False: False}
                tree = makeTree(dict)
                actor.setLegal(action, tree)


def set_student_actions(self, actor):
    # Maintain Rewards
    action = actor.addAction({'verb': 'No_Change'})
    for i in range(len(self.BEST)):
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'R'+str(i)), 0.))
        self.world.setDynamics(stateKey(action['subject'], 'R'+str(i)), action, tree)
    # Modify Rewards by either accepting or rebelling against instruction
    for i in range(len(self.BEST)):
        for val in [0.25,0.5,0.75,1.,1.25,1.5,1.75,2.,2.25,2.5,2.75,3,3.25,3.5,3.75,4.]:
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
    num_runs = 0
    last_run = Scenario(
        MAP_SIZE_X=7,
        MAP_SIZE_Y=7,
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
        AGENT=self.S_START_R
    )
    game_result = 0.
    indv_result = [0. for i in range(self.S_ACTORS)]
    for i in range(10):
        win = last_run.run_without_visual()
        game_result += win[0]
        indv_result += win[1]
    self.world.setState(None, 'Last_Result', game_result)
    self.update_student_agents(game_result, indv_result)

    while not self.world.terminated():
        result = self.world.step()
        num_runs+=1
        # self.world.explain(result, 2)
        score = self.return_score()
        students = [[int(self.world.getState('Actor'+str(i), 'R' + str(j)).domain()[0]) for j in range(len(self.BEST[0]))] for i in range(self.S_ACTORS)]
        last_run = Scenario(
            MAP_SIZE_X=7,
            MAP_SIZE_Y=7,
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
            AGENT=[students[i] for i in range(self.S_ACTORS)]
        )
        game_result = last_run.run_without_visual()
        self.world.setState(None, 'Last_Result', game_result)
        self.update_student_agents()

def return_score(self):
    result = 0.0
    for index in range(self.S_ACTORS):
        for k in range(len(self.BEST)):
            student_r = int(self.world.getState('Actor' + str(index), 'R'+str(k)).domain()[0])
            best_r = self.BEST[k]
            score = 10.0 - abs(student_r - best_r)
            result += score
    result /= (self.S_ACTORS*len(self.BEST))
    return result

def run():
    learn = Tutoring(
                S_ACTORS=3,
                S_START_R=[[1.0, 1.0, -1.0],[1.0, 1.0, -1.0],[1.0, 1.0, -1.0]],
                S_TRUST_T=[1.0,1.0,1.0],
                S_TRUST_S=[[1.0,1.0,1.0,1.0],[1.0,1.0,1.0,1.0],[1.0,1.0,1.0,1.0]],
                BEST=[[1.,1.,-1.],[1.,1.,-1.],[1.,1.,-1.]],
                TUTOR=[1.0,1.0,1.0],
                MAX=4.0)
    score = learn.run_without_visual()
    print(score)
    return score

