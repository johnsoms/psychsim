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
                 S_START_R=[0.0],
                 RESULT = 0.0,
                 AGENT=[0.0]):

        self.S_ACTORS = S_ACTORS
        self.S_START_R = S_START_R
        self.RESULT = RESULT
        self.AGENT = AGENT

        self.world = World()
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

    # Set agent's starting location
    self.world.defineState(actor.name, 'x', int)
    self.world.setState(actor.name, 'x', self.f_get_start_x(index))
    self.world.defineState(actor.name, 'goal_x', int)
    self.world.setState(actor.name, 'goal_x', self.f_get_goal_x(index))

    self.world.defineState(actor.name, 'y', int)
    self.world.setState(actor.name, 'y', self.f_get_start_y(index))
    self.world.defineState(actor.name, 'goal_y', int)
    self.world.setState(actor.name, 'goal_y', self.f_get_goal_y(index))

    self.world.defineState(actor.name, 'health', int)
    self.world.setState(actor.name, 'health', 3)

    # Positive reward for going towards goal
    actor.setReward(minimizeDifference(stateKey(actor.name, 'x'), stateKey(actor.name, 'goal_x')),
                    self.AGENT[0])
    actor.setReward(minimizeDifference(stateKey(actor.name, 'y'), stateKey(actor.name, 'goal_y')),
                    self.AGENT[0])
    actor.setReward(achieveFeatureValue(stateKey(actor.name, 'health'), '0'), self.AGENT[2])
    # Negative reward for being eliminated
    actors.append(actor)
    enemy = 'Enemy' + str(index)

    # actor.setReward(minimizeDifference(stateKey(actor.name, 'x'), stateKey(enemy, 'x')), self.AGENT[1])
    # actor.setReward(minimizeDifference(stateKey(actor.name, 'y'), stateKey(enemy, 'y')), self.AGENT[1])
    # actor.setReward(minimizeDifference(stateKey(actor.name, 'y'), stateKey(enemy, 'y')), self.AGENT[1])
    self.create_student_agents()

def create_student_agents(self):



def set_tutor_actions(self):



def set_student_actions(self):



def run_without_visual(self):
    while not self.world.terminated():
        result = self.world.step()
        # self.world.explain(result, 2)
    return self.return_score()
    self.evaluate_score()


def run():
    resultsFile = open("results.txt",'r')
    s_actors = int(resultsFile.readline())
    s_start_r = resultsFile.readline().split(",")
    s_start_r = [float(s) for s in s_start_r]
    result = float(resultsFile.readline())
    agent = resultsFile.readline().split(",")
    agent = [float(a) for a in agent]
    run = Scenario(
                S_ACTORS= s_actors,
                S_START_R= s_start_r,
                RESULT = result,
                AGENT= agent)
    score = run.run_without_visual()
    print(score)
    return score

