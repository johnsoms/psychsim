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

from argparse import ArgumentParser
import csv
import logging

import glob
import moviepy.editor as mpy

class Scenario:
    def __init__(self,
                 MAP_SIZE_X=0,
                 MAP_SIZE_Y=0,
                 F_ACTORS=0,
                 F_START_LOC=[],
                 F_GOAL_LOC=[],
                 F_ENERGY=[],
                 E_ACTORS=0,
                 E_START_LOC=[],
                 E_GOAL_LOC=[],
                 E_PATROL_RANGE=5,
                 # D_ACTORS=0,
                 # D_START_LOC=[],
                 # S_ACTORS=0,
                 # S_ENERGY=[],
                 S_START_LOC=[],
                 BASE=[0.0, 0.0],
                 # DISTRACTOR=[0.0, 0.0],
                 # SUPPLIER=[0.0, 0.0],
                 ENEMY=[0.0, 0.0, 0.0],
                 AGENT=[[0.0, 0.0, 0.0]],
                 ABILITY=[[1.0,1.0],[1.0,1.0],[1.0,1.0]],
                 ASYNC = False):

        self.MAP_SIZE_X = MAP_SIZE_X
        self.MAP_SIZE_Y = MAP_SIZE_Y
        self.DEFENSE_LIMIT = math.sqrt(MAP_SIZE_X**2+MAP_SIZE_Y**2)/2.
        self.F_ACTORS = F_ACTORS
        self.F_START_LOC = F_START_LOC
        self.F_GOAL_LOC = F_GOAL_LOC
        self.F_ENERGY = F_ENERGY
        self.E_ACTORS = E_ACTORS
        self.E_START_LOC = E_START_LOC
        self.E_GOAL_LOC = E_GOAL_LOC
        self.E_PATROL_RANGE = E_PATROL_RANGE
        # self.D_ACTORS = D_ACTORS
        # self.D_START_LOC = D_START_LOC
        # self.S_ACTORS = S_ACTORS
        # self.S_ENERGY = S_ENERGY
        # self.S_START_LOC = S_START_LOC
        self.BASE = BASE
        # self.DISTRACTOR = DISTRACTOR
        # self.SUPPLIER = SUPPLIER
        self.ENEMY = ENEMY
        self.AGENT = AGENT
        self.ABILITY = ABILITY
        self.ASYNC = ASYNC

        self.world = World()
        self.world.defineState(None, 'turns', int)
        self.world.setState(None, 'turns', 0)
        self.world.defineState(None,'zero', float)
        self.world.setState(None, 'zero', 0.)
        self.world.addTermination(makeTree({'if': thresholdRow(stateKey(None, 'turns'), 60),
                                            True: True, False: False}))
        self.enemy_agents = []
        self.friendly_agents = self.create_friendly_agents()

        # self.create_distract_agents()
        # self.create_supply_agents()
        # self.create_base()
        self.file_num = 0
        self.paused = False

        # Parallel action
        # self.world.setOrder([set(self.world.agents.keys())])
        # Sequential action
        if self.ASYNC:
            self.world.setOrder([set(self.world.agents.keys())])
        else:
            self.world.setOrder(self.world.agents.keys())

    def f_get_current_x(self, actor):
        return self.world.getState(actor.name, 'x').domain()[0]

    def f_get_current_y(self, actor):
        return self.world.getState(actor.name, 'y').domain()[0]

    def f_get_start_x(self, index):
        return int((self.F_START_LOC[index]).split(",", 1)[0])

    def f_get_start_y(self, index):
        return int((self.F_START_LOC[index]).split(",", 1)[1])

    def f_get_goal_x(self, index):
        return int((self.F_GOAL_LOC[index]).split(",", 1)[0])

    def f_get_goal_y(self, index):
        return int((self.F_GOAL_LOC[index]).split(",", 1)[1])

    def f_get_start_energy(self, index):
        return self.F_ENERGY[index]

    def f_get_current_energy(self, actor):
        return self.world.getState(actor.name, 'energy').domain()[0]

    def e_get_current_x(self, actor):
        return self.world.getState(actor.name, 'x').domain()[0]

    def e_get_current_y(self, actor):
        return self.world.getState(actor.name, 'y').domain()[0]

    def e_get_start_x(self, index):
        return int((self.E_START_LOC[index]).split(",", 1)[0])

    def e_get_start_y(self, index):
        return int((self.E_START_LOC[index]).split(",", 1)[1])

    def e_get_goal_x(self, index):
        return int((self.E_GOAL_LOC[index]).split(",", 1)[0])

    def e_get_goal_y(self, index):
        return int((self.E_GOAL_LOC[index]).split(",", 1)[1])

    def d_get_start_x(self, index):
        return int((self.D_START_LOC[index]).split(",", 1)[0])

    def d_get_start_y(self, index):
        return int((self.D_START_LOC[index]).split(",", 1)[1])

    def s_get_start_x(self, index):
        return int((self.S_START_LOC[index]).split(",", 1)[0])

    def s_get_start_y(self, index):
        return int((self.S_START_LOC[index]).split(",", 1)[1])

    def s_get_current_x(self, actor):
        return self.world.getState(actor.name,'x').domain()[0]

    def s_get_current_y(self, actor):
        return self.world.getState(actor.name,'y').domain()[0]

    def s_get_start_energy(self, index):
        return self.S_ENERGY[index]

    def s_get_current_energy(self, actor):
        return self.world.getState(actor.name, 'energy').domain()[0]

    def find_distance(self, start_x, start_y, goal_x, goal_y):
        return abs(goal_x - start_x) + abs(goal_y - start_y)

    def calculateDistance(self,a_x,a_y,s_x,s_y):
        return math.sqrt((a_x-s_x)^2 + (a_y - s_y)^2)

    def calcPosRadius(self, agent, dim):
        pos = self.world.getState(agent,dim).domain()[0]
        size = 0
        if dim == "x":
            size = self.MAP_SIZE_X - 1
        else:
            size = self.MAP_SIZE_Y - 1
        if pos == 0:
            return [(KeyedVector({stateKey(agent,dim): self.world.getState(agent,dim).domain()[0]}),0.5),
            (KeyedVector({stateKey(agent, dim): self.world.getState(agent, dim).domain()[0]+1}), 0.5)]
        elif pos == size:
            return [(KeyedVector({stateKey(agent,dim):self.world.getState(agent,dim).domain()[0]}),0.5),
            (KeyedVector({stateKey(agent, dim): self.world.getState(agent, dim).domain()[0]-1}), 0.5)]
        return [(KeyedVector({stateKey(agent,dim):self.world.getState(agent,dim).domain()[0]}),0.4),
            (KeyedVector({stateKey(agent, dim): self.world.getState(agent, dim).domain()[0]+1}), 0.3),
            (KeyedVector({stateKey(agent, dim): self.world.getState(agent, dim).domain()[0] - 1}), 0.3)]

    def create_base(self):
        for index in range(0, self.D_ACTORS):
            base = Agent('Base' + str(index))
            self.world.addAgent(base)
            base.setHorizon(5)

            self.world.defineState(base.name, 'x', int)
            self.world.setState(base.name, 'x', 0)

            self.world.defineState(base.name, 'y', int)
            self.world.setState(base.name, 'y', 0)

            # Deploy distractor
            action = base.addAction({'verb': 'DeployDistractor'})
            tree = makeTree(setToConstantMatrix(stateKey('Distractor' + str(index), 'deployed'), True))
            self.world.setDynamics(stateKey('Distractor' + str(index), 'deployed'), action, tree)

            # # Deploy supplier
            # action = base.addAction({'verb': 'DeploySupplier'})
            # tree = makeTree(setToConstantMatrix(stateKey('Supplier' + str(index), 'deployed'), True))
            # self.world.setDynamics(stateKey('Supplier' + str(index), 'deployed'), action, tree)

            # Nop
            action = base.addAction({'verb': 'Wait'})
            tree = makeTree(incrementMatrix(stateKey(action['subject'], 'x'), 0.))
            self.world.setDynamics(stateKey(action['subject'], 'x'), action, tree)
            tree = makeTree(incrementMatrix(stateKey(action['subject'], 'y'), 0.))
            self.world.setDynamics(stateKey(action['subject'], 'y'), action, tree)

            base.setReward(
                minimizeDifference(stateKey('Distractor' + str(index), 'x'), stateKey('Enemy' + str(index), 'x')),
                self.BASE[0])
            base.setReward(
                minimizeDifference(stateKey('Distractor' + str(index), 'y'), stateKey('Enemy' + str(index), 'y')),
                self.BASE[0])

            base.setReward(minimizeFeature(stateKey('Distractor' + str(index), 'cost')), self.BASE[1])

            # base.setReward(
            #     minimizeDifference(stateKey('Supplier' + str(index), 'x'), stateKey('Actor' + str(index), 'x')),
            #     self.BASE[0])
            # base.setReward(
            #     minimizeDifference(stateKey('Supplier' + str(index), 'y'), stateKey('Actor' + str(index), 'y')),
            #     self.BASE[0])

            base.setReward(minimizeFeature(stateKey('Supplier' + str(index), 'cost')), self.BASE[1])

    def create_friendly_agents(self):
        actors = []
        for index in range(0, self.F_ACTORS):
            actor = Agent('Actor' + str(index))
            self.world.addAgent(actor)
            actor.setHorizon(5)

            # Set agent's starting location
            self.world.defineState(actor.name, 'x', int)
            self.world.setState(actor.name, 'x', self.f_get_start_x(index))
            self.world.defineState(actor.name, 'start_x', int)
            self.world.setState(actor.name, 'start_x', self.f_get_start_x(index))
            self.world.defineState(actor.name, 'goal_x', int)
            self.world.setState(actor.name, 'goal_x', self.f_get_goal_x(index))

            self.world.defineState(actor.name, 'y', int)
            self.world.setState(actor.name, 'y', self.f_get_start_y(index))
            self.world.defineState(actor.name, 'start_y', int)
            self.world.setState(actor.name, 'start_y', self.f_get_start_y(index))
            self.world.defineState(actor.name, 'goal_y', int)
            self.world.setState(actor.name, 'goal_y', self.f_get_goal_y(index))

            self.world.defineState(actor.name, 'health', int)
            self.world.setState(actor.name, 'health', 3)
            self.world.defineState(actor.name, 'defenses', int)
            self.world.setState(actor.name, 'defenses', 0)

            # Positive reward for going towards goal
            actor.setReward(minimizeDifference(stateKey(actor.name, 'x'), stateKey(actor.name, 'goal_x')),
                            self.AGENT[index][0])
            actor.setReward(minimizeDifference(stateKey(actor.name, 'y'), stateKey(actor.name, 'goal_y')),
                            self.AGENT[index][0])
            # Negative reward for being eliminated
            actors.append(actor)
        self.create_enemy_agents()
        for index in range(0, self.F_ACTORS):
            actor = actors[index]
            self.world.setModel(actor.name, True)
            # Reward for attacking enemy
            for index2 in range(0, self.E_ACTORS):
                enemy = 'Enemy' + str(index2)
                # start = self.E_START_LOC[index2]
                # start_x = int(start.split(",")[0])
                # start_y = int(start.split(",")[1])
                # omega = actor.defineObservation('perceived_enemy'+str(index2)+'_health', domain=int)
                # real = stateKey(enemy, 'health')
                # actor.setBelief(stateKey(enemy, 'health'),3,True)
                # actor.setBelief(stateKey(enemy, 'x'), start_x, True)
                # actor.setBelief(stateKey(enemy, 'y'), start_y, True)

                # tree = makeTree({'if': differenceRow(stateKey(actor.name, 'x'), stateKey(enemy, 'x'), -2),
                #                  True: {'if': differenceRow(stateKey(actor.name, 'x'), stateKey(enemy, 'x'), 2),
                #                         True: {'distribution': self.calcPosRadius(enemy, 'x')},
                #                         False: {
                #                             'if': differenceRow(stateKey(actor.name, 'y'), stateKey(enemy, 'y'), -2),
                #                             True: {
                #                                 'if': differenceRow(stateKey(actor.name, 'y'), stateKey(enemy, 'y'), 2),
                #                                 True: {'distribution':  [(setToFeatureMatrix(omega,real),0.3), # Observation of true value with some probability
                #                                       (setToFeatureMatrix(omega,real,shift=1),              # Observation of off-by-1 value otherwise
                #                                       1.-0.3)]},
                #                                 False: {'distribution': [(KeyedVector({stateKey(enemy, 'health'):
                #                                                                            self.world.getState(enemy,
                #                                                                                                'health').domain()[
                #                                                                                0]}), 1.0)]}
                #                                 },
                #                             False: {'distribution': [(setToFeatureMatrix(omega,real),0.3), # Observation of true value with some probability
                #                                       (setToFeatureMatrix(omega,real,shift=1),              # Observation of off-by-1 value otherwise
                #                                       1.-0.3)]}
                #                             }
                #                         },
                #                  False: {'distribution': [(setToFeatureMatrix(omega,real),0.3), # Observation of true value with some probability
                #                                       (setToFeatureMatrix(omega,real,shift=1),              # Observation of off-by-1 value otherwise
                #                                       1.-0.3)]}
                #                  })
                #
                # actor.defineObservation(stateKey(enemy, 'health'), tree)

                # tree = makeTree({'if': differenceRow(stateKey(actor.name, 'x'),stateKey(enemy, 'x'), -2),
                #                  True: {'if': differenceRow(stateKey(actor.name, 'x'),stateKey(enemy, 'x'), 2),
                #                         True: {'distribution': self.calcPosRadius(enemy,'x')},
                #                         False: {'if': differenceRow(stateKey(actor.name, 'y'),stateKey(enemy, 'y'), -2),
                #                             True: {'if': differenceRow(stateKey(actor.name, 'y'),stateKey(enemy, 'y'), 2),
                #                                 True: {'distribution': self.calcPosRadius(enemy,'x')},
                #                                 False: {'distribution': [(KeyedVector({stateKey(enemy,'x'): self.world.getState(enemy, 'x').domain()[0]}), 1.0)]}
                #                                    },
                #                             False: {'distribution': self.calcPosRadius(enemy,'x')}
                #                                 }
                #                         },
                #                  False: {'distribution': self.calcPosRadius(enemy,'x')}
                #                  })
                #
                # actor.defineObservation(stateKey(enemy, 'x'), tree)
                #
                # tree = makeTree({'if': differenceRow(stateKey(actor.name, 'x'),stateKey(enemy, 'x'), -2),
                #                  True: {'if': differenceRow(stateKey(actor.name, 'x'),stateKey(enemy, 'x'), 2),
                #                         True: {'distribution': self.calcPosRadius(enemy,'y')},
                #                         False: {'if': differenceRow(stateKey(actor.name, 'y'),stateKey(enemy, 'y'), -2),
                #                             True: {'if': differenceRow(stateKey(actor.name, 'y'),stateKey(enemy, 'y'), 2),
                #                                 True: {'distribution': self.calcPosRadius(enemy,'y')},
                #                                 False: {'distribution': [(KeyedVector({stateKey(enemy,'y'): self.world.getState(enemy, 'y').domain()[0]}), 1.0)]}
                #                                    },
                #                             False: {'distribution': self.calcPosRadius(enemy,'y')}
                #                                 }
                #                         },
                #                  False: {'distribution': self.calcPosRadius(enemy,'y')}
                #                  })
                #
                # actor.defineObservation(stateKey(enemy, 'y'), tree)

                actor.setReward(minimizeDifference(stateKey(enemy, 'x'), stateKey(enemy, 'goal_x')),
                                -1*self.AGENT[index][1])
                actor.setReward(minimizeDifference(stateKey(enemy, 'y'), stateKey(enemy, 'goal_y')),
                                -1*self.AGENT[index][1])
            for index2 in [i for i in range(0, self.F_ACTORS) if i != index]:
                ally = 'Actor'+str(index2)
                # start = self.F_START_LOC[index2]
                # start_x = int(start.split(",")[0])
                # start_y = int(start.split(",")[1])
                # actor.setBelief(stateKey(ally, 'health'), 3, True)
                # actor.setBelief(stateKey(ally, 'x'), Distribution({start_x:1.0}), True)
                # actor.setBelief(stateKey(ally, 'y'), Distribution({start_y:1.0}), True)
                #
                # tree = makeTree({'if': differenceRow(stateKey(actor.name, 'x'), stateKey(ally, 'x'), -2),
                #                  True: {'if': differenceRow(stateKey(actor.name, 'x'), stateKey(ally, 'x'), 2),
                #                         True: {'distribution': self.calcPosRadius(ally, 'x')},
                #                         False: {
                #                             'if': differenceRow(stateKey(actor.name, 'y'), stateKey(ally, 'y'), -2),
                #                             True: {
                #                                 'if': differenceRow(stateKey(actor.name, 'y'), stateKey(ally, 'y'), 2),
                #                                 True: {'distribution': self.calcPosRadius(ally, 'x')},
                #                                 False: {'distribution': [(KeyedVector({stateKey(ally, 'x'):
                #                                                                            self.world.getState(ally,
                #                                                                                                'x').domain()[
                #                                                                                0]}), 1.0)]}
                #                                 },
                #                             False: {'distribution': self.calcPosRadius(ally, 'x')}
                #                             }
                #                         },
                #                  False: {'distribution': self.calcPosRadius(ally, 'x')}
                #                  })
                #
                # actor.defineObservation(stateKey(enemy, 'x'), tree)
                #
                # tree = makeTree({'if': differenceRow(stateKey(actor.name, 'x'), stateKey(ally, 'x'), -2),
                #                  True: {'if': differenceRow(stateKey(actor.name, 'x'), stateKey(ally, 'x'), 2),
                #                         True: {'distribution': self.calcPosRadius(ally, 'y')},
                #                         False: {
                #                             'if': differenceRow(stateKey(actor.name, 'y'), stateKey(ally, 'y'), -2),
                #                             True: {
                #                                 'if': differenceRow(stateKey(actor.name, 'y'), stateKey(ally, 'y'), 2),
                #                                 True: {'distribution': self.calcPosRadius(ally, 'y')},
                #                                 False: {'distribution': [(KeyedVector({stateKey(ally, 'y'):
                #                                                                            self.world.getState(ally,
                #                                                                                                'y').domain()[
                #                                                                                0]}), 1.0)]}
                #                                 },
                #                             False: {'distribution': self.calcPosRadius(ally, 'y')}
                #                             }
                #                         },
                #                  False: {'distribution': self.calcPosRadius(ally, 'y')}
                #                  })
                #
                # actor.defineObservation(stateKey(enemy, 'y'), tree)
                actor.setReward(minimizeDifference(stateKey(ally, 'x'), stateKey(ally, 'goal_x')),
                                self.AGENT[index][2])
                actor.setReward(minimizeDifference(stateKey(ally, 'y'), stateKey(ally, 'goal_y')),
                                self.AGENT[index][2])
            self.set_friendly_actions(actor)

            # Terminate if agent reaches goal
            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'x'), stateKey(actor.name, 'goal_x')),
                             True: {'if': equalFeatureRow(stateKey(actor.name, 'y'), stateKey(actor.name, 'goal_y')),
                                    True: True,
                                    False: False},
                             False: False})
            self.world.addTermination(tree)

            # self.set_friendly_models(actor, index)

        return actors

    def set_friendly_actions(self, actor):
        # Nop
        i = int(actor.name[-1])
        # print(i)
        action = actor.addAction({'verb': 'Wait'})
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'x'), 0))
        self.world.setDynamics(stateKey(action['subject'], 'x'), action, tree)
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'y'), 0))
        self.world.setDynamics(stateKey(action['subject'], 'y'), action, tree)
        tree = makeTree(incrementMatrix('turns', 1.0))
        self.world.setDynamics(stateKey(None, 'turns'), action, tree)
        # tree = makeTree(incrementMatrix(stateKey(action['subject'], 'energy'), 0.0))
        # self.world.setDynamics(stateKey(action['subject'], 'energy'), action, tree)

        # Increment X position
        action = actor.addAction({'verb': 'MoveRight'})
        tree = makeTree({'distribution':[
                (incrementMatrix(stateKey(action['subject'], 'x'), 1), self.ABILITY[i][0]),
                (incrementMatrix(stateKey(action['subject'], 'x'), -1), (1. - self.ABILITY[i][0])/2),
                (incrementMatrix(stateKey(action['subject'], 'x'), 0), (1. - self.ABILITY[i][0])/2),
            ]})
        self.world.setDynamics(stateKey(action['subject'], 'x'), action, tree)
        tree = makeTree(incrementMatrix('turns', 1.0))
        self.world.setDynamics(stateKey(None, 'turns'), action, tree)
        # tree = makeTree(incrementMatrix(stateKey(action['subject'], 'energy'), -1.0))
        # self.world.setDynamics(stateKey(action['subject'], 'energy'), action, tree)

        # Rightmost boundary check
        dict = {}

        edl = []
        for index2 in range(self.E_ACTORS):
            if not dict:
                dict.update({'if': equalRow(stateKey(actor.name, 'x'), self.MAP_SIZE_X-1),
                         True: False,
                         False: {'if':differenceRow(stateKey(actor.name, 'x'),stateKey('Enemy' + str(index2), 'x'), -1),
                                 True: {'if': differenceRow(stateKey(actor.name, 'x'),stateKey('Enemy' + str(index2), 'x'), 0),
                                        True: {},
                                        False: {'if': equalFeatureRow(stateKey(actor.name, 'y'),stateKey('Enemy' + str(index2), 'y')),
                                                True: {'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                                    True: {},
                                                    False: False},
                                                False:{}}},
                                 False: {}}})
            else:
                newdict = {'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Enemy' + str(index2), 'x'), -1),
                           True: {
                               'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Enemy' + str(index2), 'x'), 0),
                               True: {},
                               False: {
                                   'if': equalFeatureRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y')),
                                   True: {'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                           True: {},
                                           False: False},
                                   False: {}}},
                           False: {}}
                edl.append(newdict)
        for index2 in range(self.F_ACTORS):
            if 'Actor'+str(index2) == actor.name:
                continue
            elif index2 != (self.F_ACTORS-1):
                if index2+1 == (self.F_ACTORS - 1) and 'Actor' + str(index2+1) == actor.name:
                    newdict = {'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Actor' + str(index2), 'x'), -1),
                               True: {
                                   'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Actor' + str(index2), 'x'),
                                                       0),
                                   True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                   False: {
                                       'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                             stateKey('Actor' + str(index2), 'y')),
                                       False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                       True: {'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                              True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                              False: False}}},
                               False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}
                    combined_dict = newdict
                    for dict2 in reversed(edl):
                        combined_dict = recursiveFillEmptyDicts(dict2, combined_dict)
                    dict = recursiveFillEmptyDicts(dict, combined_dict)
                else:
                    newdict = {'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Actor' + str(index2), 'x'), -1),
                               True: {
                                   'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Actor' + str(index2), 'x'), 0),
                                   True: {},
                                   False: {
                                       'if': equalFeatureRow(stateKey(actor.name, 'y'), stateKey('Actor' + str(index2), 'y')),
                                       True: {'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                               True: {},
                                               False: False},
                                       False: {}}},
                               False: {}}
                    edl.append(newdict)
            else:
                newdict = {'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Actor' + str(index2), 'x'), -1),
                           True: {
                               'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Actor' + str(index2), 'x'), 0),
                               True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                               False: {
                                   'if': equalFeatureRow(stateKey(actor.name, 'y'), stateKey('Actor' + str(index2), 'y')),
                                   True: {'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                           True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                           False: False},
                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}},
                           False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}
                combined_dict = newdict
                for dict2 in reversed(edl):
                    combined_dict = recursiveFillEmptyDicts(dict2, combined_dict)
                dict = recursiveFillEmptyDicts(dict, combined_dict)
        tree = makeTree(dict)
        actor.setLegal(action, tree)

        ##############################

        # Decrement X position
        action = actor.addAction({'verb': 'MoveLeft'})
        tree = makeTree({'distribution':[
                (incrementMatrix(stateKey(action['subject'], 'x'), -1), self.ABILITY[i][0]),
                (incrementMatrix(stateKey(action['subject'], 'x'), 1), (1. - self.ABILITY[i][0]) / 2),
                (incrementMatrix(stateKey(action['subject'], 'x'), 0), (1. - self.ABILITY[i][0]) / 2),
            ]})
        self.world.setDynamics(stateKey(action['subject'], 'x'), action, tree)
        tree = makeTree(incrementMatrix('turns', 1.0))
        self.world.setDynamics(stateKey(None, 'turns'), action, tree)
        # tree = makeTree(incrementMatrix(stateKey(action['subject'], 'energy'), -1.0))
        # self.world.setDynamics(stateKey(action['subject'], 'energy'), action, tree)

        # Leftmost boundary check, min X = 0
        dict = {}
        edl = []
        for index2 in range(self.E_ACTORS):
            if not dict:
                dict.update({'if': equalRow(stateKey(actor.name, 'x'), 0),
                             True: False,
                             False: {
                                 'if': differenceRow(stateKey('Enemy' + str(index2), 'x'), stateKey(actor.name, 'x'),
                                                     -1),
                                 True: {'if': differenceRow(stateKey('Enemy' + str(index2), 'x'), stateKey(actor.name, 'x'),
                                                             0),
                                        True: {},
                                        False: {'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                                    stateKey('Enemy' + str(index2), 'y')),
                                                False: {},
                                                True: {'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'),
                                                                              '0'),
                                                        True: {},
                                                        False: False}}},
                                 False: {}}})
            else:
                newdict = {'if': differenceRow(stateKey('Enemy' + str(index2), 'x'), stateKey(actor.name, 'x'),  -1),
                           True: {
                               'if': differenceRow(stateKey('Enemy' + str(index2), 'x'), stateKey(actor.name, 'x'), 0),
                               True: {},
                               False: {
                                   'if': equalFeatureRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y')),
                                   False: {},
                                   True: {'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                           True: {},
                                           False: False}}},
                           False: {}}
                edl.append(newdict)
        for index2 in range(self.F_ACTORS):
            if 'Actor' + str(index2) == actor.name:
                continue
            elif index2 != (self.F_ACTORS - 1):
                if index2+1 == (self.F_ACTORS - 1) and 'Actor' + str(index2+1) == actor.name:
                    newdict = {'if': differenceRow(stateKey('Actor' + str(index2), 'x'), stateKey(actor.name, 'x'), -1),
                               True: {
                                   'if': differenceRow(stateKey('Actor' + str(index2), 'x'), stateKey(actor.name, 'x'),
                                                       0),
                                   True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                   False: {
                                       'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                             stateKey('Actor' + str(index2), 'y')),
                                       False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                       True: {'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                              True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                              False: False}}},
                               False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}
                    combined_dict = newdict
                    for dict2 in reversed(edl):
                        combined_dict = recursiveFillEmptyDicts(dict2, combined_dict)
                    dict = recursiveFillEmptyDicts(dict, combined_dict)
                else:
                    newdict = {'if': differenceRow(stateKey('Actor' + str(index2), 'x'), stateKey(actor.name, 'x'), -1),
                               True: {
                                   'if': differenceRow(stateKey('Actor' + str(index2), 'x'), stateKey(actor.name, 'x'), 0),
                                   True: {},
                                   False: {
                                       'if': equalFeatureRow(stateKey(actor.name, 'y'), stateKey('Actor' + str(index2), 'y')),
                                       False: {},
                                       True: {'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                               True: {},
                                               False: False}}},
                               False: {}}
                    edl.append(newdict)
            else:
                newdict = {'if': differenceRow(stateKey('Actor' + str(index2), 'x'), stateKey(actor.name, 'x'), -1),
                           True: {
                               'if': differenceRow(stateKey('Actor' + str(index2), 'x'), stateKey(actor.name, 'x'), 0),
                               True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                               False: {
                                   'if': equalFeatureRow(stateKey(actor.name, 'y'), stateKey('Actor' + str(index2), 'y')),
                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                   True: {'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                           True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                           False: False}}},
                           False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}
                combined_dict = newdict
                for dict2 in reversed(edl):
                    combined_dict = recursiveFillEmptyDicts(dict2, combined_dict)
                dict = recursiveFillEmptyDicts(dict, combined_dict)
        tree = makeTree(dict)
        actor.setLegal(action, tree)

        ##############################

        # Increment Y position
        action = actor.addAction({'verb': 'MoveUp'})
        tree = makeTree({'distribution':[
                (incrementMatrix(stateKey(action['subject'], 'y'), 1), self.ABILITY[i][0]),
                (incrementMatrix(stateKey(action['subject'], 'y'), -1), (1. - self.ABILITY[i][0]) / 2),
                (incrementMatrix(stateKey(action['subject'], 'y'), 0), (1. - self.ABILITY[i][0]) / 2),
            ]})
        self.world.setDynamics(stateKey(action['subject'], 'y'), action, tree)
        tree = makeTree(incrementMatrix('turns', 1.0))
        self.world.setDynamics(stateKey(None, 'turns'), action, tree)
        # tree = makeTree(incrementMatrix(stateKey(action['subject'], 'energy'), -1.0))
        # self.world.setDynamics(stateKey(action['subject'], 'energy'), action, tree)

        # Downmost boundary check, max Y
        dict = {}
        edl = []
        for index2 in range(self.E_ACTORS):
            if not dict:
                dict.update({'if': equalRow(stateKey(actor.name, 'y'), str(self.MAP_SIZE_Y-1)),
                             True: False,
                             False: {
                                 'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y'),
                                                     -1),
                                 True: {'if': differenceRow(stateKey(actor.name, 'y'),
                                                            stateKey('Enemy' + str(index2), 'y'), 0),
                                        True: {},
                                        False: {'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                                    stateKey('Enemy' + str(index2), 'x')),
                                                False: {},
                                                True: {'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'),
                                                                              '0'),
                                                        True: {},
                                                        False: False}}},
                                 False: {}}})
            else:
                newdict = {'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y'), -1),
                           True: {
                               'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y'), 0),
                               True: {},
                               False: {
                                   'if': equalFeatureRow(stateKey(actor.name, 'x'), stateKey('Enemy' + str(index2), 'x')),
                                   False: {},
                                   True: {'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                           True: {},
                                           False: False}}},
                           False: {}}
                edl.append(newdict)
        for index2 in range(self.F_ACTORS):
            if 'Actor' + str(index2) == actor.name:
                continue
            elif index2 != (self.F_ACTORS - 1):
                if index2+1 == (self.F_ACTORS - 1) and 'Actor' + str(index2+1) == actor.name:
                    newdict = {'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Actor' + str(index2), 'y'), -1),
                               True: {
                                   'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Actor' + str(index2), 'y'),
                                                       0),
                                   True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                   False: {
                                       'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                             stateKey('Actor' + str(index2), 'x')),
                                       False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                       True: {'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                              True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                              False: False}}},
                               False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}
                    combined_dict = newdict
                    for dict2 in reversed(edl):
                        combined_dict = recursiveFillEmptyDicts(dict2, combined_dict)
                    dict = recursiveFillEmptyDicts(dict, combined_dict)
                else:
                    newdict = {'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Actor' + str(index2), 'y'), -1),
                               True: {
                                   'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Actor' + str(index2), 'y'), 0),
                                   True: {},
                                   False: {
                                       'if': equalFeatureRow(stateKey(actor.name, 'x'), stateKey('Actor' + str(index2), 'x')),
                                       False: {},
                                       True: {'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                               True: {},
                                               False: False}}},
                               False: {}}
                    
                    edl.append(newdict)
            else:
                
                newdict = {'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Actor' + str(index2), 'y'), -1),
                           True: {
                               'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Actor' + str(index2), 'y'), 0),
                               True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                               False: {
                                   'if': equalFeatureRow(stateKey(actor.name, 'x'), stateKey('Actor' + str(index2), 'x')),
                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                   True: {'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                           True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                           False: False}}},
                           False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}
                
                combined_dict = newdict
                for dict2 in reversed(edl):
                    combined_dict = recursiveFillEmptyDicts(dict2, combined_dict)
                dict = recursiveFillEmptyDicts(dict, combined_dict)
        tree = makeTree(dict)
        actor.setLegal(action, tree)

        ##############################

        # Decrement Y position
        action = actor.addAction({'verb': 'MoveDown'})
        tree = makeTree({'distribution':[
                (incrementMatrix(stateKey(action['subject'], 'y'), -1), self.ABILITY[i][0]),
                (incrementMatrix(stateKey(action['subject'], 'y'), 1), (1. - self.ABILITY[i][0]) / 2),
                (incrementMatrix(stateKey(action['subject'], 'y'), 0), (1. - self.ABILITY[i][0]) / 2),
            ]})
        self.world.setDynamics(stateKey(action['subject'], 'y'), action, tree)
        tree = makeTree(incrementMatrix('turns', 1.0))
        self.world.setDynamics(stateKey(None, 'turns'), action, tree)
        # tree = makeTree(incrementMatrix(stateKey(action['subject'], 'energy'), -1.0))
        # self.world.setDynamics(stateKey(action['subject'], 'energy'), action, tree)

        # Upmost boundary check, min Y = 0
        dict = {}
        edl = []
        for index2 in range(self.E_ACTORS):
            if not dict:
                dict.update({'if': equalRow(stateKey(actor.name, 'y'), '0'),
                             True: False,
                             False: {
                                 'if': differenceRow(stateKey('Enemy' + str(index2), 'y'), stateKey(actor.name, 'y'),
                                                     -1),
                                 True: {'if': differenceRow(stateKey('Enemy' + str(index2), 'y'), stateKey(actor.name, 'y'),
                                                             0),
                                        True: {},
                                        False: {'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                                    stateKey('Enemy' + str(index2), 'x')),
                                                False: {},
                                                True: {'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'),
                                                                              '0'),
                                                        True: {},
                                                        False: False}}},
                                 False: {}}})
            else:
                newdict = {'if': differenceRow(stateKey('Enemy' + str(index2), 'y'), stateKey(actor.name, 'y'), -1),
                           True: {
                               'if': differenceRow(stateKey('Enemy' + str(index2), 'y'), stateKey(actor.name, 'y'), 0),
                               True: {},
                               False: {
                                   'if': equalFeatureRow(stateKey(actor.name, 'x'), stateKey('Enemy' + str(index2), 'x')),
                                   False: {},
                                   True: {'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                           True: {},
                                           False: False}}},
                           False: {}}
                
                edl.append(newdict)
        for index2 in range(self.F_ACTORS):
            if 'Actor' + str(index2) == actor.name:
                continue
            elif index2 != (self.F_ACTORS - 1):
                if index2+1 == (self.F_ACTORS - 1) and 'Actor' + str(index2+1) == actor.name:
                    
                    newdict = {'if': differenceRow(stateKey('Actor' + str(index2), 'y'), stateKey(actor.name, 'y'), -1),
                               True: {
                                   'if': differenceRow(stateKey('Actor' + str(index2), 'y'), stateKey(actor.name, 'y'),
                                                       0),
                                   True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                   False: {
                                       'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                             stateKey('Actor' + str(index2), 'x')),
                                       False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                       True: {'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                              True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                              False: False}}},
                               False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}
                    
                    combined_dict = newdict
                    for dict2 in reversed(edl):
                        combined_dict = recursiveFillEmptyDicts(dict2, combined_dict)
                    dict = recursiveFillEmptyDicts(dict, combined_dict)
                else:
                    newdict = {'if': differenceRow(stateKey('Actor' + str(index2), 'y'), stateKey(actor.name, 'y'), -1),
                               True: {
                                   'if': differenceRow(stateKey('Actor' + str(index2), 'y'), stateKey(actor.name, 'y'), 0),
                                   True: {},
                                   False: {
                                       'if': equalFeatureRow(stateKey(actor.name, 'x'), stateKey('Actor' + str(index2), 'x')),
                                       False: {},
                                       True: {'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                               True: {},
                                               False: False}}},
                               False: {}}
                    
                    edl.append(newdict)
            else:
                
                newdict = {'if': differenceRow(stateKey('Actor' + str(index2), 'y'), stateKey(actor.name, 'y'),  -1),
                           True: {
                               'if': differenceRow(stateKey('Actor' + str(index2), 'y'), stateKey(actor.name, 'y'), 0),
                               True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                               False: {
                                   'if': equalFeatureRow(stateKey(actor.name, 'x'), stateKey('Actor' + str(index2), 'x')),
                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                   True: {'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                           True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                           False: False}}},
                           False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}
                
                combined_dict = newdict
                for dict2 in reversed(edl):
                    combined_dict = recursiveFillEmptyDicts(dict2, combined_dict)
                dict = recursiveFillEmptyDicts(dict, combined_dict)
        tree = makeTree(dict)
        actor.setLegal(action, tree)

        ##############################

        # Attack right direction
        action = actor.addAction({'verb': 'AttackRight'})
        tree = makeTree(incrementMatrix('turns', 1.0))
        self.world.setDynamics(stateKey(None, 'turns'), action, tree)
        for index2 in range(0, self.E_ACTORS):
            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y')),
                    True: {'if': differenceRow(stateKey(actor.name, 'x'),stateKey('Enemy' + str(index2), 'x'),-1),
	                    True: {'if': differenceRow(stateKey(actor.name, 'x'),stateKey('Enemy' + str(index2), 'x'),0),
		                    True: incrementMatrix(stateKey('Enemy' + str(index2),'x'), 0.0),
		                    False: {'if':equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                    True: incrementMatrix(stateKey('Enemy' + str(index2),'x'), 0.0),
                                    False: {'distribution':
                                                [(setToFeatureMatrix(stateKey('Enemy' + str(index2),'x'), stateKey('Enemy' + str(index2),'start_x')), self.ABILITY[i][1]),
                                                 (setToFeatureMatrix(stateKey('Enemy' + str(index2), 'x'),
                                                                     stateKey('Enemy' + str(index2), 'x')), 1. - self.ABILITY[i][1])
                                                 ]}
                                    }},
	                    False: incrementMatrix(stateKey('Enemy' + str(index2),'x'), 0.0)},
                    False: incrementMatrix(stateKey('Enemy' + str(index2),'x'), 0.0)})
            self.world.setDynamics(stateKey('Enemy' + str(index2), 'x'), action, tree)

            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y')),
                             True: {'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Enemy' + str(index2), 'x'),
                                                        -1),
                                    True: {'if': differenceRow(stateKey(actor.name, 'x'),
                                                               stateKey('Enemy' + str(index2), 'x'), 0),
                                           True: incrementMatrix(stateKey('Enemy' + str(index2), 'y'), 0.0),
                                           False: {
                                               'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                               True: incrementMatrix(stateKey('Enemy' + str(index2), 'y'), 0.0),
                                               False: {'distribution':
                                                           [(setToFeatureMatrix(stateKey('Enemy' + str(index2), 'y'),
                                                                                stateKey('Enemy' + str(index2),
                                                                                         'start_y')),
                                                             self.ABILITY[i][1]),
                                                            (setToFeatureMatrix(stateKey('Enemy' + str(index2), 'y'),
                                                                                stateKey('Enemy' + str(index2), 'y')),
                                                             1. - self.ABILITY[i][1])
                                                            ]}
                                               }},
                                    False: incrementMatrix(stateKey('Enemy' + str(index2), 'y'), 0.0)},
                             False: incrementMatrix(stateKey('Enemy' + str(index2), 'y'), 0.0)})
            self.world.setDynamics(stateKey('Enemy' + str(index2), 'y'), action, tree)


            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y')),
                             True: {'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Enemy' + str(index2), 'x'),
                                                        -1),
                                    True: {'if': differenceRow(stateKey(actor.name, 'x'),
                                                               stateKey('Enemy' + str(index2), 'x'), 0),
                                           True: incrementMatrix(stateKey(actor.name, 'defenses'), 0.0),
                                           False: {
                                               'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                               True: incrementMatrix(stateKey(actor.name, 'defenses'), 0.0),
                                               False: {
                                                   "if": differenceRow(stateKey('Enemy' + str(index2), 'dist_start'), stateKey(None, 'zero'),self.DEFENSE_LIMIT),
                                                    True:incrementMatrix(stateKey(actor.name, 'defenses'), 1.0),
                                                    False: incrementMatrix(stateKey(actor.name, 'defenses'), 0.0)
                                                    }
                                                }
                                            },
                                    False: incrementMatrix(stateKey(actor.name, 'defenses'), 0.0)},
                             False: incrementMatrix(stateKey(actor.name, 'defenses'), 0.0)})
            self.world.setDynamics(stateKey(actor.name, 'defenses'), action, tree)
        actor.setLegal(action, makeTree({'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}))

        ##############################

        # Attack left direction
        action = actor.addAction({'verb': 'AttackLeft'})
        tree = makeTree(incrementMatrix('turns', 1.0))
        self.world.setDynamics(stateKey(None, 'turns'), action, tree)
        for index2 in range(0, self.E_ACTORS):
            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y')),
                    True: {'if': differenceRow(stateKey(actor.name, 'x'),stateKey('Enemy' + str(index2), 'x'), 0),
	                    True: {'if': differenceRow(stateKey(actor.name, 'x'),stateKey('Enemy' + str(index2), 'x'),1),
		                    True: incrementMatrix(stateKey('Enemy' + str(index2),'x'), 0.0),
		                    False: {'if':equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                    True: incrementMatrix(stateKey('Enemy' + str(index2),'x'), 0.0),
                                    False: {'distribution':
                                                [(setToFeatureMatrix(stateKey('Enemy' + str(index2),'x'), stateKey('Enemy' + str(index2),'start_x')), self.ABILITY[i][1]),
                                                 (setToFeatureMatrix(stateKey('Enemy' + str(index2), 'x'),
                                                                     stateKey('Enemy' + str(index2), 'x')), 1. - self.ABILITY[i][1])
                                                 ]}
                                    }},
	                    False: incrementMatrix(stateKey('Enemy' + str(index2),'x'), 0.0)},
                    False: incrementMatrix(stateKey('Enemy' + str(index2),'x'), 0.0)})
            self.world.setDynamics(stateKey('Enemy' + str(index2), 'x'), action, tree)

            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y')),
                             True: {'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Enemy' + str(index2), 'x'),
                                                        0),
                                    True: {'if': differenceRow(stateKey(actor.name, 'x'),
                                                               stateKey('Enemy' + str(index2), 'x'), 1),
                                           True: incrementMatrix(stateKey('Enemy' + str(index2), 'y'), 0.0),
                                           False: {
                                               'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                               True: incrementMatrix(stateKey('Enemy' + str(index2), 'y'), 0.0),
                                               False: {'distribution':
                                                           [(setToFeatureMatrix(stateKey('Enemy' + str(index2), 'y'),
                                                                                stateKey('Enemy' + str(index2),
                                                                                         'start_y')),
                                                             self.ABILITY[i][1]),
                                                            (setToFeatureMatrix(stateKey('Enemy' + str(index2), 'y'),
                                                                                stateKey('Enemy' + str(index2), 'y')),
                                                             1. - self.ABILITY[i][1])
                                                            ]}
                                               }},
                                    False: incrementMatrix(stateKey('Enemy' + str(index2), 'y'), 0.0)},
                             False: incrementMatrix(stateKey('Enemy' + str(index2), 'y'), 0.0)})
            self.world.setDynamics(stateKey('Enemy' + str(index2), 'y'), action, tree)

            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y')),
                             True: {'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Enemy' + str(index2), 'x'),
                                                        0),
                                    True: {'if': differenceRow(stateKey(actor.name, 'x'),
                                                               stateKey('Enemy' + str(index2), 'x'), 1),
                                           True: incrementMatrix(stateKey(actor.name, 'defenses'), 0.0),
                                           False: {
                                               'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                               True: incrementMatrix(stateKey(actor.name, 'defenses'), 0.0),
                                               False: {
                                                   "if": differenceRow(stateKey('Enemy' + str(index2), 'dist_start'),
                                                                       stateKey(None, 'zero'), self.DEFENSE_LIMIT),
                                                   True: incrementMatrix(stateKey(actor.name, 'defenses'), 1.0),
                                                   False: incrementMatrix(stateKey(actor.name, 'defenses'), 0.0)
                                               }
                                           }
                                           },
                                    False: incrementMatrix(stateKey(actor.name, 'defenses'), 0.0)},
                             False: incrementMatrix(stateKey(actor.name, 'defenses'), 0.0)})
            self.world.setDynamics(stateKey(actor.name, 'defenses'), action, tree)
        actor.setLegal(action, makeTree({'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}))

        ##############################

        # Attack up direction
        action = actor.addAction({'verb': 'AttackUp'})
        tree = makeTree(incrementMatrix('turns', 1.0))
        self.world.setDynamics(stateKey(None, 'turns'), action, tree)
        for index2 in range(0, self.E_ACTORS):
            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'x'), stateKey('Enemy' + str(index2), 'x')),
                    True: {'if': differenceRow(stateKey(actor.name, 'y'),stateKey('Enemy' + str(index2), 'y'),-1),
	                    True: {'if': differenceRow(stateKey(actor.name, 'y'),stateKey('Enemy' + str(index2), 'y'),0),
		                    True: incrementMatrix(stateKey('Enemy' + str(index2),'x'), 0.0),
		                    False: {'if':equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                    True: incrementMatrix(stateKey('Enemy' + str(index2),'x'), 0.0),
                                    False: {'distribution':
                                                [(setToFeatureMatrix(stateKey('Enemy' + str(index2),'x'), stateKey('Enemy' + str(index2),'start_x')), self.ABILITY[i][1]),
                                                 (setToFeatureMatrix(stateKey('Enemy' + str(index2), 'x'),
                                                                     stateKey('Enemy' + str(index2), 'x')), 1. - self.ABILITY[i][1])
                                                 ]}
                                    }},
	                    False: incrementMatrix(stateKey('Enemy' + str(index2),'x'), 0.0)},
                    False: incrementMatrix(stateKey('Enemy' + str(index2),'x'), 0.0)})
            self.world.setDynamics(stateKey('Enemy' + str(index2), 'x'), action, tree)

            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'x'), stateKey('Enemy' + str(index2), 'x')),
                             True: {'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y'),
                                                        -1),
                                    True: {'if': differenceRow(stateKey(actor.name, 'y'),
                                                               stateKey('Enemy' + str(index2), 'y'), 0),
                                           True: incrementMatrix(stateKey('Enemy' + str(index2), 'y'), 0.0),
                                           False: {
                                               'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                               True: incrementMatrix(stateKey('Enemy' + str(index2), 'y'), 0.0),
                                               False: {'distribution':
                                                           [(setToFeatureMatrix(stateKey('Enemy' + str(index2), 'y'),
                                                                                stateKey('Enemy' + str(index2),
                                                                                         'start_y')),
                                                             self.ABILITY[i][1]),
                                                            (setToFeatureMatrix(stateKey('Enemy' + str(index2), 'y'),
                                                                                stateKey('Enemy' + str(index2), 'y')),
                                                             1. - self.ABILITY[i][1])
                                                            ]}
                                               }},
                                    False: incrementMatrix(stateKey('Enemy' + str(index2), 'y'), 0.0)},
                             False: incrementMatrix(stateKey('Enemy' + str(index2), 'y'), 0.0)})
            self.world.setDynamics(stateKey('Enemy' + str(index2), 'y'), action, tree)

            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'x'), stateKey('Enemy' + str(index2), 'x')),
                             True: {'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y'),
                                                        -1),
                                    True: {'if': differenceRow(stateKey(actor.name, 'y'),
                                                               stateKey('Enemy' + str(index2), 'y'), 0),
                                           True: incrementMatrix(stateKey(actor.name, 'defenses'), 0.0),
                                           False: {
                                               'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                               True: incrementMatrix(stateKey(actor.name, 'defenses'), 0.0),
                                               False: {
                                                   "if": differenceRow(stateKey('Enemy' + str(index2), 'dist_start'),
                                                                       stateKey(None, 'zero'), self.DEFENSE_LIMIT),
                                                   True: incrementMatrix(stateKey(actor.name, 'defenses'), 1.0),
                                                   False: incrementMatrix(stateKey(actor.name, 'defenses'), 0.0)
                                               }
                                           }
                                           },
                                    False: incrementMatrix(stateKey(actor.name, 'defenses'), 0.0)},
                             False: incrementMatrix(stateKey(actor.name, 'defenses'), 0.0)})
            self.world.setDynamics(stateKey(actor.name, 'defenses'), action, tree)
        actor.setLegal(action, makeTree({'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}))

        ##############################

        # Attack down direction
        action = actor.addAction({'verb': 'AttackDown'})
        tree = makeTree(incrementMatrix('turns', 1.0))
        self.world.setDynamics(stateKey(None, 'turns'), action, tree)
        for index2 in range(0, self.E_ACTORS):
            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'x'), stateKey('Enemy' + str(index2), 'x')),
                    True: {'if': differenceRow(stateKey(actor.name, 'y'),stateKey('Enemy' + str(index2), 'y'),0),
	                    True: {'if': differenceRow(stateKey(actor.name, 'y'),stateKey('Enemy' + str(index2), 'y'),1),
		                    True: incrementMatrix(stateKey('Enemy' + str(index2),'x'), 0.0),
		                    False: {'if':equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                    True: incrementMatrix(stateKey('Enemy' + str(index2),'x'), 0.0),
                                    False: {'distribution':
                                                [(setToFeatureMatrix(stateKey('Enemy' + str(index2),'x'), stateKey('Enemy' + str(index2),'start_x')), self.ABILITY[i][1]),
                                                 (setToFeatureMatrix(stateKey('Enemy' + str(index2), 'x'),
                                                                     stateKey('Enemy' + str(index2), 'x')), 1. - self.ABILITY[i][1])
                                                 ]}
                                    }},
	                    False: incrementMatrix(stateKey('Enemy' + str(index2),'x'), 0.0)},
                    False: incrementMatrix(stateKey('Enemy' + str(index2),'x'), 0.0)})
            self.world.setDynamics(stateKey('Enemy' + str(index2), 'x'), action, tree)

            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'x'), stateKey('Enemy' + str(index2), 'x')),
                             True: {'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y'),
                                                        0),
                                    True: {'if': differenceRow(stateKey(actor.name, 'y'),
                                                               stateKey('Enemy' + str(index2), 'y'), 1),
                                           True: incrementMatrix(stateKey('Enemy' + str(index2), 'y'), 0.0),
                                           False: {
                                               'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                               True: incrementMatrix(stateKey('Enemy' + str(index2), 'y'), 0.0),
                                               False: {'distribution':
                                                           [(setToFeatureMatrix(stateKey('Enemy' + str(index2), 'y'),
                                                                                stateKey('Enemy' + str(index2),
                                                                                         'start_y')),
                                                             self.ABILITY[i][1]),
                                                            (setToFeatureMatrix(stateKey('Enemy' + str(index2), 'y'),
                                                                                stateKey('Enemy' + str(index2), 'y')),
                                                             1. - self.ABILITY[i][1])
                                                            ]}
                                               }},
                                    False: incrementMatrix(stateKey('Enemy' + str(index2), 'y'), 0.0)},
                             False: incrementMatrix(stateKey('Enemy' + str(index2), 'y'), 0.0)})
            self.world.setDynamics(stateKey('Enemy' + str(index2), 'y'), action, tree)

            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'x'), stateKey('Enemy' + str(index2), 'x')),
                             True: {'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y'),
                                                        0),
                                    True: {'if': differenceRow(stateKey(actor.name, 'y'),
                                                               stateKey('Enemy' + str(index2), 'y'), 1),
                                           True: incrementMatrix(stateKey(actor.name, 'defenses'), 0.0),
                                           False: {
                                               'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                               True: incrementMatrix(stateKey(actor.name, 'defenses'), 0.0),
                                               False: {
                                                   "if": differenceRow(stateKey('Enemy' + str(index2), 'dist_start'),
                                                                       stateKey(None, 'zero'), self.DEFENSE_LIMIT),
                                                   True: incrementMatrix(stateKey(actor.name, 'defenses'), 1.0),
                                                   False: incrementMatrix(stateKey(actor.name, 'defenses'), 0.0)
                                               }
                                           }
                                           },
                                    False: incrementMatrix(stateKey(actor.name, 'defenses'), 0.0)},
                             False: incrementMatrix(stateKey(actor.name, 'defenses'), 0.0)})
            self.world.setDynamics(stateKey(actor.name, 'defenses'), action, tree)
        actor.setLegal(action, makeTree({'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}))

    # def set_friendly_models(self, actor, index):
    #     for index2 in range(0, self.E_ACTORS):
    #         enemy = 'Enemy' + str(index2)
    #         start = self.E_START_LOC[index2]
    #         start_x = int(start.split(",")[0])
    #         start_y = int(start.split(",")[1])
    #         # actor.setBelief(stateKey(enemy, 'health'),3,True)
    #         # actor.setBelief(stateKey(enemy, 'x'), start_x, True)
    #         # actor.setBelief(stateKey(enemy, 'y'), start_y, True)
    #
    #         tree = makeTree({'if': differenceRow(stateKey(actor.name, 'x'),stateKey(enemy, 'x'), -2),
    #                          True: {'if': differenceRow(stateKey(actor.name, 'x'),stateKey(enemy, 'x'), 2),
    #                                 True: {'distribution': self.calcPosRadius(enemy,'x')},
    #                                 False: {'if': differenceRow(stateKey(actor.name, 'y'),stateKey(enemy, 'y'), -2),
    #                                     True: {'if': differenceRow(stateKey(actor.name, 'y'),stateKey(enemy, 'y'), 2),
    #                                         True: {'distribution': self.calcPosRadius(enemy,'x')},
    #                                         False: {'distribution': [(KeyedVector({stateKey(enemy,'x'): self.world.getState(enemy, 'x').domain()[0]}), 1.0)]}
    #                                            },
    #                                     False: {'distribution': self.calcPosRadius(enemy,'x')}
    #                                         }
    #                                 },
    #                          False: {'distribution': self.calcPosRadius(enemy,'x')}
    #                          })
    #
    #         actor.defineObservation(stateKey(enemy, 'x'), tree)
    #
    #         tree = makeTree({'if': differenceRow(stateKey(actor.name, 'x'),stateKey(enemy, 'x'), -2),
    #                          True: {'if': differenceRow(stateKey(actor.name, 'x'),stateKey(enemy, 'x'), 2),
    #                                 True: {'distribution': self.calcPosRadius(enemy,'y')},
    #                                 False: {'if': differenceRow(stateKey(actor.name, 'y'),stateKey(enemy, 'y'), -2),
    #                                     True: {'if': differenceRow(stateKey(actor.name, 'y'),stateKey(enemy, 'y'), 2),
    #                                         True: {'distribution': self.calcPosRadius(enemy,'y')},
    #                                         False: {'distribution': [(KeyedVector({stateKey(enemy,'y'): self.world.getState(enemy, 'y').domain()[0]}), 1.0)]}
    #                                            },
    #                                     False: {'distribution': self.calcPosRadius(enemy,'y')}
    #                                         }
    #                                 },
    #                          False: {'distribution': self.calcPosRadius(enemy,'y')}
    #                          })
    #
    #         actor.defineObservation(stateKey(enemy, 'y'), tree)
    #
    #     for index2 in [i for i in range(0, self.F_ACTORS) if i != index]:
    #         ally = 'Actor' + str(index2)
    #         start = self.F_START_LOC[index2]
    #         start_x = int(start.split(",")[0])
    #         start_y = int(start.split(",")[1])
    #         # actor.setBelief(stateKey(ally, 'health'), 3, True)
    #         # actor.setBelief(stateKey(ally, 'x'), Distribution({start_x:1.0}), True)
    #         # actor.setBelief(stateKey(ally, 'y'), Distribution({start_y:1.0}), True)
    #
    #         tree = makeTree({'if': differenceRow(stateKey(actor.name, 'x'), stateKey(ally, 'x'), -1),
    #                          True: {'if': differenceRow(stateKey(actor.name, 'x'), stateKey(ally, 'x'), 1),
    #                                 True: {'distribution': self.calcPosRadius(ally, 'x')},
    #                                 False: {
    #                                     'if': differenceRow(stateKey(actor.name, 'y'), stateKey(ally, 'y'), -1),
    #                                     True: {
    #                                         'if': differenceRow(stateKey(actor.name, 'y'), stateKey(ally, 'y'), 1),
    #                                         True: {'distribution': self.calcPosRadius(ally, 'x')},
    #                                         False: {'distribution': [(KeyedVector({stateKey(ally, 'x'):
    #                                                                                    self.world.getState(ally,
    #                                                                                                        'x').domain()[
    #                                                                                        0]}), 1.0)]}
    #                                         },
    #                                     False: {'distribution': self.calcPosRadius(ally, 'x')}
    #                                     }
    #                                 },
    #                          False: {'distribution': self.calcPosRadius(ally, 'x')}
    #                          })
    #
    #         actor.defineObservation(stateKey(ally, 'x'), tree)
    #
    #         tree = makeTree({'if': differenceRow(stateKey(actor.name, 'x'), stateKey(ally, 'x'), -1),
    #                          True: {'if': differenceRow(stateKey(actor.name, 'x'), stateKey(ally, 'x'), 1),
    #                                 True: {'distribution': self.calcPosRadius(ally, 'y')},
    #                                 False: {
    #                                     'if': differenceRow(stateKey(actor.name, 'y'), stateKey(ally, 'y'), -1),
    #                                     True: {
    #                                         'if': differenceRow(stateKey(actor.name, 'y'), stateKey(ally, 'y'), 1),
    #                                         True: {'distribution': self.calcPosRadius(ally, 'y')},
    #                                         False: {'distribution': [(KeyedVector({stateKey(ally, 'y'):
    #                                                                                    self.world.getState(ally,
    #                                                                                                        'y').domain()[
    #                                                                                        0]}), 1.0)]}
    #                                         },
    #                                     False: {'distribution': self.calcPosRadius(ally, 'y')}
    #                                     }
    #                                 },
    #                          False: {'distribution': self.calcPosRadius(ally, 'y')}
    #                          })
    #
    #         actor.defineObservation(stateKey(ally, 'y'), tree)


    def create_distract_agents(self):
        for index in range(0, self.D_ACTORS):
            actor = Agent('Distractor' + str(index))
            self.world.addAgent(actor)
            actor.setHorizon(5)

            # Agent is not allowed to move if not deployed by the base
            self.world.defineState(actor.name, 'deployed', bool)
            self.world.setState(actor.name, 'deployed', False)

            # Every time the agent makes an action, there is a cost associated
            self.world.defineState(actor.name, 'cost', int)
            self.world.setState(actor.name, 'cost', 0)

            # Set agent's starting location
            self.world.defineState(actor.name, 'x', int)
            self.world.setState(actor.name, 'x', 0)

            self.world.defineState(actor.name, 'y', int)
            self.world.setState(actor.name, 'y', 0)

            # Positive reward for luring enemy away from Agents
            actor.setReward(
                minimizeDifference(stateKey('Actor' + str(index), 'x'), stateKey('Enemy' + str(index), 'x')),
                self.DISTRACTOR[0])
            actor.setReward(
                minimizeDifference(stateKey('Actor' + str(index), 'y'), stateKey('Enemy' + str(index), 'y')),
                self.DISTRACTOR[0])

            # Positive reward for moving closer to enemy
            actor.setReward(
                minimizeDifference(stateKey('Distractor' + str(index), 'x'), stateKey('Enemy' + str(index), 'x')),
                self.DISTRACTOR[1])
            actor.setReward(
                minimizeDifference(stateKey('Distractor' + str(index), 'y'), stateKey('Enemy' + str(index), 'y')),
                self.DISTRACTOR[1])

            self.set_distract_actions(actor)

    def set_distract_actions(self, actor):
        # Nop
        action = actor.addAction({'verb': 'Wait'})
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'x'), 0.))
        self.world.setDynamics(stateKey(action['subject'], 'x'), action, tree)
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'y'), 0.))
        self.world.setDynamics(stateKey(action['subject'], 'y'), action, tree)
        # Reward for not moving
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'cost'), -1.))
        self.world.setDynamics(stateKey(action['subject'], 'cost'), action, tree)

        # Increment X position
        action = actor.addAction({'verb': 'MoveRight'})
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'x'), 1.))
        self.world.setDynamics(stateKey(action['subject'], 'x'), action, tree)

        # Cost for moving
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'cost'), 1.))
        self.world.setDynamics(stateKey(action['subject'], 'cost'), action, tree)

        # Rightmost boundary check
        tree = makeTree({'if': equalRow(stateKey(actor.name, 'deployed'), True),
                         True: {'if': equalRow(stateKey(actor.name, 'x'), str(self.MAP_SIZE_X)),
                                True: False, False: True}, False: False})
        actor.setLegal(action, tree)

        ##############################

        # Decrement X position
        action = actor.addAction({'verb': 'MoveLeft'})
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'x'), -1.))
        self.world.setDynamics(stateKey(action['subject'], 'x'), action, tree)

        # Cost for moving
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'cost'), 1.))
        self.world.setDynamics(stateKey(action['subject'], 'cost'), action, tree)

        # Leftmost boundary check, min X = 0
        tree = makeTree({'if': equalRow(stateKey(actor.name, 'deployed'), True),
                         True: {'if': equalRow(stateKey(actor.name, 'x'), 0),
                                True: False, False: True}, False: False})
        actor.setLegal(action, tree)

        ##############################

        # Increment Y position
        action = actor.addAction({'verb': 'MoveUp'})
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'y'), 1.))
        self.world.setDynamics(stateKey(action['subject'], 'y'), action, tree)

        # Cost for moving
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'cost'), 1.))
        self.world.setDynamics(stateKey(action['subject'], 'cost'), action, tree)

        # Downmost boundary check, max Y
        tree = makeTree({'if': equalRow(stateKey(actor.name, 'deployed'), True),
                         True: {'if': equalRow(stateKey(actor.name, 'y'), str(self.MAP_SIZE_Y)),
                                True: False, False: True}, False: False})
        actor.setLegal(action, tree)

        ##############################

        # Decrement Y position
        action = actor.addAction({'verb': 'MoveDown'})
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'y'), -1.))
        self.world.setDynamics(stateKey(action['subject'], 'y'), action, tree)

        # Cost for moving
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'cost'), 1.))
        self.world.setDynamics(stateKey(action['subject'], 'cost'), action, tree)

        # Upmost boundary check, min Y = 0
        tree = makeTree({'if': equalRow(stateKey(actor.name, 'deployed'), True),
                         True: {'if': equalRow(stateKey(actor.name, 'Y'), 0),
                                True: False, False: True}, False: False})
        actor.setLegal(action, tree)

    # def create_supply_agents(self):
    #     for index in range(0, self.S_ACTORS):
    #         actor = Agent('Supplier' + str(index))
    #         self.world.addAgent(actor)
    #         actor.setHorizon(5)
    #
    #         # Agent is not allowed to move if not deployed by the base
    #         self.world.defineState(actor.name, 'deployed', bool)
    #         self.world.setState(actor.name, 'deployed', True)
    #
    #         # Every time the agent makes an action, there is a cost associated
    #         self.world.defineState(actor.name, 'cost', int)
    #         self.world.setState(actor.name, 'cost', 0)
    #
    #         # Set agent's starting location
    #         self.world.defineState(actor.name, 'x', int)
    #         self.world.setState(actor.name, 'x', 0)
    #
    #         self.world.defineState(actor.name, 'y', int)
    #         self.world.setState(actor.name, 'y', 0)
    #
    #         self.world.defineState(actor.name, 'energy', float)
    #         self.world.setState(actor.name, 'energy', self.s_get_start_energy(index))
    #
    #         # Positive reward for supplying friendly agents with energy
    #         actor.setReward(
    #             minimizeDifference(stateKey('Supplier' + str(index), 'x'), stateKey('Actor' + str(index), 'x')),
    #             self.SUPPLIER[0])
    #         actor.setReward(
    #             minimizeDifference(stateKey('Supplier' + str(index), 'y'), stateKey('Actor' + str(index), 'y')),
    #             self.SUPPLIER[0])
    #
    #         # Positive reward for supplying friendly agents with energy
    #         actor.setReward(
    #             maximizeFeature(stateKey('Actor' + str(index), 'energy')),
    #             self.SUPPLIER[1])
    #
    #         self.set_supply_actions(actor)

    # def set_supply_actions(self, actor):
    #     # Nop
    #     action = actor.addAction({'verb': 'Wait'})
    #     tree = makeTree(incrementMatrix(stateKey(action['subject'], 'x'), 0.))
    #     self.world.setDynamics(stateKey(action['subject'], 'x'), action, tree)
    #     tree = makeTree(incrementMatrix(stateKey(action['subject'], 'y'), 0.))
    #     self.world.setDynamics(stateKey(action['subject'], 'y'), action, tree)
    #     # Reward for not moving
    #     tree = makeTree(incrementMatrix(stateKey(action['subject'], 'cost'), -1.))
    #     self.world.setDynamics(stateKey(action['subject'], 'cost'), action, tree)
    #
    #     tree = makeTree(incrementMatrix(stateKey(action['subject'], 'energy'), 2.))
    #     self.world.setDynamics(stateKey(action['subject'], 'energy'), action, tree)
    #
    #     # Increment X position
    #     action = actor.addAction({'verb': 'MoveRight'})
    #     tree = makeTree(incrementMatrix(stateKey(action['subject'], 'x'), 1.))
    #     self.world.setDynamics(stateKey(action['subject'], 'x'), action, tree)
    #
    #     # Cost for moving
    #     tree = makeTree(incrementMatrix(stateKey(action['subject'], 'cost'), 1.))
    #     self.world.setDynamics(stateKey(action['subject'], 'cost'), action, tree)
    #
    #     # Rightmost boundary check
    #     tree = makeTree({'if': equalRow(stateKey(actor.name, 'deployed'), True),
    #                      True: {'if': equalRow(stateKey(actor.name, 'x'), str(self.MAP_SIZE_X)),
    #                             True: False, False: True}, False: False})
    #     actor.setLegal(action, tree)
    #
    #     ##############################
    #
    #     # Decrement X position
    #     action = actor.addAction({'verb': 'MoveLeft'})
    #     tree = makeTree(incrementMatrix(stateKey(action['subject'], 'x'), -1.))
    #     self.world.setDynamics(stateKey(action['subject'], 'x'), action, tree)
    #
    #     # Cost for moving
    #     tree = makeTree(incrementMatrix(stateKey(action['subject'], 'cost'), 1.))
    #     self.world.setDynamics(stateKey(action['subject'], 'cost'), action, tree)
    #
    #     # Leftmost boundary check, min X = 0
    #     tree = makeTree({'if': equalRow(stateKey(actor.name, 'deployed'), True),
    #                      True: {'if': equalRow(stateKey(actor.name, 'x'), 0),
    #                             True: False, False: True}, False: False})
    #     actor.setLegal(action, tree)
    #
    #     ##############################
    #
    #     # Increment Y position
    #     action = actor.addAction({'verb': 'MoveUp'})
    #     tree = makeTree(incrementMatrix(stateKey(action['subject'], 'y'), 1.))
    #     self.world.setDynamics(stateKey(action['subject'], 'y'), action, tree)
    #
    #     # Cost for moving
    #     tree = makeTree(incrementMatrix(stateKey(action['subject'], 'cost'), 1.))
    #     self.world.setDynamics(stateKey(action['subject'], 'cost'), action, tree)
    #
    #     # Downmost boundary check, max Y
    #     tree = makeTree({'if': equalRow(stateKey(actor.name, 'deployed'), True),
    #                      True: {'if': equalRow(stateKey(actor.name, 'y'), str(self.MAP_SIZE_Y)),
    #                             True: False, False: True}, False: False})
    #     actor.setLegal(action, tree)
    #
    #     ##############################
    #
    #     # Decrement Y position
    #     action = actor.addAction({'verb': 'MoveDown'})
    #     tree = makeTree(incrementMatrix(stateKey(action['subject'], 'y'), -1.))
    #     self.world.setDynamics(stateKey(action['subject'], 'y'), action, tree)
    #
    #     # Cost for moving
    #     tree = makeTree(incrementMatrix(stateKey(action['subject'], 'cost'), 1.))
    #     self.world.setDynamics(stateKey(action['subject'], 'cost'), action, tree)
    #
    #     # Upmost boundary check, min Y = 0
    #     tree = makeTree({'if': equalRow(stateKey(actor.name, 'deployed'), True),
    #                      True: {'if': equalRow(stateKey(actor.name, 'y'), 0),
    #                             True: False, False: True}, False: False})
    #     actor.setLegal(action, tree)
    #
    #     # Supply Allies
    #     action = actor.addAction({'verb': 'Supply'})
    #     # Effect on actor's energy
    #     tree = makeTree(incrementMatrix(stateKey('Actor' + str(0), 'energy'), 2.0/(self.calculateDistance(
    #                                                                                     self.f_get_current_x(Agent('Actor' + str(0))),
    #                                                                                     self.f_get_current_y(Agent('Actor' + str(0))),
    #                                                                                     self.s_get_current_x(actor),
    #                                                                                     self.s_get_current_y(actor))+1)))
    #     self.world.setDynamics(stateKey('Actor' + str(0), 'energy'), action, tree)
    #
    #     # Cost for supplying
    #     tree = makeTree(incrementMatrix(stateKey(action['subject'], 'cost'), 1.))
    #     self.world.setDynamics(stateKey(action['subject'], 'cost'), action, tree)
    #
    #     tree = makeTree(incrementMatrix(stateKey(action['subject'], 'energy'), -2.))
    #     self.world.setDynamics(stateKey(action['subject'], 'energy'), action, tree)
    #
    #     # Upmost boundary check, min Energy = 0
    #     tree = makeTree({'if': equalRow(stateKey(actor.name, 'deployed'), True),
    #                      True: {'if': equalRow(stateKey(actor.name, 'energy'), 0),
    #                             True: False, False: True}, False: False})
    #     actor.setLegal(action, tree)

    def create_enemy_agents(self):
        for index in range(0, self.E_ACTORS):
            actor = Agent('Enemy' + str(index))
            self.enemy_agents.append(actor)
            self.world.addAgent(actor)
            actor.setHorizon(5)

            # Set agent's starting location
            self.world.defineState(actor.name, 'x', int)
            self.world.setState(actor.name, 'x', self.e_get_start_x(index))
            self.world.defineState(actor.name, 'start_x', int)
            self.world.setState(actor.name, 'start_x', self.e_get_start_x(index))
            self.world.defineState(actor.name, 'goal_x', int)
            self.world.setState(actor.name, 'goal_x', self.e_get_goal_x(index))

            self.world.defineState(actor.name, 'y', int)
            self.world.setState(actor.name, 'y', self.e_get_start_y(index))
            self.world.defineState(actor.name, 'start_y', int)
            self.world.setState(actor.name, 'start_y', self.e_get_start_y(index))
            self.world.defineState(actor.name, 'goal_y', int)
            self.world.setState(actor.name, 'goal_y', self.e_get_goal_y(index))

            self.world.defineState(actor.name,'dist_start', float)
            self.world.setState(actor.name, 'dist_start', 1.0)

            self.world.defineState(actor.name, 'health', int)
            self.world.setState(actor.name, 'health', 3)

            # enemy = 'Actor' + str(index)
            actor.setReward(minimizeDifference(stateKey(actor.name, 'x'), stateKey(actor.name, 'goal_x')), self.ENEMY[0])
            actor.setReward(minimizeDifference(stateKey(actor.name, 'y'), stateKey(actor.name, 'goal_y')), self.ENEMY[0])

            # actor.setReward(minimizeDifference(stateKey(actor.name, 'x'), stateKey('Distractor' + str(index), 'x')),
            #                 self.ENEMY[1])
            # actor.setReward(minimizeDifference(stateKey(actor.name, 'y'), stateKey('Distractor' + str(index), 'y')),
            #                 self.ENEMY[1])


            # Reward for attacking enemy
            # dict = {}
            # edl = []
            for index2 in range(0, self.F_ACTORS):
                enemy = 'Actor' + str(index2)

                # actor.setReward(minimizeFeature(stateKey(enemy, 'health')), self.ENEMY[0])
                actor.setReward(minimizeDifference(stateKey(enemy, 'x'), stateKey(enemy, 'goal_x')),
                                -1*self.ENEMY[1])
                actor.setReward(minimizeDifference(stateKey(enemy, 'y'), stateKey(enemy, 'goal_y')),
                                -1*self.ENEMY[1])
                # if index == 0:
                #     dict = {'if': equalFeatureRow(stateKey('Actor' + str(index), 'health'), '0'),
                #      True: {}, False: False}
                # elif index != (self.F_ACTORS - 1):
                #     newdict = {'if': equalFeatureRow(stateKey('Actor' + str(index), 'health'), '0'),
                #      True: {}, False: False}
                #     edl.append(newdict)
                # else:
                #     newdict = {'if': equalFeatureRow(stateKey('Actor' + str(index), 'health'), '0'),
                #                True: True, False: False}
                #     combined_dict = newdict
                #     for dict2 in reversed(edl):
                #         combined_dict = recursiveFillEmptyDicts(dict2, combined_dict)
                #     dict = recursiveFillEmptyDicts(dict, combined_dict)
            for index2 in [i for i in range(0, self.E_ACTORS) if i != index]:
                ally = 'Enemy' + str(index2)

                # actor.setReward(minimizeFeature(stateKey(enemy, 'health')), self.ENEMY[0])
                actor.setReward(minimizeDifference(stateKey(ally, 'x'), stateKey(ally, 'goal_x')),
                                self.ENEMY[2])
                actor.setReward(minimizeDifference(stateKey(ally, 'y'), stateKey(ally, 'goal_y')),
                                self.ENEMY[2])
            self.set_enemy_actions(actor, index)

            # Terminate game if enemy reaches goal
            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'x'), stateKey(actor.name, 'goal_x')),
                             True: {'if': equalFeatureRow(stateKey(actor.name, 'y'), stateKey(actor.name, 'goal_y')),
                                    True: True,
                                    False: False},
                             False: False})
            self.world.addTermination(tree)

    def set_enemy_actions(self, actor, index):
        # Nop
        index2 = 0
        action = actor.addAction({'verb': 'Wait'})
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'x'), 0.))
        self.world.setDynamics(stateKey(action['subject'], 'x'), action, tree)
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'y'), 0.))
        self.world.setDynamics(stateKey(action['subject'], 'y'), action, tree)
        tree = makeTree(incrementMatrix('turns', 1.0))
        self.world.setDynamics(stateKey(None, 'turns'), action, tree)

        # Increment X position
        action = actor.addAction({'verb': 'MoveRight'})
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'x'), 1.))
        self.world.setDynamics(stateKey(action['subject'], 'x'), action, tree)
        tree = makeTree(incrementMatrix('turns', 1.0))
        self.world.setDynamics('turns', action, tree)

        curr_dist = float(self.world.getState(action['subject'], 'dist_start').domain()[0])
        new_x = float(self.world.getState(action['subject'], 'x').domain()[0])+1.0
        y = float(self.world.getState(action['subject'], 'y').domain()[0])
        start_x = float(self.world.getState(action['subject'], 'start_x').domain()[0])
        start_y = float(self.world.getState(action['subject'], 'start_y').domain()[0])
        new_dist = math.sqrt((start_x-new_x)**2+(start_y-y)**2)
        inc = new_dist - curr_dist
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'dist_start'), inc))
        self.world.setDynamics(stateKey(action['subject'], 'dist_start'), action, tree)

        # Rightmost boundary check
        dict = {}

        edl = []
        for index2 in range(self.F_ACTORS):
            if not dict:
                dict.update({'if': equalRow(stateKey(actor.name, 'x'), str(self.MAP_SIZE_X-1)),
                             True: False,
                             False: {
                                 'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Actor' + str(index2), 'x'),
                                                     -1),
                                 True: {'if': differenceRow(stateKey(actor.name, 'x'),
                                                            stateKey('Actor' + str(index2), 'x'), 0),
                                        True: {
                                            'if': differenceRow(stateKey(actor.name, 'x'),
                                                                stateKey('Actor' + str(index2), 'goal_x'),
                                                                -1),
                                            True: {'if': differenceRow(stateKey(actor.name, 'x'),
                                                                       stateKey('Actor' + str(index2), 'goal_x'), 0),
                                                   True: {},
                                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                                                 stateKey('Actor' + str(index2), 'goal_y')),
                                                           True: False,
                                                           False: {}}},
                                            False: {}
                                        },
                                        False: {'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                                      stateKey('Actor' + str(index2), 'y')),
                                                True: {'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'),
                                                                             '0'),
                                                       True: {

                                                           'if': differenceRow(stateKey(actor.name, 'x'),
                                                                               stateKey('Actor' + str(index2),
                                                                                        'goal_x'),
                                                                               -1),
                                                           True: {'if': differenceRow(stateKey(actor.name, 'x'),
                                                                                      stateKey('Actor' + str(index2),
                                                                                               'goal_x'), 0),
                                                                  True: {},
                                                                  False: {
                                                                      'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                                                            stateKey(
                                                                                                'Actor' + str(index2),
                                                                                                'goal_y')),
                                                                      True: False,
                                                                      False: {}}},
                                                           False: {}
                                                       },
                                                       False: False},
                                                False: {

                                                    'if': differenceRow(stateKey(actor.name, 'x'),
                                                                        stateKey('Actor' + str(index2), 'goal_x'),
                                                                        -1),
                                                    True: {'if': differenceRow(stateKey(actor.name, 'x'),
                                                                               stateKey('Actor' + str(index2),
                                                                                        'goal_x'), 0),
                                                           True: {},
                                                           False: {'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                                                         stateKey('Actor' + str(index2),
                                                                                                  'goal_y')),
                                                                   True: False,
                                                                   False: {}}},
                                                    False: {}
                                                }}},
                                 False: {

                                     'if': differenceRow(stateKey(actor.name, 'x'),
                                                         stateKey('Actor' + str(index2), 'goal_x'),
                                                         -1),
                                     True: {'if': differenceRow(stateKey(actor.name, 'x'),
                                                                stateKey('Actor' + str(index2), 'goal_x'), 0),
                                            True: {},
                                            False: {'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                                          stateKey('Actor' + str(index2), 'goal_y')),
                                                    True: False,
                                                    False: {}}},
                                     False: {}
                                 }}})
            else:
                newdict = {'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Actor' + str(index2), 'x'), -1),
                           True: {
                               'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Actor' + str(index2), 'x'), 0),
                               True: {},
                               False: {
                                   'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                         stateKey('Actor' + str(index2), 'y')),
                                   True: {'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                          True: {},
                                          False: False},
                                   False: {}}},
                           False: {}}
                
                edl.append(newdict)
        for index2 in range(self.E_ACTORS):
            if 'Enemy' + str(index2) == actor.name:
                continue
            elif index2 != (self.E_ACTORS - 1):
                if index2 + 1 == (self.E_ACTORS - 1) and 'Enemy' + str(index2 + 1) == actor.name:
                    
                    newdict = {'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Enemy' + str(index2), 'x'), -1),
                               True: {
                                   'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Enemy' + str(index2), 'x'),
                                                       0),
                                   True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                   False: {
                                       'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                             stateKey('Enemy' + str(index2), 'y')),
                                       False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                       True: {'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                              True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                              False: False}}},
                               False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}
                    
                    combined_dict = newdict
                    for dict2 in reversed(edl):
                        combined_dict = recursiveFillEmptyDicts(dict2, combined_dict)
                    dict = recursiveFillEmptyDicts(dict, combined_dict)
                else:
                    newdict = {'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Enemy' + str(index2), 'x'), -1),
                               True: {
                                   'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Enemy' + str(index2), 'x'),
                                                       0),
                                   True: {},
                                   False: {
                                       'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                             stateKey('Enemy' + str(index2), 'y')),
                                       True: {'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                              True: {},
                                              False: False},
                                       False: {}}},
                               False: {}}
                    
                    edl.append(newdict)
            else:
                
                newdict = {'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Enemy' + str(index2), 'x'), -1),
                           True: {
                               'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Enemy' + str(index2), 'x'), 0),
                               True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                               False: {
                                   'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                         stateKey('Enemy' + str(index2), 'y')),
                                   True: {'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                          True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                          False: False},
                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}},
                           False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}
                
                combined_dict = newdict
                for dict2 in reversed(edl):
                    combined_dict = recursiveFillEmptyDicts(dict2, combined_dict)
                dict = recursiveFillEmptyDicts(dict, combined_dict)
                # print(dict)
        tree = makeTree(dict)
        actor.setLegal(action, tree)
        #
        # ##############################
        #
        # Decrement X position
        action = actor.addAction({'verb': 'MoveLeft'})
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'x'), -1.))
        self.world.setDynamics(stateKey(action['subject'], 'x'), action, tree)
        tree = makeTree(incrementMatrix('turns', 1.0))
        self.world.setDynamics(stateKey(None, 'turns'), action, tree)

        curr_dist = float(self.world.getState(action['subject'], 'dist_start').domain()[0])
        new_x = float(self.world.getState(action['subject'], 'x').domain()[0]) - 1.0
        y = float(self.world.getState(action['subject'], 'y').domain()[0])
        start_x = float(self.world.getState(action['subject'], 'start_x').domain()[0])
        start_y = float(self.world.getState(action['subject'], 'start_y').domain()[0])
        new_dist = math.sqrt((start_x - new_x) ** 2 + (start_y - y) ** 2)
        inc = new_dist - curr_dist
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'dist_start'), inc))
        self.world.setDynamics(stateKey(action['subject'], 'dist_start'), action, tree)

        # Leftmost boundary check, min X = 0
        dict = {}

        edl = []
        for index2 in range(self.F_ACTORS):
            if not dict:
                dict.update({'if': equalRow(stateKey(actor.name, 'x'), '0'),
                             True: False,
                             False: {
                                 'if': differenceRow(stateKey('Actor' + str(index2), 'x'), stateKey(actor.name, 'x'),
                                                     -1),
                                 True: {'if': differenceRow(stateKey('Actor' + str(index2), 'x'), stateKey(actor.name, 'x'), 0),
                                        True: {
                                            'if': differenceRow(stateKey('Actor' + str(index2), 'goal_x'),
                                                                stateKey(actor.name, 'x'),
                                                                -1),
                                            True: {'if': differenceRow(stateKey('Actor' + str(index2), 'goal_x'),
                                                                       stateKey(actor.name, 'x'), 0),
                                                   True: {},
                                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                                                 stateKey('Actor' + str(index2), 'goal_y')),
                                                           True: False,
                                                           False: {}}},
                                            False: {}
                                        },
                                        False: {'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                                      stateKey('Actor' + str(index2), 'y')),
                                                True: {'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'),
                                                                             '0'),
                                                       True: {'if': differenceRow(stateKey('Actor' + str(index2), 'goal_x'),
                                                                stateKey(actor.name, 'x'),
                                                                -1),
                                            True: {'if': differenceRow(stateKey('Actor' + str(index2), 'goal_x'),
                                                                       stateKey(actor.name, 'x'), 0),
                                                   True: {},
                                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                                                 stateKey('Actor' + str(index2), 'goal_y')),
                                                           True: False,
                                                           False: {}}},
                                            False: {}},
                                                       False: False},
                                                False: {'if': differenceRow(stateKey('Actor' + str(index2), 'goal_x'),
                                                                stateKey(actor.name, 'x'),
                                                                -1),
                                            True: {'if': differenceRow(stateKey('Actor' + str(index2), 'goal_x'),
                                                                       stateKey(actor.name, 'x'), 0),
                                                   True: {},
                                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                                                 stateKey('Actor' + str(index2), 'goal_y')),
                                                           True: False,
                                                           False: {}}},
                                            False: {}}}},
                                 False: {'if': differenceRow(stateKey('Actor' + str(index2), 'goal_x'),
                                                                stateKey(actor.name, 'x'),
                                                                -1),
                                            True: {'if': differenceRow(stateKey('Actor' + str(index2), 'goal_x'),
                                                                       stateKey(actor.name, 'x'), 0),
                                                   True: {},
                                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                                                 stateKey('Actor' + str(index2), 'goal_y')),
                                                           True: False,
                                                           False: {}}},
                                            False: {}}}})
            else:
                newdict = {'if': differenceRow(stateKey('Actor' + str(index2), 'x'), stateKey(actor.name, 'x'), -1),
                           True: {
                               'if': differenceRow(stateKey('Actor' + str(index2), 'x'), stateKey(actor.name, 'x'), 0),
                               True: {},
                               False: {
                                   'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                         stateKey('Actor' + str(index2), 'y')),
                                   True: {'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                          True: {},
                                          False: False},
                                   False: {}}},
                           False: {}}
                
                edl.append(newdict)
        for index2 in range(self.E_ACTORS):
            if 'Enemy' + str(index2) == actor.name:
                continue
            elif index2 != (self.E_ACTORS - 1):
                if index2 + 1 == (self.E_ACTORS - 1) and 'Enemy' + str(index2 + 1) == actor.name:
                    
                    newdict = {'if': differenceRow(stateKey('Actor' + str(index2), 'x'), stateKey(actor.name, 'x'), -1),
                               True: {
                                   'if': differenceRow(stateKey('Enemy' + str(index2), 'x'), stateKey(actor.name, 'x'),
                                                       0),
                                   True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                   False: {
                                       'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                             stateKey('Enemy' + str(index2), 'y')),
                                       False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                       True: {'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                              True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                              False: False}}},
                               False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}
                    
                    combined_dict = newdict
                    for dict2 in reversed(edl):
                        combined_dict = recursiveFillEmptyDicts(dict2, combined_dict)
                    dict = recursiveFillEmptyDicts(dict, combined_dict)
                else:
                    newdict = {'if': differenceRow(stateKey('Enemy' + str(index2), 'x'), stateKey(actor.name, 'x'), -1),
                               True: {
                                   'if': differenceRow(stateKey('Enemy' + str(index2), 'x'), stateKey(actor.name, 'x'),
                                                       0),
                                   True: {},
                                   False: {
                                       'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                             stateKey('Enemy' + str(index2), 'y')),
                                       True: {'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                              True: {},
                                              False: False},
                                       False: {}}},
                               False: {}}
                    
                    edl.append(newdict)
            else:
                
                newdict = {'if': differenceRow(stateKey('Enemy' + str(index2), 'x'), stateKey(actor.name, 'x'),  -1),
                           True: {
                               'if': differenceRow(stateKey('Enemy' + str(index2), 'x'), stateKey(actor.name, 'x'), 0),
                               True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                               False: {
                                   'if': equalFeatureRow(stateKey(actor.name, 'y'),
                                                         stateKey('Enemy' + str(index2), 'y')),
                                   True: {'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                          True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                          False: False},
                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}},
                           False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}
                
                combined_dict = newdict
                for dict2 in reversed(edl):
                    combined_dict = recursiveFillEmptyDicts(dict2, combined_dict)
                dict = recursiveFillEmptyDicts(dict, combined_dict)
                # print(dict)
        tree = makeTree(dict)
        actor.setLegal(action, tree)
        #
        # ##############################
        #
        # Increment Y position
        action = actor.addAction({'verb': 'MoveUp'})
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'y'), 1.))
        self.world.setDynamics(stateKey(action['subject'], 'y'), action, tree)
        tree = makeTree(incrementMatrix('turns', 1.0))
        self.world.setDynamics(stateKey(None, 'turns'), action, tree)

        curr_dist = float(self.world.getState(action['subject'], 'dist_start').domain()[0])
        x = float(self.world.getState(action['subject'], 'x').domain()[0])
        new_y = float(self.world.getState(action['subject'], 'y').domain()[0]) + 1.0
        start_x = float(self.world.getState(action['subject'], 'start_x').domain()[0])
        start_y = float(self.world.getState(action['subject'], 'start_y').domain()[0])
        new_dist = math.sqrt((start_x - x) ** 2 + (start_y - new_y) ** 2)
        inc = new_dist - curr_dist
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'dist_start'), inc))
        self.world.setDynamics(stateKey(action['subject'], 'dist_start'), action, tree)

        # Upmost boundary check, max Y
        dict = {}

        edl = []
        for index2 in range(self.F_ACTORS):
            if not dict:
                dict.update({'if': equalRow(stateKey(actor.name, 'y'), str(self.MAP_SIZE_Y-1)),
                             True: False,
                             False: {
                                 'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Actor' + str(index2), 'y'),
                                                     -1),
                                 True: {'if': differenceRow(stateKey(actor.name, 'y'),
                                                            stateKey('Actor' + str(index2), 'y'), 0),
                                        True: {
                                            'if': differenceRow(stateKey(actor.name, 'y'),
                                                                stateKey('Actor' + str(index2), 'goal_y'),
                                                                -1),
                                            True: {'if': differenceRow(stateKey(actor.name, 'y'),
                                                                       stateKey('Actor' + str(index2), 'goal_y'), 0),
                                                   True: {},
                                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                                                 stateKey('Actor' + str(index2), 'goal_x')),
                                                           True: False,
                                                           False: {}}},
                                            False: {}
                                        },
                                        False: {'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                                      stateKey('Actor' + str(index2), 'x')),
                                                True: {'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'),
                                                                             '0'),
                                                       True: {'if': differenceRow(stateKey(actor.name, 'y'),
                                                                stateKey('Actor' + str(index2), 'goal_y'),
                                                                -1),
                                            True: {'if': differenceRow(stateKey(actor.name, 'y'),
                                                                       stateKey('Actor' + str(index2), 'goal_y'), 0),
                                                   True: {},
                                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                                                 stateKey('Actor' + str(index2), 'goal_x')),
                                                           True: False,
                                                           False: {}}},
                                            False: {}},
                                                       False: False},
                                                False: {'if': differenceRow(stateKey(actor.name, 'y'),
                                                                stateKey('Actor' + str(index2), 'goal_y'),
                                                                -1),
                                            True: {'if': differenceRow(stateKey(actor.name, 'y'),
                                                                       stateKey('Actor' + str(index2), 'goal_y'), 0),
                                                   True: {},
                                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                                                 stateKey('Actor' + str(index2), 'goal_x')),
                                                           True: False,
                                                           False: {}}},
                                            False: {}}}},
                                 False: {'if': differenceRow(stateKey(actor.name, 'y'),
                                                                stateKey('Actor' + str(index2), 'goal_y'),
                                                                -1),
                                            True: {'if': differenceRow(stateKey(actor.name, 'y'),
                                                                       stateKey('Actor' + str(index2), 'goal_y'), 0),
                                                   True: {},
                                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                                                 stateKey('Actor' + str(index2), 'goal_x')),
                                                           True: False,
                                                           False: {}}},
                                            False: {}}}})
            else:
                newdict = {'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Actor' + str(index2), 'y'), -1),
                           True: {
                               'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Actor' + str(index2), 'y'), 0),
                               True: {},
                               False: {
                                   'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                         stateKey('Actor' + str(index2), 'x')),
                                   True: {'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                          True: {},
                                          False: False},
                                   False: {}}},
                           False: {}}
                
                edl.append(newdict)
        for index2 in range(self.E_ACTORS):
            if 'Enemy' + str(index2) == actor.name:
                continue
            elif index2 != (self.E_ACTORS - 1):
                if index2 + 1 == (self.E_ACTORS - 1) and 'Enemy' + str(index2 + 1) == actor.name:
                    
                    newdict = {'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y'), -1),
                               True: {
                                   'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y'),
                                                       0),
                                   True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                   False: {
                                       'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                             stateKey('Enemy' + str(index2), 'x')),
                                       False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                       True: {'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                              True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                              False: False}}},
                               False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}
                    
                    combined_dict = newdict
                    for dict2 in reversed(edl):
                        combined_dict = recursiveFillEmptyDicts(dict2, combined_dict)
                    dict = recursiveFillEmptyDicts(dict, combined_dict)
                else:
                    newdict = {'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y'), -1),
                               True: {
                                   'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y'),
                                                       0),
                                   True: {},
                                   False: {
                                       'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                             stateKey('Enemy' + str(index2), 'x')),
                                       True: {'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                              True: {},
                                              False: False},
                                       False: {}}},
                               False: {}}
                    
                    edl.append(newdict)
            else:
                
                newdict = {'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y'), -1),
                           True: {
                               'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Enemy' + str(index2), 'y'), 0),
                               True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                               False: {
                                   'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                         stateKey('Enemy' + str(index2), 'x')),
                                   True: {'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                          True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                          False: False},
                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}},
                           False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}
                
                combined_dict = newdict
                for dict2 in reversed(edl):
                    combined_dict = recursiveFillEmptyDicts(dict2, combined_dict)
                dict = recursiveFillEmptyDicts(dict, combined_dict)
                # print(dict)
        tree = makeTree(dict)
        actor.setLegal(action, tree)
        #
        # ##############################
        #
        # Decrement Y position
        action = actor.addAction({'verb': 'MoveDown'})
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'y'), -1.))
        self.world.setDynamics(stateKey(action['subject'], 'y'), action, tree)
        tree = makeTree(incrementMatrix('turns', 1.0))
        self.world.setDynamics(stateKey(None, 'turns'), action, tree)

        curr_dist = float(self.world.getState(action['subject'], 'dist_start').domain()[0])
        x = float(self.world.getState(action['subject'], 'x').domain()[0])
        new_y = float(self.world.getState(action['subject'], 'y').domain()[0]) - 1.0
        start_x = float(self.world.getState(action['subject'], 'start_x').domain()[0])
        start_y = float(self.world.getState(action['subject'], 'start_y').domain()[0])
        new_dist = math.sqrt((start_x - x) ** 2 + (start_y - new_y) ** 2)
        inc = new_dist - curr_dist
        tree = makeTree(incrementMatrix(stateKey(action['subject'], 'dist_start'), inc))
        self.world.setDynamics(stateKey(action['subject'], 'dist_start'), action, tree)

        # Downmost boundary check, min Y = 0
        dict = {}

        edl = []
        for index2 in range(self.F_ACTORS):
            if not dict:
                dict.update({'if': equalRow(stateKey(actor.name, 'y'), '0'),
                             True: False,
                             False: {
                                 'if': differenceRow(stateKey('Actor' + str(index2), 'y'), stateKey(actor.name, 'y'),
                                                     -1),
                                 True: {'if': differenceRow(stateKey('Actor' + str(index2), 'y'), stateKey(actor.name, 'y'),
                                                             0),
                                        True: {
                                            'if': differenceRow(stateKey('Actor' + str(index2), 'goal_y'),
                                                                stateKey(actor.name, 'y'),
                                                                -1),
                                            True: {'if': differenceRow(stateKey('Actor' + str(index2), 'goal_y'),
                                                                       stateKey(actor.name, 'y'),
                                                                       0),
                                                   True: {},
                                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                                                 stateKey('Actor' + str(index2), 'goal_x')),
                                                           True: False,
                                                           False: {}}},
                                            False: {}
                                        },
                                        False: {'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                                      stateKey('Actor' + str(index2), 'x')),
                                                True: {'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'),
                                                                             '0'),
                                                       True: {
                                            'if': differenceRow(stateKey('Actor' + str(index2), 'goal_y'),
                                                                stateKey(actor.name, 'y'),
                                                                -1),
                                            True: {'if': differenceRow(stateKey('Actor' + str(index2), 'goal_y'),
                                                                       stateKey(actor.name, 'y'),
                                                                       0),
                                                   True: {},
                                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                                                 stateKey('Actor' + str(index2), 'goal_x')),
                                                           True: False,
                                                           False: {}}},
                                            False: {}},
                                                       False: False},
                                                False: {
                                            'if': differenceRow(stateKey('Actor' + str(index2), 'goal_y'),
                                                                stateKey(actor.name, 'y'),
                                                                -1),
                                            True: {'if': differenceRow(stateKey('Actor' + str(index2), 'goal_y'),
                                                                       stateKey(actor.name, 'y'),
                                                                       0),
                                                   True: {},
                                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                                                 stateKey('Actor' + str(index2), 'goal_x')),
                                                           True: False,
                                                           False: {}}},
                                            False: {}}}},
                                 False: {
                                            'if': differenceRow(stateKey('Actor' + str(index2), 'goal_y'),
                                                                stateKey(actor.name, 'y'),
                                                                -1),
                                            True: {'if': differenceRow(stateKey('Actor' + str(index2), 'goal_y'),
                                                                       stateKey(actor.name, 'y'),
                                                                       0),
                                                   True: {},
                                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                                                 stateKey('Actor' + str(index2), 'goal_x')),
                                                           True: False,
                                                           False: {}}},
                                            False: {}}}})
            else:
                newdict = {'if': differenceRow(stateKey('Actor' + str(index2), 'y'), stateKey(actor.name, 'y'), -1),
                           True: {
                               'if': differenceRow(stateKey('Actor' + str(index2), 'y'), stateKey(actor.name, 'y'), 0),
                               True: {},
                               False: {
                                   'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                         stateKey('Actor' + str(index2), 'x')),
                                   True: {'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                          True: {},
                                          False: False},
                                   False: {}}},
                           False: {}}
                
                edl.append(newdict)
        for index2 in range(self.E_ACTORS):
            if 'Enemy' + str(index2) == actor.name:
                continue
            elif index2 != (self.E_ACTORS - 1):
                if index2 + 1 == (self.E_ACTORS - 1) and 'Enemy' + str(index2 + 1) == actor.name:
                    
                    newdict = {'if': differenceRow(stateKey('Enemy' + str(index2), 'y'), stateKey(actor.name, 'y'), -1),
                               True: {
                                   'if': differenceRow(stateKey('Enemy' + str(index2), 'y'), stateKey(actor.name, 'y'),
                                                       0),
                                   True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                   False: {
                                       'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                             stateKey('Enemy' + str(index2), 'x')),
                                       False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                       True: {'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                              True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                              False: False}}},
                               False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}
                    
                    combined_dict = newdict
                    for dict2 in reversed(edl):
                        combined_dict = recursiveFillEmptyDicts(dict2, combined_dict)
                    dict = recursiveFillEmptyDicts(dict, combined_dict)
                else:
                    newdict = {'if': differenceRow(stateKey('Enemy' + str(index2), 'y'), stateKey(actor.name, 'y'), -1),
                               True: {
                                   'if': differenceRow(stateKey('Enemy' + str(index2), 'y'), stateKey(actor.name, 'y'),
                                                       0),
                                   True: {},
                                   False: {
                                       'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                             stateKey('Enemy' + str(index2), 'x')),
                                       True: {'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                              True: {},
                                              False: False},
                                       False: {}}},
                               False: {}}
                    
                    edl.append(newdict)
            else:
                
                newdict = {'if': differenceRow(stateKey('Enemy' + str(index2), 'y'), stateKey(actor.name, 'y'), -1),
                           True: {
                               'if': differenceRow(stateKey('Enemy' + str(index2), 'y'), stateKey(actor.name, 'y'), 0),
                               True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                               False: {
                                   'if': equalFeatureRow(stateKey(actor.name, 'x'),
                                                         stateKey('Enemy' + str(index2), 'x')),
                                   True: {'if': equalFeatureRow(stateKey('Enemy' + str(index2), 'health'), '0'),
                                          True: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True},
                                          False: False},
                                   False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}},
                           False: {'if': equalFeatureRow(stateKey(actor.name, 'health'), '0'),
                                                True: False,
                                                False: True}}
                
                combined_dict = newdict
                for dict2 in reversed(edl):
                    combined_dict = recursiveFillEmptyDicts(dict2, combined_dict)
                dict = recursiveFillEmptyDicts(dict, combined_dict)
                # print(dict)
        tree = makeTree(dict)
        actor.setLegal(action, tree)

        ##############################

        # Attack right direction
        action = actor.addAction({'verb': 'AttackRight'})
        tree = makeTree(incrementMatrix('turns', 1.0))
        self.world.setDynamics(stateKey(None, 'turns'), action, tree)
        for index2 in range(0, self.F_ACTORS):
            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'y'), stateKey('Actor' + str(index2), 'y')),
                             True: {'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Actor' + str(index2), 'x'),
                                                        -1),
                                    True: {'if': differenceRow(stateKey(actor.name, 'x'),
                                                               stateKey('Actor' + str(index2), 'x'), 0),
                                           True: incrementMatrix(stateKey('Actor' + str(index2), 'x'), 0.0),
                                           False: {
                                               'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                               True: incrementMatrix(stateKey('Actor' + str(index2), 'x'), 0.0),
                                               False: setToFeatureMatrix(stateKey('Actor' + str(index2),'x'), stateKey('Actor' + str(index2),'start_x'))}},
                                    False: incrementMatrix(stateKey('Actor' + str(index2), 'x'), 0.0)},
                             False: incrementMatrix(stateKey('Actor' + str(index2), 'x'), 0.0)})
            self.world.setDynamics(stateKey('Actor' + str(index2), 'x'), action, tree)

            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'y'), stateKey('Actor' + str(index2), 'y')),
                             True: {'if': differenceRow(stateKey(actor.name, 'x'), stateKey('Actor' + str(index2), 'x'),
                                                        -1),
                                    True: {'if': differenceRow(stateKey(actor.name, 'x'),
                                                               stateKey('Actor' + str(index2), 'x'), 0),
                                           True: incrementMatrix(stateKey('Actor' + str(index2), 'y'), 0.0),
                                           False: {
                                               'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                               True: incrementMatrix(stateKey('Actor' + str(index2), 'y'), 0.0),
                                               False: setToFeatureMatrix(stateKey('Actor' + str(index2), 'y'),
                                                                         stateKey('Actor' + str(index2), 'start_y'))}},
                                    False: incrementMatrix(stateKey('Actor' + str(index2), 'y'), 0.0)},
                             False: incrementMatrix(stateKey('Actor' + str(index2), 'y'), 0.0)})
            self.world.setDynamics(stateKey('Actor' + str(index2), 'y'), action, tree)
        actor.setLegal(action, makeTree({'if': equalRow(stateKey(actor.name, 'health'), '0'), True: False, False: True}))

        ##############################

        # Attack left direction
        action = actor.addAction({'verb': 'AttackLeft'})
        tree = makeTree(incrementMatrix('turns', 1.0))
        self.world.setDynamics(stateKey(None, 'turns'), action, tree)
        for index2 in range(0, self.F_ACTORS):
            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'y'), stateKey('Actor' + str(index2), 'y')),
                             True: {'if': differenceRow(stateKey('Actor' + str(index2), 'x'), stateKey(actor.name, 'x'),
                                                        -1),
                                    True: {'if': differenceRow(stateKey('Actor' + str(index2), 'x'), stateKey(actor.name, 'x'),
                                                                0),
                                           True: incrementMatrix(stateKey('Actor' + str(index2), 'x'), 0.0),
                                           False: {
                                               'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                               True: incrementMatrix(stateKey('Actor' + str(index2), 'x'), 0.0),
                                               False: setToFeatureMatrix(stateKey('Actor' + str(index2),'x'), stateKey('Actor' + str(index2),'start_x'))}},
                                    False: incrementMatrix(stateKey('Actor' + str(index2), 'health'), 0.0)},
                             False: incrementMatrix(stateKey('Actor' + str(index2), 'x'), 0.0)})
            self.world.setDynamics(stateKey('Actor' + str(index2), 'x'), action, tree)

            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'y'), stateKey('Actor' + str(index2), 'y')),
                             True: {'if': differenceRow(stateKey('Actor' + str(index2), 'x'), stateKey(actor.name, 'x'),
                                                        -1),
                                    True: {'if': differenceRow(stateKey('Actor' + str(index2), 'x'),
                                                               stateKey(actor.name, 'x'),
                                                               0),
                                           True: incrementMatrix(stateKey('Actor' + str(index2), 'y'), 0.0),
                                           False: {
                                               'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                               True: incrementMatrix(stateKey('Actor' + str(index2), 'y'), 0.0),
                                               False: setToFeatureMatrix(stateKey('Actor' + str(index2), 'y'),
                                                                         stateKey('Actor' + str(index2), 'start_y'))}},
                                    False: incrementMatrix(stateKey('Actor' + str(index2), 'y'), 0.0)},
                             False: incrementMatrix(stateKey('Actor' + str(index2), 'y'), 0.0)})
            self.world.setDynamics(stateKey('Actor' + str(index2), 'y'), action, tree)
        actor.setLegal(action, makeTree({'if': equalRow(stateKey(actor.name, 'health'), '0'), True: False, False: True}))

        ##############################

        # Attack up direction
        action = actor.addAction({'verb': 'AttackUp'})
        tree = makeTree(incrementMatrix('turns', 1.0))
        self.world.setDynamics(stateKey(None, 'turns'), action, tree)
        for index2 in range(0, self.F_ACTORS):
            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'x'), stateKey('Actor' + str(index2), 'x')),
                             True: {'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Actor' + str(index2), 'y'),
                                                        -1),
                                    True: {'if': differenceRow(stateKey(actor.name, 'y'),
                                                               stateKey('Actor' + str(index2), 'y'), 0),
                                           True: incrementMatrix(stateKey('Actor' + str(index2), 'x'), 0.0),
                                           False: {
                                               'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                               True: incrementMatrix(stateKey('Actor' + str(index2), 'x'), 0.0),
                                               False: setToFeatureMatrix(stateKey('Actor' + str(index2),'x'), stateKey('Actor' + str(index2),'start_x'))}},
                                    False: incrementMatrix(stateKey('Actor' + str(index2), 'x'), 0.0)},
                             False: incrementMatrix(stateKey('Actor' + str(index2), 'x'), 0.0)})
            self.world.setDynamics(stateKey('Actor' + str(index2), 'x'), action, tree)

            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'x'), stateKey('Actor' + str(index2), 'x')),
                             True: {'if': differenceRow(stateKey(actor.name, 'y'), stateKey('Actor' + str(index2), 'y'),
                                                        -1),
                                    True: {'if': differenceRow(stateKey(actor.name, 'y'),
                                                               stateKey('Actor' + str(index2), 'y'), 0),
                                           True: incrementMatrix(stateKey('Actor' + str(index2), 'y'), 0.0),
                                           False: {
                                               'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                               True: incrementMatrix(stateKey('Actor' + str(index2), 'y'), 0.0),
                                               False: setToFeatureMatrix(stateKey('Actor' + str(index2), 'y'),
                                                                         stateKey('Actor' + str(index2), 'start_y'))}},
                                    False: incrementMatrix(stateKey('Actor' + str(index2), 'y'), 0.0)},
                             False: incrementMatrix(stateKey('Actor' + str(index2), 'y'), 0.0)})
            self.world.setDynamics(stateKey('Actor' + str(index2), 'y'), action, tree)
        actor.setLegal(action, makeTree({'if': equalRow(stateKey(actor.name, 'health'), '0'), True: False, False: True}))

        ##############################

        # Attack down direction
        action = actor.addAction({'verb': 'AttackDown'})
        tree = makeTree(incrementMatrix('turns', 1.0))
        self.world.setDynamics(stateKey(None, 'turns'), action, tree)
        for index2 in range(0, self.F_ACTORS):
            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'x'), stateKey('Actor' + str(index2), 'x')),
                             True: {'if': differenceRow(stateKey('Actor' + str(index2), 'y'), stateKey(actor.name, 'y'),
                                                        -1),
                                    True: {'if': differenceRow(stateKey('Actor' + str(index2), 'y'),
                                                               stateKey(actor.name, 'y'), 0),
                                           True: incrementMatrix(stateKey('Actor' + str(index2), 'x'), 0.0),
                                           False: {
                                               'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                               True: incrementMatrix(stateKey('Actor' + str(index2), 'x'), 0.0),
                                               False: setToFeatureMatrix(stateKey('Actor' + str(index2),'x'), stateKey('Actor' + str(index2),'start_x'))}},
                                    False: incrementMatrix(stateKey('Actor' + str(index2), 'x'), 0.0)},
                             False: incrementMatrix(stateKey('Actor' + str(index2), 'x'), 0.0)})
            self.world.setDynamics(stateKey('Actor' + str(index2), 'x'), action, tree)

            tree = makeTree({'if': equalFeatureRow(stateKey(actor.name, 'x'), stateKey('Actor' + str(index2), 'x')),
                             True: {'if': differenceRow(stateKey('Actor' + str(index2), 'y'), stateKey(actor.name, 'y'),
                                                        -1),
                                    True: {'if': differenceRow(stateKey('Actor' + str(index2), 'y'),
                                                               stateKey(actor.name, 'y'), 0),
                                           True: incrementMatrix(stateKey('Actor' + str(index2), 'y'), 0.0),
                                           False: {
                                               'if': equalFeatureRow(stateKey('Actor' + str(index2), 'health'), '0'),
                                               True: incrementMatrix(stateKey('Actor' + str(index2), 'y'), 0.0),
                                               False: setToFeatureMatrix(stateKey('Actor' + str(index2), 'y'),
                                                                         stateKey('Actor' + str(index2), 'start_y'))}},
                                    False: incrementMatrix(stateKey('Actor' + str(index2), 'y'), 0.0)},
                             False: incrementMatrix(stateKey('Actor' + str(index2), 'y'), 0.0)})
            self.world.setDynamics(stateKey('Actor' + str(index2), 'y'), action, tree)
        actor.setLegal(action, makeTree({'if': equalRow(stateKey(actor.name, 'health'), '0'), True: False, False: True}))

    # Obsolete
    def evaluate_score(self):
        cwd = os.getcwd()
        # print(cwd)
        t = str(time())
        file = open(cwd + "\output\\" + t + ".txt", "w")
        file.write("Parameters:\n")
        file.write("Map Size X: " + str(self.MAP_SIZE_X) + "\n")
        file.write("Map Size Y: " + str(self.MAP_SIZE_Y) + "\n")
        file.write("Soldiers: " + str(self.F_ACTORS) + "\n")
        file.write("Soldier Start Locations: " + str(self.F_START_LOC) + "\n")
        file.write("Soldier Goal Locations: " + str(self.F_GOAL_LOC) + "\n")
        file.write("Enemies: " + str(self.E_ACTORS) + "\n")
        file.write("Enemy Start Locations: " + str(self.E_START_LOC) + "\n")
        # file.write("Bases/Helicopters: " + str(self.D_ACTORS) + "\n")
        # file.write("Base/Helicopter Start Locations: " + str(self.D_START_LOC) + "\n")
        file.write("\n \n")
        file.write("Weights:\n")
        file.write("Soldier:\n")
        file.write("Minimizing soldier and goal distance: " + str(self.AGENT[0]) + "\n")
        file.write("Minimizing soldier and enemy distance: " + str(self.AGENT[1]) + "\n")
        file.write("Enemy:\n")
        file.write("Minimizing soldier and enemy distance: " + str(self.ENEMY[0]) + "\n")
        file.write("Minimizing soldier and helicopter distance: " + str(self.ENEMY[1]) + "\n")
        # file.write("Minimizing soldier and goal distance: " + str(self.ENEMY[2]) + "\n")
        file.write("Base:\n")
        file.write("Minimizing helicopter and enemy distance: " + str(self.BASE[0]) + "\n")
        file.write("Minimizing helicopter cost: " + str(self.BASE[1]) + "\n")
        file.write("Helicopter:\n")
        # file.write("Minimizing helicopter and enemy distance: " + str(self.DISTRACTOR[0]) + "\n")
        # file.write("Minimizing soldier and enemy distance : " + str(self.DISTRACTOR[1]) + "\n")
        file.write("\n \n")

        max_distance = self.MAP_SIZE_X + self.MAP_SIZE_Y

        file.write("Scores:\n")
        file.write("Soldier-Goal Manhattan Distance: \n")
        agent_goal_scores = []
        for index in range(0, self.F_ACTORS):
            ending_x = int(self.world.getState('Actor' + str(index), 'x').domain()[0])
            ending_y = int(self.world.getState('Actor' + str(index), 'y').domain()[0])
            soldier_goal_distance = abs(self.f_get_goal_x(index) - ending_x) + abs(
                self.f_get_goal_y(index) - ending_y)
            agent_goal_scores.append(max_distance - soldier_goal_distance)
            file.write("Soldier" + str(index) + ": " + str(agent_goal_scores[index]) + "\n")
            # print(agent_goal_scores[index])

        file.write("Soldier-Enemy Manhattan Distance: \n")
        agent_enemy_scores = []
        for index in range(0, self.F_ACTORS):
            soldier_x = int(self.world.getState('Actor' + str(index), 'x').domain()[0])
            soldier_y = int(self.world.getState('Actor' + str(index), 'y').domain()[0])
            enemy_x = int(self.world.getState('Enemy' + str(index), 'x').domain()[0])
            enemy_y = int(self.world.getState('Enemy' + str(index), 'y').domain()[0])
            soldier_enemy_distance = abs(soldier_x - enemy_x) + abs(
                soldier_y - enemy_y)
            agent_enemy_scores.append(soldier_enemy_distance - max_distance)
            file.write("Soldier" + str(index) + ": " + str(agent_enemy_scores[index]) + "\n")
            if(agent_enemy_scores[index] == 0):
                file.write("Soldier was captured, penalty awarded")

        file.write("Soldier-Enemy Health: \n")
        # agent_enemy_scores = []
        for index in range(0, self.F_ACTORS):
            soldier_h = int(self.world.getState('Actor' + str(index), 'health').domain()[0])
            enemy_h = int(self.world.getState('Enemy' + str(index), 'health').domain()[0])
            file.write("Soldier" + str(index) + ": " + str(soldier_h) + "\n")
            file.write("Enemy" + str(index) + ": " + str(enemy_h) + "\n")
            # print(agent_enemy_scores[index])

        # file.write("Helicopter Deployment Costs: \n")
        # helicopter_cost_scores =[]
        # for index in range(0, self.D_ACTORS):
        #     helicopter_score = int(self.world.getState('Distractor'+str(index), 'cost').domain()[0])
        #     helicopter_cost_scores.append(helicopter_score)
        #     file.write("Distractor"+str(index)+": "+ str(helicopter_cost_scores[index])+"\n")

        file.write("Turns Taken: \n")
        turns = int(self.world.getState(None,'turns').domain()[0])
        file.write(str(turns) + "\n")
        if(turns < 10):
            file.write("Bonus for taking less than 10 turns")

        file.write("Overall Score: \n")
        for index in range(0, self.F_ACTORS):
            score = agent_goal_scores[index] + agent_enemy_scores[index] + 20 + 20 - turns
            if int(self.world.getState('Actor' + str(index), 'health').domain()[0]) == 0:
                score = -1
            possible = max_distance + 20 + 20
            # print(float(score * 100 / possible))
            total = float(score * 100 / possible)
            file.write("Soldier" + str(index) + ": " + str(total))

    def return_score(self):
        # max_distance = self.MAP_SIZE_X + self.MAP_SIZE_Y
        win = 0.0
        perform = [0.0 for i in range(self.F_ACTORS)]
        soldier_goal_distance = [-1.0 for i in range(self.F_ACTORS)]
        soldier_enemy_defenses = [-1.0 for i in range(self.F_ACTORS)]
        for index in range(0,self.F_ACTORS):
            soldier_health = int(self.world.getState('Actor' + str(index), 'health').domain()[0])
            ending_x = int(self.world.getState('Actor' + str(index), 'x').domain()[0])
            ending_y = int(self.world.getState('Actor' + str(index), 'y').domain()[0])
            soldier_goal_distance[index] = abs(self.f_get_goal_x(0) - ending_x) + abs(
                    self.f_get_goal_y(0) - ending_y)
            soldier_enemy_defenses = int(self.world.getState('Actor' + str(index), 'defenses').domain()[0])
            if soldier_goal_distance[index] == 0 and soldier_health > 0:
                win = 1.0
                # perform[index] = 1.0
        for index in range(0, self.E_ACTORS):
            enemy_health = int(self.world.getState('Enemy' + str(index), 'health').domain()[0])
            ending_x = int(self.world.getState('Enemy' + str(index), 'x').domain()[0])
            ending_y = int(self.world.getState('Enemy' + str(index), 'y').domain()[0])
            enemy_goal_distance = abs(self.e_get_goal_x(0) - ending_x) + abs(
                self.e_get_goal_y(0) - ending_y)
            if enemy_goal_distance == 0 and enemy_health > 0:
                win = -1.0
                # perform[index] = 1.0
            # elif soldier_health == 0:
            #     perform[index] = -1.0
        # soldier_x = int(self.world.getState('Actor' + str(0), 'x').domain()[0])
        # soldier_y = int(self.world.getState('Actor' + str(0), 'y').domain()[0])
        # enemy_x = int(self.world.getState('Enemy' + str(0), 'x').domain()[0])
        # enemy_y = int(self.world.getState('Enemy' + str(0), 'y').domain()[0])
        # soldier_enemy_distance = abs(soldier_x - enemy_x) + abs(
        #     soldier_y - enemy_y)
        #
        # helicopter_score = int(self.world.getState('Distractor'+str(0), 'cost').domain()[0])
        #
        for index in range(0,perform.length()):
            if perform[index]==0.:
                perform[index] = soldier_enemy_defenses[index] - soldier_goal_distance[index] + 20
        turns = int(self.world.getState(None,'turns').domain()[0])
        if not win:
            turns = 99999.
        result = {'team': win, 'indv': perform, 'turns': turns}
        return result

    def run_without_visual(self):
        while not self.world.terminated():
            result = self.world.step()
        return self.return_score()
        self.evaluate_score()

    def run_with_visual(self):
        pyglet.resource.path = ['../Resources/teamwork']
        pyglet.resource.reindex()

        SCREEN_WIDTH = self.MAP_SIZE_X * 32
        SCREEN_HEIGHT = self.MAP_SIZE_Y * 32
        window = pyglet.window.Window(resizable=True)
        window.set_size(SCREEN_WIDTH, SCREEN_HEIGHT)

        tile_image = pyglet.resource.image("grass.png")
        tiles_batch = pyglet.graphics.Batch()
        tiles = []
        for y in range(0, self.MAP_SIZE_Y):
            for x in range(0, self.MAP_SIZE_X):
                tiles.append(pyglet.sprite.Sprite(
                    img=tile_image,
                    x=x * 32,
                    y=y * 32,
                    batch=tiles_batch)
                )

        goal_image = pyglet.resource.image("target.png")
        goals_batch = pyglet.graphics.Batch()
        goals = []
        for index in range(0, len(self.F_GOAL_LOC)):
            goals.append(pyglet.sprite.Sprite(
                img=goal_image,
                x=self.f_get_goal_x(index) * 32,
                y=self.f_get_goal_y(index) * 32,
                batch=goals_batch)
            )

        e_goal_image = pyglet.resource.image("e_target.png")
        e_goals_batch = pyglet.graphics.Batch()
        e_goals = []
        for index in range(0, len(self.E_GOAL_LOC)):
            e_goals.append(pyglet.sprite.Sprite(
                img=e_goal_image,
                x=self.e_get_goal_x(index) * 32,
                y=self.e_get_goal_y(index) * 32,
                batch=e_goals_batch)
            )

        agent_image = pyglet.resource.image("soldier_blue_3.png")
        agents_batch = pyglet.graphics.Batch()
        agents = []
        for index in range(0, self.F_ACTORS):
            agents.append(pyglet.sprite.Sprite(
                img=agent_image,
                x=self.f_get_start_x(index) * 32,
                y=self.f_get_start_y(index) * 32,
                batch=agents_batch)
            )

        enemy_image = pyglet.resource.image("soldier_red_3.png")
        enemies_batch = pyglet.graphics.Batch()
        enemies = []
        for index in range(0, self.E_ACTORS):
            enemies.append(pyglet.sprite.Sprite(
                img=enemy_image,
                x=self.e_get_start_x(index) * 32,
                y=self.e_get_start_y(index) * 32,
                batch=enemies_batch)
            )

        distractor_image = pyglet.resource.image("heli.png")
        base_image = pyglet.resource.image("base.png")
        allies_batch = pyglet.graphics.Batch()
        bases = []
        # distractors = []
        # for index in range(0, self.D_ACTORS):
        #     bases.append(pyglet.sprite.Sprite(
        #         img=base_image,
        #         x=self.d_get_start_x(index) * 32,
        #         y=self.d_get_start_y(index) * 32,
        #         batch=allies_batch)
        #     )
        #     distractors.append(pyglet.sprite.Sprite(
        #         img=distractor_image,
        #         x=self.d_get_start_x(index) * 32,
        #         y=self.d_get_start_y(index) * 32,
        #         batch=allies_batch)
        #     )

        # supplier_image = pyglet.resource.image("target_old.png")
        # suppliers_batch = pyglet.graphics.Batch()
        # suppliers = []
        # for index in range(0, self.S_ACTORS):
        #     suppliers.append(pyglet.sprite.Sprite(
        #         img=supplier_image,
        #         x=self.s_get_start_x(index) * 32,
        #         y=self.s_get_start_y(index) * 32,
        #         batch=suppliers_batch)
        #     )



        @window.event
        def on_draw():
            window.clear()
            tiles_batch.draw()
            goals_batch.draw()
            e_goals_batch.draw()
            agents_batch.draw()
            enemies_batch.draw()
            allies_batch.draw()

            # suppliers_batch.draw()

        @window.event
        def on_key_press(symbol, modifiers):
            if symbol == key.P:
                self.paused = True
                print('Paused')
            if symbol == key.U:
                self.paused = False
                print('Resumed')

        def update(dt):
            if not self.paused:
                result = self.world.step()
                self.world.explain(result, 2)
                if self.world.terminated():
                    self.evaluate_score()
                    window.close()
                    event_loop = pyglet.app.EventLoop()
                    event_loop.exit()

            for index in range(0, self.F_ACTORS):
                agents[index].x = int(self.world.getState('Actor' + str(index), 'x').domain()[0]) * 32
                agents[index].y = int(self.world.getState('Actor' + str(index), 'y').domain()[0]) * 32
                if int(self.world.getState('Actor' + str(index), 'health').domain()[0]) == 2:
                    agents[index] = pyglet.sprite.Sprite(
                        img=pyglet.resource.image("soldier_blue_2.png"),
                        x=agents[index].x,
                        y=agents[index].y,
                        batch=agents_batch)
                elif int(self.world.getState('Actor' + str(index), 'health').domain()[0]) == 1:
                    agents[index] = pyglet.sprite.Sprite(
                        img=pyglet.resource.image("soldier_blue_1.png"),
                        x=agents[index].x,
                        y=agents[index].y,
                        batch=agents_batch)
                elif int(self.world.getState('Actor' + str(index), 'health').domain()[0]) == 0:
                    agents[index] = pyglet.sprite.Sprite(
                        img=pyglet.resource.image("scenario.png"),
                        x=agents[index].x,
                        y=agents[index].y,
                        batch=agents_batch)
                    # del self.world.agents["Actor" + str(index)]
                    # self.world.setOrder(self.world.agents.keys())

                    # self.world.setOrder([a for a in self.world.agents.keys() if a != "Actor"+str(index)])
                    # for i in range(0,self.F_ACTORS):
                    #     self.friendly_agents[i].beliefs.deleteKeys([keys.turnKey("Actor" + str(index)),
                    #                     keys.modelKey("Actor" + str(index))])
                    # for i in range(0,self.E_ACTORS):
                    #     self.enemy_agents[i].beliefs.deleteKeys([keys.turnKey("Actor" + str(index)),
                    #                     keys.modelKey("Actor" + str(index))])

            for index in range(0, self.E_ACTORS):
                actor = self.friendly_agents[index]
                # print(actor.decide(self.world.state[None].domain()[0], horizon = 5))
                enemies[index].x = int(self.world.getState('Enemy' + str(index), 'x').domain()[0]) * 32
                enemies[index].y = int(self.world.getState('Enemy' + str(index), 'y').domain()[0]) * 32
                if int(self.world.getState('Enemy' + str(index), 'health').domain()[0]) == 2:
                    enemies[index] = pyglet.sprite.Sprite(
                        img=pyglet.resource.image("soldier_red_2.png"),
                        x=enemies[index].x,
                        y=enemies[index].y,
                        batch=enemies_batch)
                elif int(self.world.getState('Enemy' + str(index), 'health').domain()[0]) == 1:
                    enemies[index] = pyglet.sprite.Sprite(
                        img=pyglet.resource.image("soldier_red_1.png"),
                        x=enemies[index].x,
                        y=enemies[index].y,
                        batch=enemies_batch)
                elif int(self.world.getState('Enemy' + str(index), 'health').domain()[0]) == 0 and 'Enemy'+str(index) in self.world.agents.keys():
                    enemies[index] = pyglet.sprite.Sprite(
                        img=pyglet.resource.image("scenario.png"),
                        x=enemies[index].x,
                        y=enemies[index].y,
                        batch=enemies_batch)
                    # del self.world.agents["Enemy"+str(index)]
                    # self.world.setOrder(self.world.agents.keys())

                    # self.world.setOrder([e for e in self.world.agents.keys() if e != "Enemy" + str(index)])

                    # beliefs.deleteKeys([keys.turnKey("Enemy" + str(index)),
                    #                     keys.modelKey("Enemy" + str(index))])
                    #
                    # beliefs.deleteKeys([keys.turnKey("Enemy" + str(index)),
                    #                     keys.modelKey("Enemy" + str(index))])


            # for index in range(0, self.D_ACTORS):
            #     distractors[index].x = int(self.world.getState('Distractor' + str(index), 'x').domain()[0]) * 32
            #     distractors[index].y = int(self.world.getState('Distractor' + str(index), 'y').domain()[0]) * 32

            # for index in range(0, self.S_ACTORS):
            #     suppliers[index].x = int(self.world.getState('Supplier' + str(index), 'x').domain()[0]) * 32
            #     suppliers[index].y = int(self.world.getState('Supplier' + str(index), 'y').domain()[0]) * 32


            filename = "frame-" + str(self.file_num) + '.png'
            pyglet.image.get_buffer_manager().get_color_buffer().save(filename)
            self.file_num += 1
            file_list = glob.glob('*.png')  # Get all the pngs in the current directory
            list.sort(file_list, key=lambda x: int(
                x.split('-')[1].split('.png')[
                    0]))  # Sort the images by #, this may need to be tweaked for your use case
            i = file_list.index(filename)+1
            while i < len(file_list):
                dupname = file_list[i]
                os.remove(dupname)
                i+=1


        pyglet.clock.schedule_interval(update, 0.1)
        pyglet.app.run()
        # Thread(target=pyglet.app.run()).start()
        # target=pyglet.app.run()


def run(genome,visual,asynch):
    # s1 = genome[0]
    # s2 = genome[1]
    b1 = genome[2]
    b2 = genome[3]
    # h1 = genome[4]
    # h2 = genome[5]
    # d1 = genome[6]
    # d2 = genome[7]

    run = Scenario(
        MAP_SIZE_X=7,
        MAP_SIZE_Y=7,
        F_ACTORS=3,
        F_START_LOC=["1,2", "2,2", "2,1"],
        F_GOAL_LOC=["5,5","5,5","5,5"],
        F_ENERGY=[10.0],
        E_ACTORS=3,
        E_START_LOC=["5,4", "4,4", "4,5"],
        E_GOAL_LOC = ["1,1", "1,1", "1,1"],
        E_PATROL_RANGE=5,
        # D_ACTORS=1,
        # D_START_LOC=["2,3"],
        # S_ACTORS=1,
        # S_ENERGY=[10.0],
        # S_START_LOC=["1,4"],
        BASE=[b1, b2],
        # DISTRACTOR=[h1, h2],
        # SUPPLIER=[d1, d2],
        ENEMY=[1., 1., 1.],
        AGENT=[[1., 1., 1.], [1., 1., 1.], [1., 1., 1.]],
        ABILITY=[[1.0, 1.0], [1.0, 1.0], [1.0, 1.0]],
        ASYNC=asynch)
    score = None
    if visual:
        score = run.run_with_visual()
    else:
        score = run.run_without_visual()
    run.file_num = 0
    return score

    '''
    BASE=[0.5, 0.2],
    DISTRACTOR=[-1.0, 1.0],
    ENEMY=[0.5, 0.6, -1.0],
    AGENT=[0.5, -0.5]
    '''

def recursiveFillEmptyDicts(dict, newdict):
    copydict = copy.deepcopy(dict)
    if type(dict) is bool:
        return dict
    for (key,value) in copydict.iteritems():
        if type(value) is type(dict):
            if value == {}:
                dict[key].update(newdict)
            else:
                dict[key] = recursiveFillEmptyDicts(dict[key],newdict)
    return dict

def get_tutor_instruction(resultsdict,reward, win_strats,lose_strats):
    curr_atk = reward['attack']
    curr_pro = reward['protect']
    curr_ast = reward['assist']
    goal_pro = 1.0
    goal_ast = 1.0
    mindist = float('inf')
    for strat in win_strats:
        if math.sqrt((strat[0] - curr_pro)**2+(strat[1] - curr_ast)**2) > mindist:
            mindist = math.sqrt((strat[0] - curr_pro)**2+(strat[1] - curr_ast)**2)
            goal_pro = strat[0]
            goal_ast = strat[1]
    goal_atk = 1.0

    return (goal_atk-curr_atk, goal_pro-curr_pro, goal_ast-curr_ast)


if __name__ == '__main__':
    logging.basicConfig()
    parser = ArgumentParser()
    parser.add_argument('-d', '--debug', default='INFO', help='Level of logging detail [default: %(default)s]')
    parser.add_argument('-v', '--visual', action='store_true', help='Run with visualization [default: %(default)s]')
    parser.add_argument('-n', '--number', type=int, default=50,
                        help='Number of trials (when no visualization) [default: %(default)s]')
    parser.add_argument('-i', '--diverse', action='store_true', help='Let agents learn independently')
    parser.add_argument('-s', '--async', action='store_true', help='Let agents act simultaneously')
    parser.add_argument('-a', '--alpha', type=float, default=0.2,
                        help='Learning rate (when no visualization) [default: %(default)s]')
    parser.add_argument('-l', '--lambda', type=float, default=0.0,
                        help='Adoption rate (when no visualization) of tutor suggestion [default: %(default)s]')
    parser.add_argument('-g', '--gamma', type=float, default=0.0,
                        help='Rate (when no visualization) at which teammates learn from each other\'s performance [default: %(default)s]')
    parser.add_argument('-e', '--epsilon', type=float, default=0.0,
                        help='Probability that agents make random exploration moves in the reward space [default: %(default)s]')
    parser.add_argument('-r', '--ro', type=float, default=0.0,
                        help='Weight of random exploration deviations relative to standard learning [default: %(default)s]')

    args = vars(parser.parse_args())

    level = getattr(logging, args['debug'].upper(), None)
    if not isinstance(level, int):
        raise ValueError('Invalid debug level: %s' % args['debug'])
    logging.getLogger().setLevel(level)



    if(args['visual']):
        print(run([0.4219082416329605, -0.3776566392876486, 0.43254428266334544, 0.0, -0.6093841194695164, 0.2128550551796511,0.5,0.7],True,args['async']))
        gif_name = 'result'
        fps = 12
        file_list = glob.glob('*.png')  # Get all the pngs in the current directory
        list.sort(file_list, key=lambda x: int(
            x.split('-')[1].split('.png')[0]))  # Sort the images by #, this may need to be tweaked for your use case
        clip = mpy.ImageSequenceClip(file_list, fps=fps)
        clip.write_gif('{}.gif'.format(gif_name), fps=fps)
        for x in file_list:
            os.remove(x)
    else:
        log = []
        reward = {'attack': 1.,
                  'protect': 0.,
                  'assist': 0.}
        indv_rewards = [{'attack': 1.,
                         'protect': 1.,
                         'assist': 1.},
                        {'attack': 1.,
                         'protect': 1.,
                         'assist': 1.},
                        {'attack': 1.,
                         'protect': 1.,
                         'assist': 1.}
            ]

        if not args['diverse']:
            indv_rewards = [reward,
                            reward,
                            reward
                            ]
        for trial in range(args['number']):
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
                # D_ACTORS=1,
                # D_START_LOC=["2,3"],
                # S_ACTORS=1,
                # S_ENERGY=[10.0],
                # S_START_LOC=["1,4"],
                # BASE=[b1, b2],
                # DISTRACTOR=[h1, h2],
                # SUPPLIER=[d1, d2],
                ENEMY=[1., 1., 1.],
                AGENT=[[indv_rewards[i]['attack'], indv_rewards[i]['protect'], indv_rewards[i]['assist']] for i in range(3)],
                ABILITY=[[1.0, 1.0], [1.0, 1.0], [1.0, 1.0]],
                ASYNC=args['async'])
            result = run.run_without_visual()
            result.update({'Rattack': reward['attack'],
                           'Rprotect': reward['protect'],
                           'Rassist': reward['assist'],
                           'trial': trial,
                           'team': result['team'],
                           'turns': result['turns'],
                           'alpha': args['alpha'],
                           })
            logging.info(result)
            log.append(result)

            # Let's learn!
            totalDelta = 0.

            # Tutoring
            resultsdict = {"0.25":
                               {"0.25":0, "0.5":0, "1":0, "2":0, "4":0},
                            "0.5":
                                {"0.25":0, "0.5":0, "1":0, "2":0, "4":0},
                            "1":
                                {"0.25":0, "0.5":0, "1":0, "2":0, "4":0},
                            "2":
                                {"0.25":0,"0.5":0,"1":0,"2":0,"4":0},
                            "4":
                                {"0.25":0, "0.5":0, "1":0, "2":0, "4":0}
                            }
            win_strats = []
            lose_strats = []
            with open('rewardtrials10/results.csv') as csvfile:
                reader = csv.reader(csvfile, delimiter=',')
                vals = ["0.25","0.5","1","2","4"]
                i_c = 0
                i_r = 0
                for row in reader:
                    for val in row:
                        resultsdict[vals[i_c]][vals[i_r]]=val
                        if val == "1":
                            win_strats.append((float(vals[i_c]),float(vals[i_r])))
                        elif val == "-1":
                            lose_strats.append((float(vals[i_c]), float(vals[i_r])))
                        i_c+=1
                    i_c = 0
                    i_r+=1

            (tutor_attack, tutor_protect, tutor_assist) = get_tutor_instruction(resultsdict, reward, win_strats, lose_strats)

            delta = args['lambda'] * tutor_attack
            totalDelta += abs(delta)
            reward['attack'] += delta

            delta = args['lambda'] * tutor_protect
            totalDelta += abs(delta)
            reward['protect'] += delta

            delta = args['lambda'] * tutor_assist
            totalDelta += abs(delta)
            reward['assist'] += delta

            best_performer = None
            maxscore = -1
            for i in range(0,len(result["indv"])):
                if result["indv"][i]>maxscore:
                    best_performer = i
                    maxscore = result["indv"][i]

            (imitate_attack, imitate_protect, imitate_assist) = (indv_rewards[i]['attack'],indv_rewards[i]['protect'],indv_rewards[i]['assist'],)

            delta = args['gamma'] * imitate_attack
            totalDelta += abs(delta)
            reward['attack'] += delta

            delta = args['gamma'] * imitate_protect
            totalDelta += abs(delta)
            reward['protect'] += delta

            delta = args['gamma'] * imitate_assist
            totalDelta += abs(delta)
            reward['assist'] += delta

            if result['team'] == 1:
                logging.info('Blue Team Won')
                break

            elif result['team'] == 0:
                x = random.random()
                e_factor = 0. if x>=args["epsilon"] else args["ro"]
                logging.info('Tie')
                # Base needs to be more aggressive in attacking red flag
                delta = args['alpha'] * ((4. - reward['attack']) + e_factor*(random.uniform(0. - reward['attack'], 4. - reward['attack'])))
                totalDelta += abs(delta)
                reward['attack'] += delta
                if delta <= 0:
                    delta = args['alpha'] * ((4. - reward['assist']) + e_factor*(random.uniform(0. - reward['assist'],4. - reward['assist'])))
                    totalDelta += abs(delta)
                    reward['assist'] += delta
                else:
                    delta = args['alpha'] * (e_factor * (random.uniform(0. - reward['assist'],4. - reward['assist'])))
                    totalDelta += abs(delta)
                    reward['assist'] += delta
                # Agent needs to be less aggressive in protecting blue flag
                delta = args['alpha'] * ((0. - reward['protect']) + e_factor*(random.uniform(0. - reward['protect'],4. - reward['protect'])))
                totalDelta += abs(delta)
                reward['protect'] += delta

            else:
                x = random.random()
                e_factor = 0. if x >= args["epsilon"] else args["ro"]
                logging.info('Blue Team Lost')
                # Base needs to be more aggressive in sending Distractor
                delta = args['alpha'] * ((0. - reward['attack'])+ e_factor*(random.uniform(0. - reward['attack'],4. - reward['attack'])))
                totalDelta += abs(delta)
                reward['attack'] += delta
                # Agent needs to be more aggressive in pursuing goal
                delta = args['alpha'] * ((4. - reward['protect'])+ e_factor*(random.uniform(0. - reward['protect'],4. - reward['protect'])))
                totalDelta += abs(delta)
                reward['protect'] += delta
                if delta <= 0:
                    delta = args['alpha'] * ((4. - reward['assist'])+ e_factor*(random.uniform(0. - reward['assist'],4. - reward['assist'])))
                    totalDelta += abs(delta)
                    reward['assist'] += delta
                else:
                    if delta <= 0:
                        delta = args['alpha'] * ((4. - reward['assist']) + e_factor * (random.uniform(0. - reward['assist'],4. - reward['assist'])))
                        totalDelta += abs(delta)
                        reward['assist'] += delta

            if not args['diverse']:
                indv_rewards = [reward,
                                reward,
                                reward
                                ]

            logging.info('Total delta: %5.2f' % (totalDelta))
            result['delta'] = totalDelta
        fields = ['trial', 'Rattack', 'Rprotect', 'Rassist', 'alpha', 'team', 'turns', 'delta']
        with open(os.path.join('output', '%s.csv' % (time)), 'w') as csvfile:
            writer = csv.DictWriter(csvfile, fields, extrasaction='ignore')
            writer.writeheader()
            for result in log:
                writer.writerow(result)
