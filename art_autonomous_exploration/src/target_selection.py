#!/usr/bin/env python

import rospy
import random
import math
import time
import numpy as np
from timeit import default_timer as timer
from utilities import RvizHandler
from utilities import OgmOperations
from utilities import Print
from brushfires import Brushfires
from topology import Topology
import scipy
from path_planning import PathPlanning


# Class for selecting the next best target
class TargetSelection:

    # Constructor
    def __init__(self, selection_method):
        self.goals_position = []
        self.goals_value = []
        self.omega = 0.0
        self.radius = 0
        self.method = selection_method

        self.brush = Brushfires()
        self.topo = Topology()
        self.path_planning = PathPlanning()


    def selectTarget(self, init_ogm, coverage, robot_pose, origin, \
        resolution, force_random = False):

        target = [-1, -1]

        ######################### NOTE: QUESTION  ##############################
        # Implement a smart way to select the next target. You have the
        # following tools: ogm_limits, Brushfire field, OGM skeleton,
        # topological nodes.

        # Find only the useful boundaries of OGM. Only there calculations
        # have meaning
        ogm_limits = OgmOperations.findUsefulBoundaries(init_ogm, origin, resolution)

        # Blur the OGM to erase discontinuities due to laser rays
        ogm = OgmOperations.blurUnoccupiedOgm(init_ogm, ogm_limits)

        # Calculate Brushfire field
        tinit = time.time()
        brush = self.brush.obstaclesBrushfireCffi(ogm, ogm_limits)
        Print.art_print("Brush time: " + str(time.time() - tinit), Print.ORANGE)

        # Calculate skeletonization
        tinit = time.time()
        skeleton = self.topo.skeletonizationCffi(ogm, \
                   origin, resolution, ogm_limits)
        Print.art_print("Skeletonization time: " + str(time.time() - tinit), Print.ORANGE)

        # Find topological graph
        tinit = time.time()
        nodes = self.topo.topologicalNodes(ogm, skeleton, coverage, origin, \
                resolution, brush, ogm_limits)
        Print.art_print("Topo nodes time: " + str(time.time() - tinit), Print.ORANGE)

        # Visualization of topological nodes
        vis_nodes = []
        for n in nodes:
            vis_nodes.append([
                n[0] * resolution + origin['x'],
                n[1] * resolution + origin['y']
            ])
        RvizHandler.printMarker(\
            vis_nodes,\
            1, # Type: Arrow
            0, # Action: Add
            "map", # Frame
            "art_topological_nodes", # Namespace
            [0.3, 0.4, 0.7, 0.5], # Color RGBA
            0.1 # Scale
        )

        # SMART TARGET SELECTION USING:
        # 1. Brush-fire field
        # 2. OGM Skeleton
        # 3. Topological Nodes
        # 4. Coverage field
        # 5. OGM Limits

        # Next subtarget is selected based on a
        # weighted-calculated score for each node. The score
        # is calculated using normalized values of the brush
        # field and the number of branches. The weight values
        # are defined experimentaly through the tuning method.
        temp_score = 0
        max_score = 0
        best_node = nodes[0]

        # the max-min boundaries are set arbitarily
        BRUSH_MAX = 17
        BRUSH_MIN = 1
        BRUSH_WEIGHT = 2.5
        BRANCH_MAX = 10
        BRANCH_MIN = 0
        BRANCH_WEIGHT = 2.5
        DISTANCE_MIN = 0
        DISTANCE_MAX = 40
        DISTANCE_WEIGHT = 0.5

        for n in nodes:

            # Use brushfire to increase temp_score
            temp_score = (brush[n[0]][n[1]] - BRUSH_MIN) / (BRUSH_MAX - BRUSH_MIN) * BRUSH_WEIGHT

            # Use OGM Skeleton to find potential
            # branches following the target
            branches = 0
            for i in range(-1,2):
                for j in range(-1,2):
                    if (i != 0 or j != 0):
                        branches += skeleton[n[0]+i][n[1]+j]

            # Use OGM-Skeleton to increase temp_score (select a goal with more future options)
            temp_score += (branches - BRANCH_MIN) / (BRANCH_MAX - BRUSH_MIN) * BRANCH_WEIGHT

            # Use OGM-Limits to decrease temp_score
            # the goal closest to OGM limits is best exploration option
            distance = math.sqrt((ogm_limits['max_x'] - n[0]) ** 2 + (ogm_limits['max_y'] - n[1]) ** 2)
            temp_score -= (distance - DISTANCE_MIN) / (DISTANCE_MAX - DISTANCE_MIN) * DISTANCE_WEIGHT

            # If temp_score is higher than current max
            # score, then max score is updated and current node
            # becomes next goal - target
            if temp_score > max_score:
                max_score = temp_score
                best_node = n

        final_x = best_node[0]
        final_y = best_node[1]
        target = [final_x, final_y]

        # Random point
        # if self.method == 'random' or force_random == True:
        #     target = self.selectRandomTarget(ogm, coverage, brush, ogm_limits)
        ########################################################################

        return target

    def selectRandomTarget(self, ogm, coverage, brushogm, ogm_limits):
      # The next target in pixels
        tinit = time.time()
        next_target = [0, 0]
        found = False
        while not found:
          x_rand = random.randint(0, ogm.shape[0] - 1)
          y_rand = random.randint(0, ogm.shape[1] - 1)
          if ogm[x_rand][y_rand] < 50 and coverage[x_rand][y_rand] < 50 and \
              brushogm[x_rand][y_rand] > 5:
            next_target = [x_rand, y_rand]
            found = True
        Print.art_print("Select random target time: " + str(time.time() - tinit), \
            Print.ORANGE)
        return next_target
