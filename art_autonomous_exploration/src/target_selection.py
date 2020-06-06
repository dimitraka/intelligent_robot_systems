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
        #print "ogm_limits", ogm_limits
        # Blur the OGM to erase discontinuities due to laser rays
        ogm = OgmOperations.blurUnoccupiedOgm(init_ogm, ogm_limits)
        #print "ogm", ogm

        # Calculate Brushfire field
        tinit = time.time()
        brush = self.brush.obstaclesBrushfireCffi(ogm, ogm_limits)
        Print.art_print("Brush time: " + str(time.time() - tinit), Print.ORANGE)
        #print "brush", brush

        # Calculate skeletonization
        tinit = time.time()
        skeleton = self.topo.skeletonizationCffi(ogm, \
                   origin, resolution, ogm_limits)
        Print.art_print("Skeletonization time: " + str(time.time() - tinit), Print.ORANGE)
        #print "skeleton", skeleton

        # Find topological graph
        tinit = time.time()
        nodes = self.topo.topologicalNodes(ogm, skeleton, coverage, origin, \
                resolution, brush, ogm_limits)
        Print.art_print("Topo nodes time: " + str(time.time() - tinit), Print.ORANGE)
        #print "nodes", nodes

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
        # 2. OMG Skeleton
        # 3. Topological Nodes
        # 4. Coverage field
        max_brush = -999
        min_dist = 999
        for n in nodes:

            # Find the node with max brush value ->
            # which tends to have max distance from the obstacles
            if brush[n[0]][n[1]] > max_brush:
                max_brush = brush[n[0]][n[1]]
                brush_x = n[0]
                brush_y = n[1]

            # Use OMG Skeleton to find potential
            # branches following the target
            branches = 0
            for i in range(-1,2):
                for j in range(-1,2):
                    if (i != 0 or j != 0):
                        branches += skeleton[brush_x+i][brush_y+j]
            # If branches >= 2 --> path continues beyond the target
            if branches >= 2:
                final_x = brush_x
                final_y = brush_y


        target = [final_x, final_y]

        # Random point
        #if self.method == 'random' or force_random == True:
        #  target = self.selectRandomTarget(ogm, coverage, brush, ogm_limits)
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
