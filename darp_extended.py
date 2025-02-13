"""
source: https://github.com/alice-st/DARP
Creative Commons Attribution-NonCommercial 4.0 International License.
https://creativecommons.org/licenses/by-nc/4.0/

Reference:
@article{kapoutsisdarp,
  title={DARP: Divide Areas Algorithm for Optimal Multi-Robot Coverage Path Planning},
  author={Kapoutsis, Athanasios Ch and Chatzichristofis, Savvas A and Kosmatopoulos, Elias B},
  journal={Journal of Intelligent \& Robotic Systems},
  pages={1--18},
  publisher={Springer}
}

Modifications by Raphaël Dousson, SYCAMORE, EPFL, 2024
"""

import numpy as np
import sys
# import cv2
from Visualization import darp_area_visualization
# import time
import random
import os
from numba import njit

from queue import PriorityQueue

CONNECT_MASK_VALID_CELL = 255

np.set_printoptions(threshold=sys.maxsize)

random.seed(1)
os.environ['PYTHONHASHSEED'] = str(1)
np.random.seed(1)

@njit(fastmath=True)
def assign(droneNo, rows, cols, GridEnv, MetricMatrix, A):

    ArrayOfElements = np.zeros(droneNo)
    for i in range(rows):
        for j in range(cols):
            if GridEnv[i, j] == -1:
                minV = MetricMatrix[0, i, j]
                indMin = 0
                for r in range(droneNo):
                    if MetricMatrix[r, i, j] < minV:
                        minV = MetricMatrix[r, i, j]
                        indMin = r

                A[i, j] = indMin
                ArrayOfElements[indMin] += 1

            elif GridEnv[i, j] == -2:
                A[i, j] = droneNo
    return A, ArrayOfElements

@njit(fastmath=True)
def inverse_binary_map_as_uint8(BinaryMap):
    # cv2.distanceTransform needs input of dtype unit8 (8bit)
    return np.logical_not(BinaryMap).astype(np.uint8)

@njit(fastmath=True)
def euclidian_distance_points2d(array1: np.array, array2: np.array) -> np.float64:
    # this runs much faster than the (numba) np.linalg.norm and is totally enough for our purpose
    return (
                   ((array1[0] - array2[0]) ** 2) +
                   ((array1[1] - array2[1]) ** 2)
           ) ** 0.5

@njit(fastmath=True)
def constructBinaryImages(labels_im, robo_start_point, rows, cols):
    BinaryRobot = np.copy(labels_im)
    BinaryNonRobot = np.copy(labels_im)
    for i in range(rows):
        for j in range(cols):
            if labels_im[i, j] == labels_im[robo_start_point]:
                BinaryRobot[i, j] = 1
                BinaryNonRobot[i, j] = 0
            elif labels_im[i, j] != 0:
                BinaryRobot[i, j] = 0
                BinaryNonRobot[i, j] = 1

    return BinaryRobot, BinaryNonRobot

@njit(fastmath=True)
def CalcConnectedMultiplier(rows, cols, dist1, dist2, CCvariation, connectDistPenaltyOnly):
    returnM = np.zeros((rows, cols))
    MaxV = 0
    MinV = 2**30

    for i in range(rows):
        for j in range(cols):
            returnM[i, j] = dist1[i, j] - dist2[i, j]
            
            if connectDistPenaltyOnly:
                returnM[i, j] = -dist2[i, j]

            if MaxV < returnM[i, j]:
                MaxV = returnM[i, j]
            if MinV > returnM[i, j]:
                MinV = returnM[i, j]

    for i in range(rows):
        for j in range(cols):
            returnM[i, j] = (returnM[i, j]-MinV)*((2*CCvariation)/(MaxV - MinV)) + (1-CCvariation)

    return returnM


@njit(fastmath=True)
def NormalizedCustomDistanceBinary(BinaryMap, movements, movement_map):
    """
    Compute the shortest path distances for all cells with 1, to the nearest 0.
    """
    
    rows, cols = BinaryMap.shape
    distRobot = np.full((rows, cols), np.inf)

    queue = []
    
    # add all 0 cells to the queue and set their distance to 0
    for x in range(rows):
        for y in range(cols):
            if BinaryMap[x, y] == 0:    # cell belonging to subarea with 0s
                distRobot[x, y] = 0
                queue.append((x, y))  # add to the queue to start BFS


    # BFS to propagate distances from 0 cells
    while queue:
        cx, cy = queue.pop(0)

        # check all neighbors
        for dx, dy in movements:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < rows and 0 <= ny < cols and BinaryMap[nx, ny] == 1:  # Only traverse 1 cells
                
                # check if movement is allowed with movement map
                is_movement_allowed = None
                
                dx = nx-cx
                dy = ny-cy
                
                matching_indices = np.where((movements[:, 0] == dx) & (movements[:, 1] == dy))[0] # find the corresponding movement index using np.where
                
                if matching_indices.size == 0:
                    is_movement_allowed = False # no matching movement found
                
                movement_idx = matching_indices[0] # first matching index

                if is_movement_allowed is None:
                    is_movement_allowed = movement_map[cy, cx, movement_idx] # check the movement_map
                
                if is_movement_allowed and distRobot[nx, ny] == np.inf:
                    distRobot[nx, ny] = distRobot[cx, cy] + 1   # set the distance
                    queue.append((nx, ny))  # add to queue for further exploration

    # replace inf values with rows * cols
    for i in range(rows):
        for j in range(cols):
            if distRobot[i, j] == np.inf:
                distRobot[i, j] = rows * cols
    
    return distRobot


@njit(fastmath=True)
def find_connected_components(grid, directions, movement_map, connect_mask_valid_cell):
    """
    Find number of connected subareas in a binary grid using movement map.
    """
    rows, cols = grid.shape
    component_id = 0
    components_map = np.full(grid.shape, 0, dtype=np.int32)
    visited = np.full(grid.shape, False, dtype=np.bool_)

    # traverse all cells to find unvisited ones and start a BFS for each component
    for x in range(rows):
        for y in range(cols):
            if grid[x, y] == connect_mask_valid_cell and not visited[x, y]:
                component_id += 1

                queue = [(x, y)]
                visited[x, y] = True
                components_map[x, y] = component_id

                while queue:
                    cx, cy = queue.pop(0)

                    neighbors = []
                    for dx, dy in directions:
                        nx, ny = cx + dx, cy + dy
                        if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1] and grid[nx, ny] == connect_mask_valid_cell: # Only valid cells
                            
                            # check if movement is allowed with movement map
                            is_movement_allowed = None
                
                            dx = nx-cx
                            dy = ny-cy
                            
                            matching_indices = np.where((directions[:, 0] == dx) & (directions[:, 1] == dy))[0]
                            
                            if matching_indices.size == 0:
                                is_movement_allowed = False
                            
                            movement_idx = matching_indices[0] # first matching index

                            if is_movement_allowed is None:
                                is_movement_allowed = movement_map[cy, cx, movement_idx] # Check the movement_map


                            if is_movement_allowed:

                                # avoid diagonal crossings
                                check_for_diagonal_crossings = True
                                
                                if (cx-nx != 0) and (cy-ny != 0): # check if diagonal motion
                                    if (grid[cx, ny] != connect_mask_valid_cell) and (grid[nx, cy] != connect_mask_valid_cell): # adjacent cells not valid
                                        
                                        # check if movement is allowed with movement map
                                        is_movement_allowed = None
                
                                        dx = nx-cx
                                        dy = cy-ny
                                        
                                        matching_indices = np.where((directions[:, 0] == dx) & (directions[:, 1] == dy))[0]
                                        
                                        if matching_indices.size == 0:
                                            is_movement_allowed = False
                                        
                                        movement_idx = matching_indices[0]

                                        if is_movement_allowed is None:
                                            is_movement_allowed = movement_map[ny, cx, movement_idx]

                                        if is_movement_allowed:                                        
                                            check_for_diagonal_crossings = False
                                
                                if check_for_diagonal_crossings:
                                    neighbors.append((nx, ny))
                                # end of check for diagonal crossings

                    for nx, ny in neighbors:
                        if not visited[nx, ny]:
                            visited[nx, ny] = True
                            components_map[nx, ny] = component_id
                            queue.append((nx, ny))

    component_id += 1 # count number of connected subareas assigned to a given agent
    return component_id, components_map




class DARP:
    def __init__(self, ny, nx, notEqualPortions, given_initial_positions, given_portions, obstacles_positions,
                 visualization, MaxIter=80000, TotMaxIterMult=3, CCvariation=0.01, randomLevel=0.0001, dcells=2,
                 importance=False, astar_usage=False, map_object = None, connectDistPenaltyOnly=False):

        self.rows = ny
        self.cols = nx
        self.initial_positions, self.obstacles_positions, self.portions = self.sanity_check(given_initial_positions, given_portions, obstacles_positions, notEqualPortions)

        self.visualization = visualization
        self.MaxIter = MaxIter
        self.TotMaxIterMult = TotMaxIterMult
        self.CCvariation = CCvariation
        self.randomLevel = randomLevel
        self.dcells = dcells
        self.importance = importance
        self.notEqualPortions = notEqualPortions

        self.astar_usage = astar_usage

        self.map = map_object

        self.connectDistPenaltyOnly = connectDistPenaltyOnly
    

        print("\nInitial Conditions Defined:")
        print("Grid Dimensions:", nx, ny)
        print("Number of Robots:", len(self.initial_positions))
        print("Initial Robots' positions", self.initial_positions)

        self.droneNo = len(self.initial_positions)
        self.A = np.zeros((self.rows, self.cols))
        self.GridEnv = self.defineGridEnv()
   
        self.connectivity = np.zeros((self.droneNo, self.rows, self.cols), dtype=np.uint8)
        self.BinaryRobotRegions = np.zeros((self.droneNo, self.rows, self.cols), dtype=bool)


        #best solution found:
        self.A_best = self.A.copy()
        self.gini_best = 1
        self.ArrayOfElements_best = None
        self.ConnectedRobotRegions_best = None


        # initialize the matrix to store A* distances
        self.AStarDistances = np.zeros((self.droneNo, self.rows, self.cols), dtype=np.int32)

        # compute and store A* distances for each robot
        for r in range(self.droneNo):
            for x in range(self.rows):
                for y in range(self.cols):
                    self.AStarDistances[r, x, y] = self.a_star_search(self.GridEnv, self.initial_positions[r], (x, y))



        self.MetricMatrix, self.termThr, self.Notiles, self.DesireableAssign, self.TilesImportance, self.MinimumImportance, self.MaximumImportance= self.construct_Assignment_Matrix()
        self.ArrayOfElements = np.zeros(self.droneNo)
        self.color = []

        for r in range(self.droneNo):
            np.random.seed(r)
            self.color.append(list(np.random.choice(range(256), size=3)))
        
        np.random.seed(1)
        if self.visualization:
            self.assignment_matrix_visualization = darp_area_visualization(self.A, self.droneNo, self.color, self.initial_positions)

    def sanity_check(self, given_initial_positions, given_portions, obs_pos, notEqualPortions):
        initial_positions = []
        for position in given_initial_positions:
            if position < 0 or position >= self.rows * self.cols:
                print("Initial positions should be inside the Grid.")
                sys.exit(1)
            initial_positions.append((position // self.cols, position % self.cols))

        obstacles_positions = []
        for obstacle in obs_pos:
            if obstacle < 0 or obstacle >= self.rows * self.cols:
                print("Obstacles should be inside the Grid.")
                sys.exit(2)
            obstacles_positions.append((obstacle // self.cols, obstacle % self.cols))

        portions = []
        if notEqualPortions:
            portions = given_portions
        else:
            for drone in range(len(initial_positions)):
                portions.append(1 / len(initial_positions))

        if len(initial_positions) != len(portions):
            print("Portions should be defined for each drone")
            sys.exit(3)

        s = sum(portions)
        if abs(s - 1) >= 0.0001:
            print("Sum of portions should be equal to 1.")
            sys.exit(4)

        for idx, position in enumerate(initial_positions):
            for obstacle in obstacles_positions:
                if position[0] == obstacle[0] and position[1] == obstacle[1]:
                    print("Initial positions should not be on obstacles, robot: ", idx)
                    sys.exit(5)

        if len(initial_positions) != len(set(initial_positions)):
            print("Multiple robots should not be on the same grid cell")
            sys.exit(6)


        return initial_positions, obstacles_positions, portions
          
    def defineGridEnv(self):
        GridEnv = np.full(shape=(self.rows, self.cols), fill_value=-1)  # create non obstacle map with value -1
        
        # obstacle tiles value is -2
        for idx, obstacle_pos in enumerate(self.obstacles_positions):
            GridEnv[obstacle_pos[0], obstacle_pos[1]] = -2

        connectivity = np.zeros((self.rows, self.cols))
        
        mask = np.where(GridEnv == -1)
        connectivity[mask[0], mask[1]] = CONNECT_MASK_VALID_CELL
        image = np.uint8(connectivity)
        # num_labels, labels_im = cv2.connectedComponents(image, connectivity=4)
        # num_labels, labels_im = self.find_connected_components(image)
        num_labels, labels_im = find_connected_components(image, self.map.movements, self.map.movement_map, CONNECT_MASK_VALID_CELL)


        if num_labels > 2:
            print("The environment grid MUST not have unreachable and/or closed shape regions")
            sys.exit(6)
        
        # initial robot tiles will have their array.index as value
        for idx, robot in enumerate(self.initial_positions):
            GridEnv[robot] = idx
            self.A[robot] = idx

        return GridEnv

    def divideRegions(self):
        success = False
        cancelled = False
        criterionMatrix = np.zeros((self.rows, self.cols))
        iteration = 0
        tot_iteration = 0

        ConnectedRobotRegions = np.zeros(self.droneNo)

        self.dcells = (self.dcells - (1-self.termThr))

        while (self.termThr <= self.dcells and not success and not cancelled) or (np.sum(ConnectedRobotRegions) != self.droneNo):
            
            if tot_iteration >= self.TotMaxIterMult * self.MaxIter: # stop DARP even if no convergence
                success = False
                break

            downThres = (self.Notiles - self.termThr*(self.droneNo-1))/(self.Notiles*self.droneNo)
            upperThres = (self.Notiles + self.termThr)/(self.Notiles*self.droneNo)

            success = True

            # Main optimization loop

            iteration=0

            print("DARP outer loop, iter: ", tot_iteration, ", termThr : ", self.termThr, ", dcells: ", self.dcells)


            while iteration < self.MaxIter and not cancelled:
                self.A, self.ArrayOfElements = assign(self.droneNo,
                                                      self.rows,
                                                      self.cols,
                                                      self.GridEnv,
                                                      self.MetricMatrix,
                                                      self.A)
                ConnectedMultiplierList = np.ones((self.droneNo, self.rows, self.cols))
                # ConnectedRobotRegions = np.zeros(self.droneNo)
                plainErrors = np.zeros((self.droneNo))
                divFairError = np.zeros((self.droneNo))

                self.update_connectivity()
                for r in range(self.droneNo):
                    ConnectedMultiplier = np.ones((self.rows, self.cols))
                    ConnectedRobotRegions[r] = True
                    # num_labels, labels_im = cv2.connectedComponents(self.connectivity[r, :, :], connectivity=4)
                    # num_labels, labels_im = self.find_connected_components(self.connectivity[r, :, :])
                    # start_time = time.time()
                    num_labels, labels_im = find_connected_components(self.connectivity[r, :, :], self.map.movements, self.map.movement_map, CONNECT_MASK_VALID_CELL)
                    # print("exec time : ", time.time() - start_time)


                    if num_labels > 2:
                        ConnectedRobotRegions[r] = False
                        BinaryRobot, BinaryNonRobot = constructBinaryImages(labels_im, self.initial_positions[r], self.rows, self.cols)
                        
                        # ConnectedMultiplier = CalcConnectedMultiplier(self.rows, self.cols,
                        #                                               self.NormalizedEuclideanDistanceBinary(True, BinaryRobot),
                        #                                               self.NormalizedEuclideanDistanceBinary(False, BinaryNonRobot), self.CCvariation)
                        
                        dist1 = self.NormalizedEuclideanDistanceBinary(True, BinaryRobot)
                        dist2 = self.NormalizedEuclideanDistanceBinary(False, BinaryNonRobot)

                        ConnectedMultiplier = CalcConnectedMultiplier(self.rows, self.cols, dist1, dist2, self.CCvariation, self.connectDistPenaltyOnly)



                    ConnectedMultiplierList[r, :, :] = ConnectedMultiplier
                    plainErrors[r] = self.ArrayOfElements[r]/(self.DesireableAssign[r]*self.droneNo)
                    if plainErrors[r] < downThres:
                        divFairError[r] = downThres - plainErrors[r]
                    elif plainErrors[r] > upperThres:
                        divFairError[r] = upperThres - plainErrors[r]

                if self.IsThisAGoalState(self.termThr, ConnectedRobotRegions):

                    if self.termThr <= self.dcells :
                        self.A_best = self.A.copy()
                        self.ArrayOfElements_best = self.ArrayOfElements.copy()
                        self.ConnectedRobotRegions_best = ConnectedRobotRegions.copy()
                        break
                    else:   # store the solution if its the best one yet
                        gini_current = gini(np.array(self.ArrayOfElements))

                        if gini_current < self.gini_best:
                            self.gini_best = gini_current.copy()
                            self.A_best = self.A.copy()
                            self.ArrayOfElements_best = self.ArrayOfElements.copy()
                            self.ConnectedRobotRegions_best = ConnectedRobotRegions.copy()

                TotalNegPerc = 0
                totalNegPlainErrors = 0
                correctionMult = np.zeros(self.droneNo)

                for r in range(self.droneNo):
                    if divFairError[r] < 0:
                        TotalNegPerc += np.absolute(divFairError[r])
                        totalNegPlainErrors += plainErrors[r]

                    correctionMult[r] = 1

                for r in range(self.droneNo):
                    if totalNegPlainErrors != 0:
                        if divFairError[r] < 0:
                            correctionMult[r] = 1 + (plainErrors[r]/totalNegPlainErrors)*(TotalNegPerc/2)
                        else:
                            correctionMult[r] = 1 - (plainErrors[r]/totalNegPlainErrors)*(TotalNegPerc/2)

                        criterionMatrix = self.calculateCriterionMatrix(
                                self.TilesImportance[r],
                                self.MinimumImportance[r],
                                self.MaximumImportance[r],
                                correctionMult[r],
                                divFairError[r] < 0)

                    self.MetricMatrix[r] = self.FinalUpdateOnMetricMatrix(
                            criterionMatrix,
                            self.generateRandomMatrix(),
                            self.MetricMatrix[r],
                            ConnectedMultiplierList[r, :, :])

                iteration += 1
                tot_iteration += 1
                if self.visualization and iteration%10 == 0:
                    self.assignment_matrix_visualization.placeCells(self.A, iteration_number=tot_iteration)
                #     time.sleep(0.2)

            if iteration >= self.MaxIter:
                # if self.MaxIter > 3000:
                #     self.MaxIter = self.MaxIter/2
                success = False
                self.termThr += 1
                
        if self.visualization:
            self.assignment_matrix_visualization.placeCells(self.A, iteration_number=tot_iteration)
            # time.sleep(0.2)
        self.getBinaryRobotRegions()


        if np.sum(self.ConnectedRobotRegions_best) == self.droneNo:
            success = True

            nb_cells_per_robot = np.array(self.ArrayOfElements_best) + np.ones_like(self.ArrayOfElements_best) # Add initial pose cell

            # DARP metrics
            print("\nDARP metrics : ")
            print("Nb of cells assigned : ", nb_cells_per_robot)
            print("Average : ", np.mean(nb_cells_per_robot))
            print("Standard deviation : ", np.std(nb_cells_per_robot))
            print("Gini coefficient : ", gini(nb_cells_per_robot))
            
            


        return success, tot_iteration, self.ArrayOfElements_best
    

    def getBinaryRobotRegions(self):
        ind = np.where(self.A_best < self.droneNo)
        temp = (self.A_best[ind].astype(int),)+ind
        self.BinaryRobotRegions[temp] = True

    def generateRandomMatrix(self):
        RandomMatrix = np.zeros((self.rows, self.cols))
        RandomMatrix = 2*self.randomLevel*np.random.uniform(0, 1,size=RandomMatrix.shape) + (1 - self.randomLevel)
        return RandomMatrix

    def FinalUpdateOnMetricMatrix(self, CM, RM, currentOne, CC):
        MMnew = np.zeros((self.rows, self.cols))
        MMnew = currentOne*CM*RM*CC

        return MMnew

    def IsThisAGoalState(self, thresh, connectedRobotRegions):        
        if self.termThr <= self.dcells: # first dcells iteration groups, ending if connected and fair work division
            for r in range(self.droneNo):
                if np.absolute(self.DesireableAssign[r] - self.ArrayOfElements[r]) > thresh or not connectedRobotRegions[r]:
                    return False
        else:   # after first dcells iteration groups, ending if connected only
            if(np.sum(connectedRobotRegions)!=self.droneNo):
                return False
            
        return True

    def update_connectivity(self):
        self.connectivity = np.zeros((self.droneNo, self.rows, self.cols), dtype=np.uint8)
        for i in range(self.droneNo):
            mask = np.where(self.A == i)
            self.connectivity[i, mask[0], mask[1]] = CONNECT_MASK_VALID_CELL

    # Construct Assignment Matrix
    def construct_Assignment_Matrix(self):
        Notiles = self.rows*self.cols
        fair_division = 1/self.droneNo
        effectiveSize = Notiles - self.droneNo - len(self.obstacles_positions)
        termThr = 0

        if effectiveSize % self.droneNo != 0:
            termThr = 1

        DesireableAssign = np.zeros(self.droneNo)
        MaximunDist = np.zeros(self.droneNo)
        MaximumImportance = np.zeros(self.droneNo)
        MinimumImportance = np.zeros(self.droneNo)

        for i in range(self.droneNo):
            DesireableAssign[i] = effectiveSize * self.portions[i]
            MinimumImportance[i] = sys.float_info.max
            if (DesireableAssign[i] != int(DesireableAssign[i]) and termThr != 1):
                termThr = 1

        AllDistances = np.zeros((self.droneNo, self.rows, self.cols))
        TilesImportance = np.zeros((self.droneNo, self.rows, self.cols))


        for x in range(self.rows):
            for y in range(self.cols):
                tempSum = 0
                for r in range(self.droneNo):

                    # get precomputed A* distance
                    if(self.astar_usage):
                        AllDistances[r, x, y] = self.AStarDistances[r, x, y] # E!
                    else:
                        AllDistances[r, x, y] = euclidian_distance_points2d(np.array(self.initial_positions[r]), np.array((x, y))) # E!

                    if AllDistances[r, x, y] > MaximunDist[r]:
                        MaximunDist[r] = AllDistances[r, x, y]
                    tempSum += AllDistances[r, x, y]

                for r in range(self.droneNo):
                    if tempSum - AllDistances[r, x, y] != 0:
                        TilesImportance[r, x, y] = 1/(tempSum - AllDistances[r, x, y])
                    else:
                        TilesImportance[r, x, y] = 1

                    if TilesImportance[r, x, y] > MaximumImportance[r]:
                        MaximumImportance[r] = TilesImportance[r, x, y]

                    if TilesImportance[r, x, y] < MinimumImportance[r]:
                        MinimumImportance[r] = TilesImportance[r, x, y]

        return AllDistances, termThr, Notiles, DesireableAssign, TilesImportance, MinimumImportance, MaximumImportance


    def calculateCriterionMatrix(self, TilesImportance, MinimumImportance, MaximumImportance, correctionMult, smallerthan_zero,):
        returnCrit = np.zeros((self.rows, self.cols))
        if self.importance:
            if smallerthan_zero:
                returnCrit = (TilesImportance- MinimumImportance)*((correctionMult-1)/(MaximumImportance-MinimumImportance)) + 1
            else:
                returnCrit = (TilesImportance- MinimumImportance)*((1-correctionMult)/(MaximumImportance-MinimumImportance)) + correctionMult
        else:
            returnCrit[:, :] = correctionMult

        return returnCrit

    def NormalizedEuclideanDistanceBinary(self, RobotR, BinaryMap):
        # distRobot = cv2.distanceTransform(inverse_binary_map_as_uint8(BinaryMap), distanceType=2, maskSize=0, dstType=5)
        # start_time = time.time()
        distRobot = NormalizedCustomDistanceBinary(inverse_binary_map_as_uint8(BinaryMap), self.map.movements, self.map.movement_map)
        # print("exec time : ", time.time() - start_time)

        MaxV = np.max(distRobot)
        MinV = np.min(distRobot)

        #Normalization
        if RobotR:
            distRobot = (distRobot - MinV)*(1/(MaxV-MinV)) + 1
        else:
            distRobot = (distRobot - MinV)*(1/(MaxV-MinV))

        return distRobot
    
    def find_connected_components(self, grid):
        """
        Finds connected components in a binary grid using a custom movement rule.

        :param grid: 2D binary numpy array (1 = cell, 0 = no cell)
        :return: components_map, component_count
                 components_map is a 2D array where each cell is labeled with its component ID.
                 component_count is the total number of components found.
        """
        rows, cols = grid.shape
        components_map = np.zeros_like(grid, dtype=int)
        component_id = 0
        visited = np.zeros_like(grid, dtype=bool)
        directions = self.map.movements

        # traverse all cells to find unvisited ones and start a BFS for each component
        for x in range(rows):
            for y in range(cols):
                if grid[x, y] == CONNECT_MASK_VALID_CELL and not visited[x, y]:
                    component_id += 1
                    components_map, visited = self.bfs(x, y, component_id, components_map, visited, grid, directions)
        
        component_id += 1
        return component_id, components_map

    def bfs(self, start_x, start_y, component_id, components_map, visited, grid, directions):
        """
        BFS to explore a connected component.
        """
        queue = [(start_x, start_y)]
        visited[start_x, start_y] = True
        components_map[start_x, start_y] = component_id

        while queue:
            cx, cy = queue.pop(0)
            for nx, ny in self.get_neighbors(cx, cy, grid, directions):
                if not visited[nx, ny]:
                    visited[nx, ny] = True
                    components_map[nx, ny] = component_id
                    queue.append((nx, ny))
        return components_map, visited
    
    def get_neighbors(self, x, y, grid, directions):
        """
        Get valid neighboring cells considering the movement rules.
        """
        neighbors = []

        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1] and grid[nx, ny] == CONNECT_MASK_VALID_CELL:
                if self.map.is_movement_allowed([x, y], [nx, ny]): # avoid obstacles
                    if self.check_for_diagonal_crossings(x, y, nx, ny, grid):
                        neighbors.append((nx, ny))
        return neighbors
    
    def check_for_diagonal_crossings(self, x1, y1, x2, y2, grid):
        if ((x1-x2 != 0) and (y2-y1 != 0)): # check if diagonal motion
            if((grid[x1][y2] != CONNECT_MASK_VALID_CELL) and (grid[x2][y1] != CONNECT_MASK_VALID_CELL)): # adjacent cells not valid
                if(self.map.is_movement_allowed([x1, y2], [x2, y1])):   # if movement allowed between these adjacent cells
                    return False    # cells should not be connected to avoid to avoid a crossing scenario
        return True
    

    def a_star_search(self, grid, start, goal):
        rows, cols = grid.shape
        directions = self.map.movements
        
        pq = PriorityQueue()
        pq.put((0, start))
        
        # distance from start to cell
        g_score = {start: 0}
        
        # previous cell for path reconstruction
        came_from = {}

        while not pq.empty():
            _, current = pq.get()

            # ending condition
            if current == goal:
                return g_score[current]

            for direction in directions:    #find neighbors
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] != -2:
                    if(self.map.is_movement_allowed([current[0], current[1]], [neighbor[0], neighbor[1]])):
                        tentative_g_score = g_score[current] + 1

                        if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                            g_score[neighbor] = tentative_g_score
                            f_score = tentative_g_score + euclidian_distance_points2d(neighbor, goal)
                            pq.put((f_score, neighbor))
                            came_from[neighbor] = current
        
        return -1  # no path found
    
    
def gini(array):
    """
    Calculate the Gini coefficient of a numpy array.
    Source: https://github.com/oliviaguest/gini/
    Creative Common: CC0-1.0 license
    """
    # based on bottom eq:
    # http://www.statsdirect.com/help/generatedimages/equations/equation154.svg
    # from:
    # http://www.statsdirect.com/help/default.htm#nonparametric_methods/gini.htm
    # All values are treated equally, arrays must be 1d:
    array = array.flatten()
    if np.amin(array) < 0:
        # Values cannot be negative:
        array -= np.amin(array)
    # Values cannot be 0:
    array = array + 0.0000001
    # Values must be sorted:
    array = np.sort(array)
    # Index per array element:
    index = np.arange(1,array.shape[0]+1)
    # Number of array elements:
    n = array.shape[0]
    # Gini coefficient:
    return ((np.sum((2 * index - n  - 1) * array)) / (n * np.sum(array)))