"""
source: https://github.com/rodriguesrenato/coverage-path-planning
MIT License

Copyright (c) 2022 Renato Fernandes Rodrigues

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Modifications by Raphaël Dousson, SYCAMORE, EPFL, 2024
"""

import numpy as np
from enum import Enum, auto
import matplotlib.pyplot as plt
import matplotlib as mpl


class PlannerStatus(Enum):
    STANDBY = auto()
    COVERAGE_SEARCH = auto()
    NEARST_UNVISITED_SEARCH = auto()
    FOUND = auto()
    NOT_FOUND = auto()


class HeuristicType(Enum):
    MANHATTAN = auto()
    CHEBYSHEV = auto()
    VERTICAL = auto()
    HORIZONTAL = auto()
    EUCLIDEAN = auto()
    DIAGONAL_RIGHT = auto()
    DIAGONAL_LEFT = auto()
    DIAGONAL_BOTH = auto()
    HORIZONTAL_VERTICAL = auto()
    NULL = auto()



class CoveragePlanner():

    def __init__(self, map_open):
        self.map_grid = map_open

        self.movement = [[0,  1],  # up
                         [-1, 1],  # up left
                         [-1, 0],   # left
                         [-1,-1],   # down left
                         [0, -1],   # down
                         [1, -1],   # down right
                         [1,  0],   # right
                         [1, 1]]   # up right        
        
        
        # Readable description['up', 'up left', 'left', 'down left', 'down', 'down right', 'right', 'up right']
        self.movement_name = ['^', '^<', '<', 'v<', 'v', 'v>', '>', '^>']

        # Possible actions performed by the robot
        self.action = [0, 1, 2, 3, 4, 5, 6, 7]

        self.action_cost_origin = [1.0, 2.41, 3.0, 4.41, 5.0, 4.41, 3.0, 2.41]
        self.action_cost = self.action_cost_origin.copy()


        # A star movement cost
        self.a_star_movement_cost = [1.0, 1.41, 1.0, 1.41, 1.0, 1.41, 1.0, 1.41]

        # currente position [x, y, orientation (default = 0)]
        self.current_pos = self.get_start_position()

        # Trajectory list of points
        self.current_trajectory = []
        self.current_trajectory_annotations = []

        # The grid that accumulate the visited positions in the map
        self.coverage_grid = np.copy(map_open)

        # The FSM variable
        self.state_ = PlannerStatus.STANDBY

        # Heuristic types of each search algorithm
        self.a_star_heuristic = HeuristicType.EUCLIDEAN
        self.cp_heuristic = HeuristicType.VERTICAL

        self.debug_level = -1

    # Set the debug level
    # Determine how much information is going to be shown in the terminal
    def set_debug_level(self, level):
        self.debug_level = level

    def compute(self, map_object):
        self.printd("compute", "{}".format(self.state_.name), 1)
        while self.compute_non_blocking(map_object):
            pass
        return self.state_

    # The finite state machine that will handle searching strategy
    # Returns True if search is not finished

    def compute_non_blocking(self, map_object):
        self.printd("compute_non_blocking", "{}".format(self.state_.name), 1)
        searching = False

        # Start the FSM by switching through the self.state_ attribute
        if self.state_ == PlannerStatus.COVERAGE_SEARCH:

            # Search using coverage_search algorithm
            heuristic = self.create_heuristic(
                self.current_pos, self.cp_heuristic)
            
            # #!!!!test multi heuristic
            # if self.current_pos[0] > 6 and self.current_pos[0] > 4:
            #     heuristic = self.create_heuristic(
            #     self.current_pos, HeuristicType.VERTICAL)

            # #!!!!test multi heuristic
            # if self.current_pos[0] < 3:
            #     heuristic = self.create_heuristic(
            #     self.current_pos, HeuristicType.VERTICAL)

            res = self.coverage_search(self.current_pos, heuristic, map_object)

            # Update the current position to the final search position
            self.current_pos = [res[1][-1][1], res[1][-1][2], res[1][-1][3]]

            self.append_trajectory(res[1], "CS")

            # Update the current coverage_grid
            self.coverage_grid = res[2]

            # Check if path was successfully found. If not, try to find the closest unvisited position
            if res[0]:
                self.state_ = PlannerStatus.FOUND
                self.current_trajectory[-1][6] = PlannerStatus.FOUND
            else:
                self.state_ = PlannerStatus.NEARST_UNVISITED_SEARCH
                searching = True

        elif self.state_ == PlannerStatus.NEARST_UNVISITED_SEARCH:

            # Search using a_star_search_closest_unvisited algorithm
            heuristic = self.create_heuristic(
                self.current_pos, self.a_star_heuristic)
            res = self.a_star_search_closest_unvisited(
                self.current_pos, heuristic, map_object)

            # In case a path was found
            if res[0]:
                # Update the current position to the final search position
                self.current_pos = [res[1][-1][1],
                                    res[1][-1][2], res[1][-1][3]]

                self.append_trajectory(res[1], "A*")

                # Set FSM to do a coverage search again
                self.state_ = PlannerStatus.COVERAGE_SEARCH
                searching = True

            # If no path was found, just finish searching
            else:
                self.state_ = PlannerStatus.NOT_FOUND
                if len(self.current_trajectory) > 0:
                    self.current_trajectory[-1][6] = PlannerStatus.NOT_FOUND

        else:
            self.printd("compute_non_blocking",
                        "Invalid state given, stop FSM", 0)

        return searching

    # Restart initial position, coverage grid and trajectory list, and set ready to start searching
    def start(self, initial_orientation=0, a_star_heuristic=None, cp_heuristic=None, action_cost_mult=1):
        
        #action cost:
        self.action_cost_mult = action_cost_mult
        self.action_cost = [cost*action_cost_mult for cost in self.action_cost_origin]
        
        # Set current position to the given map start position
        self.current_pos = self.get_start_position(
            orientation=initial_orientation)

        self.coverage_grid = np.copy(self.map_grid)
        self.current_trajectory = []
        self.current_trajectory_annotations = []

        if cp_heuristic is not None:
            self.cp_heuristic = cp_heuristic
        if a_star_heuristic is not None:
            self.a_star_heuristic = a_star_heuristic

        self.state_ = PlannerStatus.COVERAGE_SEARCH
        self.printd("start", " Search set to start at {}, trajectory and coverage grid is cleared".format(
            self.current_pos), debug_level=1)


    def coverage_search(self, initial_pos, heuristic, map_object):
        # Create a reference grid for visited coords
        closed = np.copy(self.coverage_grid)
        closed[initial_pos[0]][initial_pos[1]] = 1

        if self.debug_level > 1:
            self.printd("coverage_search",
                        "initial closed grid:", 2)
            print(closed)

        x = initial_pos[0]
        y = initial_pos[1]
        o = initial_pos[2]
        v = 0

        # Fill the initial coord in the iteration list
        trajectory = [[v, x, y, o, None, None, self.state_]]

        complete_coverage = False
        resign = False

        while not complete_coverage and not resign:

            if self.check_full_coverage(self.map_grid, closed):
                self.printd("coverage_search", "Complete coverage", 2)
                complete_coverage = True

            else:
                # Get the last visited coord info
                v = trajectory[-1][0]
                x = trajectory[-1][1]
                y = trajectory[-1][2]
                o = trajectory[-1][3]

                # [accumulated_cost, x_pos, y_pos, orientation, action_performed_to_get_here, next_action]
                possible_next_coords = []

                # calculate the possible next coords
                for a in range(len(self.action)):
                    o2 = (self.action[a]+o) % len(self.movement)
                    x2 = x + self.movement[o2][0]
                    y2 = y + self.movement[o2][1]

                    # Check if it is out of the map boundaries
                    if x2 >= 0 and x2 < len(self.map_grid) and y2 >= 0 and y2 < len(self.map_grid[0]):
                        # Check if this position was already visited or if it is a visitable position on the map
                        if closed[x2][y2] == 0 and self.map_grid[x2][y2] == 0:
                            if map_object.is_movement_allowed([x,y], [x2,y2]):
                                # Compose the projected: actual accumulated cost + action cost + heuristic cost at given position
                                v2 = v + self.action_cost[a] + heuristic[x2][y2] #!!!!!
                                possible_next_coords.append(
                                    [v2, x2, y2, o2, a, None, self.state_, self.action_cost[a]])
                                
                                # print("iter, prev or ", o, " current or ", o2, " action, ", a, self.action[a], " cost, ", v2)

                # If there isn any possible next position, stop searching
                if len(possible_next_coords) == 0:
                    resign = True
                    self.printd("coverage_search",
                                "Could not find a next unvisited coord", 2)

                # otherwise update the trajectory list wiht the next position with the lowest possible cost
                else:
                    # rank by total_cost
                    possible_next_coords.sort(key=lambda x: x[0])

                    # update the last trajectory next_action
                    trajectory[-1][5] = possible_next_coords[0][4]

                    # add the lowest cost possible_next_coords to the trajectory list
                    trajectory.append(possible_next_coords[0][:-1])

                    # mark the chosen possible_next_coords position as visited
                    closed[possible_next_coords[0][1]
                           ][possible_next_coords[0][2]] = 1

        if self.debug_level > 1:
            self.printd("coverage_search", "Heuristic: ", 2)
            print(heuristic)

            self.printd("coverage_search", "Closed: ", 2)
            print(closed)

            self.printd("coverage_search", "Policy: ", 2)
            self.print_policy_map(trajectory, [])

            self.printd("coverage_search", "Trajectory: ", 2)
            self.print_trajectory(trajectory)

        total_cost = self.calculate_trajectory_cost(trajectory)
        total_steps = len(trajectory)-1

        self.printd("coverage_search", "found: {}, total_steps: {}, total_cost: {}".format(
            not resign, total_steps, total_cost), 1)

        # Pack the standard response
        # trajectory:   [value , x, y, orientation, action_performed_to_get_here, next_action, current_state_]
        # res:          [Success?, trajectory, final_coverage_grid,total_cost_actions,total_steps]
        res = [not resign, trajectory, closed, total_cost, total_steps]

        return res

    # Find the shortest path between init and goal coords based on the A* Search algorithm
    def a_star_search_closest_unvisited(self, initial_pos, heuristic, map_object):

        # Create a reference grid for visited positions
        closed = np.zeros_like(self.map_grid)
        closed[initial_pos[0]][initial_pos[1]] = 1

        if self.debug_level > 1:
            self.printd("a_star_search_closest_unvisited",
                        "initial closed grid:", 2)
            print(closed)

        # Movement orientation at the A* visited positions
        orientation = np.full(
            (np.size(self.map_grid, 0), np.size(self.map_grid, 1)), -1)

        # Added the given A* initial position with its associated costs to the "open" list
        # "open" is a list of valid positions to expand: [[f, g, x, y]]
        # g: accumulated a* movement_cost (start it with 0 cost)
        # f: total cost = a*_movement_cost + heuristic_cost at given position
        # x,y: given position
        x = initial_pos[0]
        y = initial_pos[1]
        g = 0
        f = g + heuristic[x][y]
        open = [[f, g, x, y]]

        found = False  # Whether a unvisited position is found or not
        resign = False  # flag set if we can't find expand

        while not found and not resign:
            self.printd("a_star_search_closest_unvisited",
                        " open: {}".format(open), 2)

            # If there isn't any more positions to expand, the unvisited position could not be found, then resign
            if len(open) == 0:
                resign = True
                self.printd("a_star_search_closest_unvisited",
                            " Fail to find a path", 2)

            # Otherwise expand search again
            else:

                # Sort open positions list by total cost, in descending order and reverse it to pop the element with lowest total cost
                open.sort(key=lambda x: x[0])
                open.reverse()
                next = open.pop()

                # update current search x,y,g
                x = next[2]
                y = next[3]
                g = next[1]

                # Check if a unvisited position was found
                if self.coverage_grid[x][y] == 0:
                    found = True
                else:
                    # calculate the possible next coords
                    for i in range(len(self.movement)):
                        x_next = x + self.movement[i][0]
                        y_next = y + self.movement[i][1]

                        # Check if it is out of the map boundaries
                        if x_next >= 0 and x_next < len(self.map_grid) and y_next >= 0 and y_next < len(self.map_grid[0]):
                            # Check if this position was already visited or if it is a visitable position on the map
                            if closed[x_next][y_next] != 1 and self.map_grid[x_next][y_next] != 1:
                                if map_object.is_movement_allowed([x,y], [x_next,y_next]):
                                    g2 = g + self.a_star_movement_cost[i]
                                    f = g2 + heuristic[x_next][y_next]
                                    open.append([f, g2, x_next, y_next])
                                    closed[x_next][y_next] = 1
                                    orientation[x_next][y_next] = i

        # initialize the trajectory
        trajectory = []

        # If path was found, then build the trajectory.
        # x and y at this point represents the last position on the search, which is the unvisited position
        if found:

            # Add the last position to the trajectory list, with both actions as None (they will be set later)
            trajectory = [[0, x, y, orientation[x][y], None, None, self.state_]]

            # Add the initial orientation to the orientation matrix
            orientation[initial_pos[0]][initial_pos[1]] = initial_pos[2]

            # Go backwards from the unvisited position founded to this search inital position
            # the 0 postfix means that this variable referes to the predecessor position, as 
            # this process below starts from the final position to the start position of the A*
            while (x != initial_pos[0] or y != initial_pos[1]):
                # Calculate the path predecessor position
                x0 = x - self.movement[orientation[x][y]][0]
                y0 = y - self.movement[orientation[x][y]][1]
                # The predecessor orientation is the one on the orientation matrix
                o0 = orientation[x0][y0]
                # The predecessor action will be set in the next iteration (its the next action of one position before it)
                a0 = None

                # Compute the required action index to get from the predecessor position to current iteration position
                a = (trajectory[-1][3]-o0) % len(self.action)

                # Update the successor position "action_performed_to_get_here" with the current next_action
                trajectory[-1][4] = a

                # Add the predecessor position and actions to the trajectory list
                trajectory.append([0, x0, y0, o0, a0, a, self.state_])

                # update x and y with the predecessor position, for next iteration
                x = x0
                y = y0

            trajectory.reverse()

        if self.debug_level > 1:
            self.printd("a_star_search_closest_unvisited", "Heuristic: ", 2)
            print(heuristic)

            self.printd("a_star_search_closest_unvisited", "Orientation: ", 2)
            print(orientation)

            self.printd("a_star_search_closest_unvisited", "Policy: ", 2)
            self.print_policy_map(trajectory, [])

            self.printd("[a_star_search_closest_unvisited", "Trajectory: ", 2)
            self.print_trajectory(trajectory)

        total_cost = self.calculate_trajectory_cost(trajectory)
        total_steps = len(trajectory)-1

        self.printd("a_star_search_closest_unvisited", "found: {}, total_steps: {}, total_cost: {}".format(
            found, total_steps, total_cost), 1)

        # Pack the standard response
        # trajectory:   [value , x, y, orientation, action_performed_to_get_here, next_action, current_state_]
        # res:          [Success?, trajectory, final_coverage_grid,total_cost_actions,total_steps]
        res = [found, trajectory, None, total_cost, total_steps]
        return res

    # Merge the two given grids and return True if all visitable positions were visited
    def check_full_coverage(self, grid, closed):
        return np.all(np.copy(grid)+np.copy(closed))

    # Return the Manhattan heuristic at given target point
    def create_manhattan_heuristic(self, target_point):
        heuristic = np.zeros_like(self.map_grid)
        for x in range(len(heuristic)):
            for y in range(len(heuristic[0])):
                heuristic[x][y] = abs(x-target_point[0])+abs(y-target_point[1])
        return heuristic

    # Return the Chebyshev heuristic at given target point
    def create_chebyshev_heuristic(self, target_point):
        heuristic = np.zeros_like(self.map_grid)
        for x in range(len(heuristic)):
            for y in range(len(heuristic[0])):
                heuristic[x][y] = max(
                    abs(x-target_point[0]), abs(y-target_point[1]))
        return heuristic

    # Return the horizontal heuristic at given target point
    def create_horizontal_heuristic(self, target_point):
        heuristic = np.zeros_like(self.map_grid)
        for x in range(len(heuristic)):
            for y in range(len(heuristic[0])):
                heuristic[x][y] = abs(x-target_point[0])
        return heuristic

    # Return the vertical heuristic at given target point
    def create_vertical_heuristic(self, target_point):
        heuristic = np.zeros_like(self.map_grid)
        for x in range(len(heuristic)):
            for y in range(len(heuristic[0])):
                heuristic[x][y] = abs(y-target_point[1])
        return heuristic

    # Return the requested heuristic at the given target_point
    def create_heuristic(self, target_point, heuristic_type):
        heuristic = np.zeros_like(self.map_grid)
        for x in range(len(heuristic)):
            for y in range(len(heuristic[0])):
                if heuristic_type == HeuristicType.MANHATTAN:
                    heuristic[x][y] = abs(x-target_point[0]) + abs(y-target_point[1])
                elif heuristic_type == HeuristicType.CHEBYSHEV:
                    heuristic[x][y] = max(abs(x-target_point[0]), abs(y-target_point[1]))
                elif heuristic_type == HeuristicType.HORIZONTAL:
                    heuristic[x][y] = abs(x-target_point[0])
                elif heuristic_type == HeuristicType.VERTICAL:
                    heuristic[x][y] = abs(y-target_point[1])
                elif heuristic_type == HeuristicType.EUCLIDEAN:
                    heuristic[x][y] = np.sqrt((x-target_point[0])**2 + (y-target_point[1])**2)
                elif heuristic_type == HeuristicType.DIAGONAL_RIGHT:
                    heuristic[x][y] = abs((x + y) - (target_point[0] + target_point[1]))
                elif heuristic_type == HeuristicType.DIAGONAL_LEFT:
                    heuristic[x][y] = abs((x - y) - (target_point[0] - target_point[1]))
                elif heuristic_type == HeuristicType.DIAGONAL_BOTH:
                    diag_right = abs((x - y) - (target_point[0] - target_point[1]))
                    diag_left = abs((x + y) - (target_point[0] + target_point[1]))
                    heuristic[x][y] = min(diag_right, diag_left)
                elif heuristic_type == HeuristicType.HORIZONTAL_VERTICAL:
                    heuristic[x][y] = min(abs(x - target_point[0]), abs(y - target_point[1]))
                elif heuristic_type == HeuristicType.NULL:
                    heuristic[x][y] = 0
        return heuristic
    

    # Return the initial x, y and orientation of the current map grid
    def get_start_position(self, orientation=0):
        for x in range(len(self.map_grid)):
            for y in range(len(self.map_grid[0])):
                if(self.map_grid[x][y] == 2):
                    return [x, y, orientation]

    # Append the given trajectory to the main trajectory
    def append_trajectory(self, new_trajectory, algorithm_ref):
        # If already has trajectory positions, remove duplicated positions by joining actions
        # and add a special obs at the end of the new first position list
        if len(self.current_trajectory) > 0 and len(new_trajectory) > 0:
            # As each search is only dependent of the current position, the
            # "action_performed_to_get_here" of the last accumulated trajectory
            # element has to be copied to the first element of the new trajectory.
            new_trajectory[0][4] = self.current_trajectory[-1][4]

            # Add a special annotation to be shown at the policy map
            self.current_trajectory_annotations.append(
                [new_trajectory[0][1], new_trajectory[0][2], algorithm_ref])

            # remove the duplicated position
            self.current_trajectory.pop()

        # Add the computed path to the trajectory list
        for t in new_trajectory:
            self.current_trajectory.append(t)

    # Return the trajectory total cost
    def calculate_trajectory_cost(self, trajectory):
        cost = 0

        # Sum up the actions cost of each step
        for t in trajectory:
            if t[5] is not None:
                cost += self.action_cost[t[5]]
        return cost

    # Return a numpy array containing only the xy of the trajectory
    def get_xy_trajectory(self, trajectory):
        if type(trajectory) == list:
            if type(trajectory[0]) == list:
                return [t[1:3] for t in trajectory]
                # return np.array([t[1:3] for t in trajectory])
            return trajectory[1:3]
            # return np.array(trajectory[1:3])
        return []
        # return np.empty((0, 2))

    # Return the search results: [found?, total_steps, total_cost, trajectory, xy_trajectory]
    def result(self):
        found = self.state_ == PlannerStatus.FOUND
        total_steps = len(self.current_trajectory)-1
        total_cost = self.calculate_trajectory_cost(self.current_trajectory)
        xy_trajectory = self.get_xy_trajectory(self.current_trajectory)

        total_cost = total_cost / self.action_cost_mult # normalize cost

        res = [found, total_steps, total_cost,
               self.current_trajectory, xy_trajectory]
        return res

    # Print a summary of the searching results
    def show_results(self):
        self.printd("show_results",
                    "Presenting the current searching results:\n")
        self.printd("show_results",
                    "Final status: {}".format(self.state_.name))
        # The last element just point to the last trajectory position, it is not a step done.
        self.printd("show_results", "Total steps: {}".format(
            len(self.current_trajectory)-1))
        self.printd("show_results", "Total cost: {:.2f}".format(
            self.calculate_trajectory_cost(self.current_trajectory)))
        if self.debug_level > 0:
            self.print_trajectory(self.current_trajectory)
        self.print_policy_map()

    # Print trajectory data
    def print_trajectory(self, trajectory):
        self.printd("print_trajectory", "Trajectory data:\n")
        print("{}\t{}\t{}\t{}\t{}\t{}".format(
            "l_cost", "x", "y", "orient.", "act_in", "act_next"))
        for t in trajectory:
            print("{:.2f}\t{}\t{}\t{}\t{}\t{}\t{}".format(
                t[0], t[1], t[2], t[3], t[4], t[5], t[6].name))

    # Print a given map grid with regular column width
    def print_map(self, m):
        for row in m:
            s = "["
            for i in range(len(m[0])):
                if type(row[i]) is str:
                    s += row[i]
                else:
                    s += "{:.1f}".format(row[i])
                if i is not (len(m[0])-1):
                    s += "\t,"
                else:
                    s += "\t]"
            print(s)

    # Print helper function with the standarized printing structure
    # [function_name] message
    def printd(self, f, m, debug_level=0):
        if debug_level <= self.debug_level:
            print("["+f+"] "+m)




def plot_all_trajectories_with_info(target_map, trajectories, compare_tb, cp):
    # fig, axes = plt.subplots(4, 8, figsize=(22, 22))
    fig, axes = plt.subplots(3, 4, figsize=(22, 22))
    axes = axes.flatten()

    # Color for A* visited waypoints
    a_star_color = 'lightblue'
    
    movement = [[0,  1],  # up
                [-1, 1],  # up left
                [-1, 0],   # left
                [-1,-1],   # down left
                [0, -1],   # down
                [1, -1],   # down right
                [1,  0],   # right
                [1, 1]]   # up right
    
    action = [0, 1, 2, 3, 4, 5, 6, 7]

    for i, (trajectory, ax) in enumerate(zip(trajectories, axes)):
        heuristic = compare_tb[i][0]
        orientation = cp.movement_name[compare_tb[i][1]]
        action_cost_weight = compare_tb[i][2]
        steps = compare_tb[i][4]
        cost = compare_tb[i][5]
        params_str = f"Heuristic: {heuristic}, Weight: {action_cost_weight}"

        cmap = mpl.colors.ListedColormap(['w', 'k', 'gold', 'mediumseagreen', 'red'])
        norm = mpl.colors.BoundaryNorm([0, 1, 2, 3, 4, 5], cmap.N)
        
        target_map_ref = np.copy(target_map)

        for point in trajectory:
            if point[6] == PlannerStatus.NEARST_UNVISITED_SEARCH:  # A* visited waypoint
                target_map_ref[point[1], point[2]] = 3  # Custom color reference for A* visitation

        ax.imshow(target_map_ref, interpolation='none', cmap=cmap, norm=norm)
        
        for j in range(len(trajectory) - 1):
            x, y = trajectory[j][2], trajectory[j][1]
            mov_idx = (trajectory[j][3] + action[trajectory[j][5]]) % len(movement)
            mov = movement[mov_idx]
            arrow_color = a_star_color if trajectory[j][6] == PlannerStatus.NEARST_UNVISITED_SEARCH else 'blue'
            ax.arrow(x, y, mov[1], mov[0], width=0.05, color=arrow_color, length_includes_head=True)
        
        # Plot the inital orientation
        init_direction = np.array(movement[trajectory[0][3]])/2
        ax.arrow(trajectory[0][2]-init_direction[1]/2, trajectory[0][1]-init_direction[0]/2, init_direction[1], init_direction[0], width=0.1,
                color='deeppink', length_includes_head=True, head_length=0.2)
        
        # Add additional information
        ax.set_title(params_str + f"\nSteps: {steps}, Cost: {cost:.2f}", fontsize=10)
        ax.axis('off')

    plt.suptitle(f"Computed Trajectories", fontsize=16)
    plt.tight_layout(rect=[0, 0, 1, 0.97])
    plt.show(block=False)