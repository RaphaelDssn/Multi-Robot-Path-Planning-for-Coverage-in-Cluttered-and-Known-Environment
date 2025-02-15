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

and source: https://github.com/rodriguesrenato/coverage-path-planning
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
import sys
import argparse
from tabulate import tabulate
import matplotlib.pyplot as plt

from utils import *
from map import Map
from darp_extended import DARP
import coverage_planner

class DARP_coverage():

    def __init__(self, config):
        self.x0s = config['x0s']
        self.num_agents = config['num_agents']
        self.step_size = config['step_size']
        self.cell2real_length = config['cell2real_length']
        self.robot_radius = config['robot_radius']
        self.dt = config['dt']

        self.x0s_discrete = convert_coord_syst_to_grid(self.x0s, self.cell2real_length)

        self.map = Map(cell_size=self.cell2real_length, robot_radius=self.robot_radius, visualization = False)
        self.x_real_len = self.map.map_x_max
        self.y_real_len = self.map.map_y_max
        # self.map.plot_map_init_poses(init_poses=self.x0s)
        self.grid_dim = self.map.get_grid_dim()

        self.obs_pos = convert_grid_to_Darp_obstacles(self.map.grid)
        self.vis = config['vis']
        self.astar = config['astar']
        self.MaxIterDARP = config['MaxIterDARP']

        # to introduce weighted DARP
        self.nep = False #nep=not equal portions
        self.portions = []  #portions for weighted DARP, sum needs to be 1
        



    def solve(self):

        self.solve_DARP(self.nep, self.portions)
        
        compare_tbs = self.solve_coverage_waypoints() # compare_tb: [heuristic, orientation, found?, total_steps, total_cost, trajectory]
        
        xs, time_list = self.solve_trajectory_generation()

        # figures
        # self.map.plot_map()
        # self.map.plot_map_init_poses(self.x0s)
        self.map.plot_assignement_map(self.areas)
        self.map.plot_navigation_map(self.x_ref, self.x0s_discrete)
        # self.map.plot_map_traj(trajectories=xs, nb_agents=self.num_agents)

        plt.show()  # plot all figures

        return xs, time_list



    def solve_DARP(self, notEqualPortions, given_portions): 
        # convert initial state position x0s to position in grid
        initial_positions = convert_coord_syst_to_DARP(self.x0s, self.grid_dim[1], self.cell2real_length)

        #solve DARP : Divide areas based on robots initial positions
        self.darp_instance = DARP(self.grid_dim[0], self.grid_dim[1], notEqualPortions, initial_positions, given_portions, self.obs_pos, self.vis,
                                  MaxIter=self.MaxIterDARP, TotMaxIterMult=3, CCvariation=0.1, randomLevel=0.0001, dcells=2,
                                  importance=False, astar_usage=self.astar, map_object = self.map, connectDistPenaltyOnly=False)

        self.DARP_success , self.iterations, self.darp_nb_cells_per_robot = self.darp_instance.divideRegions()


        if not self.DARP_success:
            # if no convergence for DARP, redo but with connectDistPenaltyOnly
            self.darp_instance = DARP(self.grid_dim[0], self.grid_dim[1], notEqualPortions, initial_positions, given_portions, self.obs_pos, self.vis,
                                      MaxIter=self.MaxIterDARP/2, TotMaxIterMult=12, CCvariation=0.1, randomLevel=0.0001, dcells=2,
                                      importance=False, astar_usage=self.astar, map_object = self.map, connectDistPenaltyOnly=True)

            self.DARP_success , iterations, self.darp_nb_cells_per_robot = self.darp_instance.divideRegions()
            self.iterations += iterations

        if not self.DARP_success:
            print("DARP: No solution found in ", self.iterations, " iterations")
            sys.exit(0)

        print("DARP completed")
        print("Number of iterations for convergence: ", self.iterations)

        self.areas = self.darp_instance.BinaryRobotRegions   # True : free space / False : obstacle
        self.areas = np.where(self.areas, 0, 1)              # 0    : free space / 1     : obstacle
        
        # place init pose
        for i, (x, y, _) in enumerate(self.x0s_discrete):  # Iterate over each robot's initial pose
            # Place a 2 at the robot's position on the corresponding grid
            self.areas[i, int(x), int(y)] = 2


        
    def solve_coverage_waypoints(self):
        # find waypoints for coverage

        # heuristics from coverage_planner.py
        cp_heuristics = [coverage_planner.HeuristicType.VERTICAL,
                         coverage_planner.HeuristicType.HORIZONTAL,
                         coverage_planner.HeuristicType.CHEBYSHEV,
                         coverage_planner.HeuristicType.MANHATTAN,
                         coverage_planner.HeuristicType.EUCLIDEAN,
                         coverage_planner.HeuristicType.DIAGONAL_RIGHT,
                         coverage_planner.HeuristicType.DIAGONAL_LEFT,
                         coverage_planner.HeuristicType.DIAGONAL_BOTH,
                         coverage_planner.HeuristicType.HORIZONTAL_VERTICAL,
                         coverage_planner.HeuristicType.NULL]
        
        action_cost_mults = [0.1, 0.25, 0.5, 0.75, 1.0, 1.5]
        test_show_each_result = False
        cp_debug_level = 0

        compare_tbs = []

        for idx, target_map in enumerate(self.areas):
            compare_tb = []

            cp = coverage_planner.CoveragePlanner(target_map)
            cp.set_debug_level(cp_debug_level)

            orientations = [discretize_angle(self.x0s[idx][2])] #initial robot orientation. Can find best orientation among multiple to define here

            # Iterate over each orientation with each heuristic
            idx_search = 0
            for heuristic in cp_heuristics:
                for orientation in orientations:    # find the best initial orientation if not provided
                    for action_cost_mult in action_cost_mults:

                        if test_show_each_result:
                            print("\n\nIteration[map:{}, cp:{}, initial_orientation:{}]".format(
                                "map", heuristic.name, orientation))

                        cp.start(initial_orientation=orientation, cp_heuristic=heuristic, action_cost_mult=action_cost_mult)
                        cp.compute(self.map)

                        if test_show_each_result:
                            cp.show_results()

                        res = [heuristic.name, orientation, action_cost_mult]
                        res.extend(cp.result())
                        compare_tb.append(res)

                        # compare_tb: [heuristic, orientation, action cost mult, found?, total_steps, total_cost, trajectory]
                        # trajectory: [value, x, y, orientation, action_performed_to_get_here, next_action, current_state_]

                        idx_search += 1

            
            # Sort by number of steps and action cost
            compare_tb.sort(key=lambda x: (x[4], x[5]))

            # Print the summary of results for the given map
            summary = [row[0:6] for row in compare_tb]
            for row in summary:
                row[5] = "{:.2f}".format(row[5]) # Format cost to 2 decimal
                row[1] = cp.movement_name[row[1]] # Convert movement index to movement names

            
            compare_tb_headers = ["Heuristic", "Orientation", "action mult", "Found?", "Steps", "Cost"]
            summary_tb = tabulate(summary, compare_tb_headers, tablefmt="pretty", floatfmt=".2f")
            # print(summary_tb)
            

            # plot all trajectories
            # all_trajectories = [row[6] for row in compare_tb]
            # coverage_planner.plot_all_trajectories_with_info(target_map, all_trajectories, compare_tb, cp)

            # Print the best path
            print("\nBest path: [initial orientation: {} ({}), coverage path Heuristic:{}, Action cost mult: {}, Found? {}, Steps: {}, overlapping: {}/100, Cost: {}]".format(
                  cp.movement_name[compare_tb[0][1]], compare_tb[0][1], compare_tb[0][0], compare_tb[0][2], compare_tb[0][3], compare_tb[0][4],
                  (compare_tb[0][4] - self.darp_nb_cells_per_robot[idx]) * 100.0/compare_tb[0][4], compare_tb[0][5]))

            compare_tbs.append(compare_tb[0])

        self.x_ref = [
            [
                [wp[0], 
                wp[1] * self.cell2real_length - self.x_real_len + self.cell2real_length / 2,
                wp[2] * self.cell2real_length - self.y_real_len + self.cell2real_length / 2] + wp[3:]
                for wp in compare_tb[6] # compare_tb[6] is the trajectory
            ]
            for compare_tb in compare_tbs
        ]

        return compare_tbs


    def solve_trajectory_generation(self):

        xs = [[agent_init_pose] for agent_init_pose in self.x0s]

        for agent_index in range(self.num_agents):
            for wp_index in range(len(self.x_ref[agent_index])):
                generated_path = self.generate_path_from_waypoint(agent_index, wp_index)
                xs[agent_index] = xs[agent_index] + generated_path

        xs = equalize_sublists_length(xs)

        time_list = np.arange(0, int(len(xs[0])*self.dt), self.dt)
        
        return xs, time_list



    def generate_path_from_waypoint(self, agent_index, wp_index):
        states = []
        indexes = [1,2]

        current_pose = None
        if wp_index == 0:
            current_pose = self.x0s[agent_index]
            indexes = [0,1]
        else:
            current_pose = self.x_ref[agent_index][wp_index-1]


        delta_x = self.x_ref[agent_index][wp_index][1] - current_pose[indexes[0]]
        delta_y = self.x_ref[agent_index][wp_index][2] - current_pose[indexes[1]]

        delta_dist = np.sqrt(delta_x**2 + delta_y**2)
        heading = np.arctan2(delta_y, delta_x)

        nb_timesteps = int(delta_dist / self.step_size)

        for i in range(nb_timesteps):

            x = current_pose[indexes[0]] + i * delta_x / nb_timesteps
            y = current_pose[indexes[1]] + i * delta_y / nb_timesteps

            state = [x,y, heading]
            states.append(state)

        return states


############################################################################################################################
    
def main(cell_dim, robot_radius, MaxIterDARP):

    #initial positions 

    # x0s = [[0.4, 0.4, np.pi/2]] # single robot test
    # x0s = [[-2.1, -1, np.pi/2], [-2.1, -1.1, -np.pi/2], [2.7, -1.3, -np.pi]]  # good init overlapping
    
    x0s = [[-2.1, -1, np.pi/2], [0.6, 0.2, -np.pi/2], [2.6, -1.2, -np.pi]]  # good init
    # x0s = [[-2.1, -1, np.pi/2], [-2.7, 0.9, -np.pi/2], [1, 1.2, -np.pi]] # good but tricky two left one right
    # x0s = [[1, 0, np.pi/2], [0.5, 0.5, -np.pi/2], [1.5, 0, -np.pi]]   #all in middle :good with cell size 0.6
    # x0s = [[2.7, 0, np.pi/2], [2.7, 0.52, -np.pi/2], [2.7, -0.5, -np.pi]]    #all right
    # x0s = [[0, -1, np.pi/2], [0.6, 0.2, -np.pi/2], [2.7, -1.3, -np.pi]] # bad init
    # x0s = [[-0.2, -1, np.pi/2], [0.1, -1.3, -np.pi/2], [0.6, -0.2, -np.pi]]    #all middle down

    # x0s = [[0.3, 0.2, np.pi/2], [0.6, 0.2, -np.pi/2], [0.6, -0.2, -np.pi]]   #all in middle :good with cell size 0.5
    # x0s = [[0, -1, np.pi/2], [0.6, -0.2, -np.pi/2], [2.7, -1.1, -np.pi]] # bad init for cell size 0.2

    
    # x0s = [[-0.2, -0.8, np.pi/2], [0.1, -1.3, -np.pi/2], [-0.2, -1.3, np.pi/2], [0.1, -0.8, -np.pi/2], [0.6, -0.2, -np.pi]]    #all middle down
    # x0s = [[-2.5, -0.8, np.pi/2], [0.1, -1.3, -np.pi/2], [2.5, 1, -np.pi/2], [0.6, -0.2, -np.pi], [-2.5, 1, -np.pi], [2.5, -1, -np.pi/2]]    #many good init
    # x0s = [[-2.5, -0.8, np.pi/2], [0.1, -1.3, -np.pi/2], [2.5, 1, -np.pi/2], [0.6, -0.2, -np.pi], [-2.5, 1, -np.pi], [2.7, -1, -np.pi/2], [-1, -1, -np.pi/2]]    #many good init
    # x0s = [[-0.2, -0.8, np.pi/2], [0.2, -1.3, -np.pi/2], [-0.2, -1.3, np.pi/2], [2.5, 1, -np.pi/2], [0.6, -0.2, -np.pi], [-2.5, 1, -np.pi]]    #many all middle down


    # configuration dictionary
    config = {
        'step_size': 0.02,  # step size in [m] for trajectory planning
        'dt': 0.02,         # timestep for traj planning
        'x0s': x0s,         # initial positions
        'num_agents': len(x0s),
        'vis': True,        # extra visualization for cell iterative assignement
        'astar': True,      # use A* in DARP
        'cell2real_length': cell_dim,   # robot's workspace in [m]
        'robot_radius': robot_radius,   # robot's footprint in [m]
        'MaxIterDARP' : MaxIterDARP
    }

    # instance of the DARP_coverage class
    mCPP = DARP_coverage(config)

    # trajectory planning
    traj, times = mCPP.solve()
    # print(traj)
    # print(times)

    return traj, times


if __name__ == "__main__":

    cell_dim = 0.5
    robot_radius = 0.1
    MaxIterDARP = 2000

    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '-cell',
        default=cell_dim,
        type=float,
        nargs=1,
        help='cell dimension (side length) [in m] (default: ' + str(cell_dim) + ')')
    argparser.add_argument(
        '-robot_radius',
        default=robot_radius,
        type=float,
        nargs=1,
        help='Robots radius [in m] (default: ' + str(robot_radius) + ')')
    argparser.add_argument(
        '-MaxIter',
        default=MaxIterDARP,
        type=int,
        nargs=1,
        help='covering factor: Maximum number of iteration for DARP (default: ' + str(MaxIterDARP) + ')')
    args = argparser.parse_args()

    try:
        cell_dim = args.cell[0]
    except:
        cell_dim = args.cell

    try:
        cover_pourcentage = args.robot_radius[0]
    except:
        cover_pourcentage = args.robot_radius

    try:
        MaxIterDARP = args.MaxIter[0]
    except:
        MaxIterDARP = args.MaxIter


    main(cell_dim, robot_radius, MaxIterDARP)