"""
Raphaël Dousson, SYCAMORE, EPFL, 2024
"""

import numpy as np

x_real_len = 3
y_real_len = 1.5


def discretize_angle(angle_rad):
        
    angle_rad = angle_rad % (2 * np.pi) # normalize angle
    discretized_angle = int((np.round(angle_rad / (np.pi / 4)) - 2) % 8)  # discretize the angle into 8 sectors
    
    return discretized_angle


def convert_coord_syst_to_grid(state, cell2real_length):
    real2cell_length = 1.0 / cell2real_length

    return [
        [
            int(real2cell_length * (pose[0] + x_real_len)),  # x_ccell
            int(real2cell_length * (pose[1] + y_real_len)),  # y_ccell
            np.round(np.mod(pose[2], 2 * np.pi) / (np.pi / 4)) * (np.pi / 4)  # o_ccell
        ]
        for pose in state
    ]


def convert_coord_syst_to_DARP(state, y_max, cell2real_length):
    real2cell_length = 1.0 / cell2real_length

    return [
        int(real2cell_length * (pose[0] + x_real_len)) * y_max + int(real2cell_length * (pose[1] + y_real_len))
        for pose in state
    ]


def convert_grid_to_Darp_obstacles(grid):
    
    converted_grid = []

    for i in range(grid.shape[0]):      # width
        for j in range(grid.shape[1]):  # height
            if grid[i,j] == 1:          # obstacle
                converted_grid.append(j*grid.shape[0]+i)

    return converted_grid


def equalize_sublists_length(lst):
   # equalize the length of the path of each robot with the goal pose
   max_length = max(len(sublist) for sublist in lst)

   for sublist in lst:
       while len(sublist) < max_length:
           sublist.append(sublist[-1])

   return lst


# test utils functions:
if __name__ == '__main__':
    a = 0
    for i in range(20):
        b = discretize_angle(a)
        print(a, " : ", b)
        a += np.pi/10.0


