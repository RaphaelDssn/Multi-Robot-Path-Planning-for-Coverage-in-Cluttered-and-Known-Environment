"""
Raphaël Dousson, SYCAMORE, EPFL, 2024
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import matplotlib.colors as mcolors
from matplotlib.patches import Polygon
from shapely.geometry import Polygon as ShapelyPolygon
from shapely.geometry import Point

import coverage_planner


class Map:
    def __init__(self, cell_size=0.3, robot_radius=0.1, visualization=False):

        self.robot_radius = robot_radius
        self.map_y_min = -1.5
        self.map_y_max = 1.5
        self.map_x_min = -3.0
        self.map_x_max = 3.0
        
        self.cell_size = cell_size

        self.grid_dim_x = 0
        self.grid_dim_y = 0
        self.grid = np.array([])
        
        
        # x and y axis coordinates for obstacles
        self.ox = []
        self.oy = []

        #----------------------------------------------------------------------------------
        #------------------------------For custom Environment------------------------------
        # Comment following:
        self.initialise_obstacles()

        # to initialize custom obstacles:
        # list of sublists (for each obstacles), add first point also at the end for each obstacles

        # example with:
        # square of coordinates: (0,0), (0,-1), (-1,-1), (-1,0) and
        # triangle of coordinates: (3,-1), (2,1), (1,0)

        # Uncomment following:
        """
        self.ox = [[0.0, 0.0, -1.0, -1.0, 0.0], [3, 2, 1, 3]]
        self.oy = [[0.0, -1.0, -1.0, 0.0, 0.0], [-1, 1, 0, -1]]
        """
        #----------------------------------------------------------------------------------


        self.initialize_grid()
        self.mark_obstacles_on_grid()
        
        # self.plot_map()
        # print(self.grid)
        
        # movement directions (8 possible directions)
        self.movements = np.array([(-1, 0), (1, 0), (0, -1), (0, 1),  # left, right, down, up
                                   (-1, -1), (-1, 1), (1, -1), (1, 1)])  # diagonal movements
        
        self.movement_map = self.generate_movement_map()

        self.robot_base_colors = ['blue', 'red', 'green', 'orange', 'purple', 'cyan', 'brown']

        if visualization:
            self.plot_movement_map()
        

    def initialise_obstacles(self):
        #obstacles exact positions
        self.add_line([-1.498, 2.998], [0.001, 3.000])
        self.add_line([1.051, 3.001], [1.494, 3.000])
        self.add_line([1.494, 3.000], [1.493, 0.430])
        self.add_line([1.494, -0.374], [1.497, -2.998])
        self.add_line([0.002, -2.999], [1.497, -2.998])
        self.add_line([-1.498, -2.999], [-1.050, -2.999])
        self.add_line([-1.496,-0.500], [-1.498, -2.999])
        self.add_line([-1.496, 0.750], [-1.495, 0.299])
        self.add_line([-1.498, 2.998], [-1.498, 1.553])
        self.add_line([-0.481, 2.382], [0.879, 1.356])
        self.add_line([-1.498, 1.553], [-0.700, 1.551])
        self.add_line([1.018, 0.429], [1.493, 0.430])
        self.add_line([0.141, 1.040], [-0.269, 0.524])
        self.add_line([-1.496, 0.526], [-0.269, 0.524])
        self.add_line([-0.269, 0.524], [-0.261, -0.008])
        self.add_line([-0.261,-0.008], [0.480, -0.008])
        self.add_line([0.011, -0.008], [0.011, -0.486])
        self.add_line([-1.496, -0.859], [-0.492, -0.860])
        self.add_line([0.922, -0.613], [0.924, -2.093])
        self.add_line([0.260, -1.084], [0.260, -2.093])
        self.add_line([-0.665, -2.094], [0.924, -2.093])
        self.add_line([-0.685, -2.103], [-0.931, -2.414])


    def add_line(self,p0,p1):
        p0_rotated = [-p0[1], p0[0]]
        p1_rotated = [-p1[1], p1[0]]

        p0 = np.array(p0_rotated)
        p1 = np.array(p1_rotated)

        dir = (p1 - p0) / np.linalg.norm((p1 - p0))
        dirperp = np.array([-dir[1], dir[0]])

        point_mm = p0 - 0.025 * dir - 0.025 * dirperp
        point_mp = p0 - 0.025 * dir + 0.025 * dirperp
        point_pm = p1 + 0.025 * dir - 0.025 * dirperp
        point_pp = p1 + 0.025 * dir + 0.025 * dirperp
        self.ox.append([point_mm[0], point_mp[0], point_pp[0], point_pm[0], point_mm[0]])
        self.oy.append([point_mm[1], point_mp[1], point_pp[1], point_pm[1], point_mm[1]])

    
    def initialize_grid(self):
        """Initialize a grid to represent the environment"""
        x_range = self.map_x_max - self.map_x_min
        y_range = self.map_y_max - self.map_y_min
        
        self.grid_dim_x = int(np.ceil(x_range / self.cell_size))
        self.grid_dim_y = int(np.ceil(y_range / self.cell_size))
        
        self.grid = np.zeros((self.grid_dim_y, self.grid_dim_x), dtype=int)
    

    def mark_obstacles_on_grid(self):
        """Mark grid cells that overlap with obstacles."""
        for ox, oy in zip(self.ox, self.oy):
            # Create a Shapely polygon for each obstacle
            obstacle_polygon = ShapelyPolygon(zip(ox, oy))

            for i in range(self.grid_dim_y):
                for j in range(self.grid_dim_x):
                    # Center of the current grid cell
                    x_center = self.map_x_min + (j + 0.5) * self.cell_size
                    y_center = self.map_y_min + (i + 0.5) * self.cell_size

                    # Define the circular subcell
                    subcell_radius = self.robot_radius
                    if self.robot_radius > self.cell_size:
                        subcell_radius = self.cell_size / 2  # Use half of the cell size if the radius is larger
                    subcell_circle = Point(x_center, y_center).buffer(subcell_radius)

                    # Check for intersection between the obstacle and the circular subcell
                    if obstacle_polygon.intersects(subcell_circle):
                        self.grid[i, j] = 1


    def check_path_intersection(self, start_pos, goal_pos):
        """
        Check if a path between start_pos and goal_pos intersects or partially overlaps with any obstacle using Shapely.
        """
        # Compute the direction vector of the line
        dx, dy = goal_pos[0] - start_pos[0], goal_pos[1] - start_pos[1]
        length = np.sqrt(dx**2 + dy**2)
        
        # Normalize the direction vector
        dx /= length
        dy /= length
        
        # extend star and goal pose by the robots radius (triangular shaped)
        start_pic = (start_pos[0] - dx*self.robot_radius, start_pos[1] - dy*self.robot_radius)
        goal_pic = (goal_pos[0] + dx*self.robot_radius, goal_pos[1] + dy*self.robot_radius)

        # Perpendicular vector for path width
        perp_dx = -dy * self.robot_radius
        perp_dy = dx * self.robot_radius

        # Compute the rectangle corners for the path (the area the robot sweeps)
        path_corners = [
            (start_pos[0] + perp_dx, start_pos[1] + perp_dy),
            start_pic,
            (start_pos[0] - perp_dx, start_pos[1] - perp_dy),
            (goal_pos[0] - perp_dx, goal_pos[1] - perp_dy),
            goal_pic,
            (goal_pos[0] + perp_dx, goal_pos[1] + perp_dy)
        ]

        # Create Shapely polygon for the path
        path_polygon = ShapelyPolygon(path_corners)
        obstacles_to_plot = []

        # Check intersections with obstacles
        for ox, oy in zip(self.ox, self.oy):
            # Create Shapely polygon for the obstacle
            obstacle_polygon = ShapelyPolygon(zip(ox, oy))
            obstacles_to_plot.append(obstacle_polygon)

            # Check if path_polygon intersects with the obstacle path
            if path_polygon.intersects(obstacle_polygon):
                return True

        return False


    def plot_path_and_obstacles(self, path_polygon, obstacle_polygons, highlight_obstacle=None, start_pose=None, goal_pose=None):
        """
        Plot the path polygon and all obstacles.
        """
        fig, ax = plt.subplots()
        ax.set_aspect('equal', adjustable='box')

        # Plot the cells considered as obstacles
        for i in range(self.grid_dim_y):
            for j in range(self.grid_dim_x):
                if self.grid[i, j] == 1:
                    x_min = self.map_x_min + j * self.cell_size
                    y_min = self.map_y_min + i * self.cell_size
                    rect = plt.Rectangle(
                        (x_min, y_min), self.cell_size, self.cell_size, 
                        color='lightgray'
                    )
                    ax.add_patch(rect)

        # Plot the grid (y-axis)
        for i in range(self.grid_dim_y + 1):
            y = self.map_y_min + i * self.cell_size
            ax.plot([self.map_x_min, self.map_x_max], [y, y], 'gray', linewidth=0.5)

        # Plot the grid (x-axis)
        for j in range(self.grid_dim_x + 1):
            x = self.map_x_min + j * self.cell_size
            ax.plot([x, x], [self.map_y_min, self.map_y_max], 'gray', linewidth=0.5)

        # Plot the start and goal poses as blue dots
        if start_pose:
            start_circle = plt.Circle(start_pose, radius=0.1, color='blue', label='Start Pose')
            ax.add_patch(start_circle)
        if goal_pose:
            goal_circle = plt.Circle(goal_pose, radius=0.1, color='green', label='Goal Pose')
            ax.add_patch(goal_circle)

        # Plot the path polygon
        path_patch = Polygon(list(path_polygon.exterior.coords), closed=True, edgecolor='blue', facecolor='cyan', alpha=0.7, label='Path Polygon')
        ax.add_patch(path_patch)

        # Plot all obstacle polygons
        for obstacle_polygon in obstacle_polygons:
            color = 'red' if obstacle_polygon == highlight_obstacle else 'black'
            label = 'Intersecting Obstacle' if obstacle_polygon == highlight_obstacle else 'Obstacle'
            obstacle_patch = Polygon(list(obstacle_polygon.exterior.coords), closed=True, edgecolor='black', facecolor=color, alpha=0.7, label=label)
            ax.add_patch(obstacle_patch)

        # remove duplicate labels
        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        plt.legend(by_label.values(), by_label.keys())

        ax.set_xlim(self.map_x_min, self.map_x_max)
        ax.set_ylim(self.map_y_min, self.map_y_max)

        ax.set_title("Path and Obstacles Intersection Check")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")

        plt.pause(0.1)
        plt.show(block=True)



    def generate_movement_map(self):
        movement_map = np.zeros((self.grid_dim_y, self.grid_dim_x, len(self.movements)), dtype=bool)

        # for each cell in the grid, check if movement to its neighbors is possible
        for i in range(self.grid_dim_y):
            for j in range(self.grid_dim_x):
                current_cell = (j * self.cell_size - self.map_x_max + self.cell_size / 2, i * self.cell_size - self.map_y_max + self.cell_size / 2)

                for k, (dx, dy) in enumerate(self.movements):
                    neighbor_x = j + dx
                    neighbor_y = i + dy
                    
                    # check if the neighbor is within bounds
                    if 0 <= neighbor_x < self.grid_dim_x and 0 <= neighbor_y < self.grid_dim_y:
                        # check if current cell not obstacle (1) and neighbor neither
                        if self.grid[i, j] != 1 and self.grid[neighbor_y, neighbor_x] != 1:
                            # center of the neighbor cell
                            neighbor_cell = (neighbor_x*self.cell_size - self.map_x_max + self.cell_size/2.0, neighbor_y*self.cell_size - self.map_y_max + self.cell_size/2.0)
                            
                            # check if the path from the current cell to the neighbor cell intersects with obstacles
                            if not self.check_path_intersection(current_cell, neighbor_cell):
                                movement_map[i, j, k] = True  # Mark that movement is possible
                
        return movement_map



    def is_movement_allowed(self, start_pos, goal_pos):
        """
        Check if movement from start_pos to goal_pos is allowed in the movement_map.
        """
        dx = goal_pos[0] - start_pos[0]
        dy = goal_pos[1] - start_pos[1]

        # find corresponding movement index
        matching_indices = np.where((self.movements[:, 0] == dx) & (self.movements[:, 1] == dy))[0]

        if matching_indices.size == 0:
            # no matching movement found
            return False

        movement_idx = matching_indices[0] # first matching index

        # check the movement_map
        return self.movement_map[start_pos[1], start_pos[0], movement_idx]

    ############################################################################################################################

    def plot_movement_map(self):
        """
        Plot the movement map showing allowed movements between cells.
        """

        fig, ax = plt.subplots(figsize=(9, 5))
        grid_dim_y, grid_dim_x, _ = self.movement_map.shape

        # plot the cells considered as obstacles
        for i in range(self.grid_dim_y):
            for j in range(self.grid_dim_x):
                if self.grid[i, j] == 1:
                    x_min = self.map_x_min + j * self.cell_size
                    y_min = self.map_y_min + i * self.cell_size
                    rect = plt.Rectangle(
                        (x_min, y_min), self.cell_size, self.cell_size, 
                        color='lightgray'
                    )
                    ax.add_patch(rect)

        # plot the exact obstacles
        for i in range(len(self.ox)):
            plt.fill(self.ox[i],self.oy[i],'k')

        # plot the grid (y axis)
        for i in range(self.grid_dim_y + 1):
            y = self.map_y_min + i * self.cell_size
            ax.plot([self.map_x_min, self.map_x_max], [y, y], 'gray', linewidth=0.5)

        # plot the grid (x axis)
        for j in range(self.grid_dim_x + 1):
            x = self.map_x_min + j * self.cell_size
            ax.plot([x, x], [self.map_y_min, self.map_y_max], 'gray', linewidth=0.5)

        # Loop through each cell in the grid
        for i in range(grid_dim_y):
            for j in range(grid_dim_x):
                # Plot the current cell as a point
                x_center = j * self.cell_size - self.map_x_max + self.cell_size / 2
                y_center = i * self.cell_size - self.map_y_max + self.cell_size / 2
                ax.plot(x_center, y_center, 'ko', markersize=3)  # Black dot for the cell center

                # Loop through all movement directions
                for k, (dx, dy) in enumerate(self.movements):
                    # check if movement is allowed
                    if self.movement_map[i, j, k]:
                        # plot an arrow for the movement
                        ax.arrow(x_center, y_center, dx * self.cell_size * 0.8, dy * self.cell_size * 0.8, 
                                head_width=0.1 * self.cell_size, head_length=0.15 * self.cell_size, fc='blue', ec='blue')

        ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        ax.set_title("Movement Map")
        plt.tight_layout()
        plt.pause(1)
        plt.show(block=False)



    def get_grid_dim(self):
        return [self.grid_dim_x, self.grid_dim_y]
    
    def plot_map(self):
        fig, ax = plt.subplots()
        ax.set_aspect('equal', adjustable='box')
        plt.xlim(self.map_x_min, self.map_x_max)
        plt.ylim(self.map_y_min, self.map_y_max)
        
        # plot the cells considered as obstacles
        for i in range(self.grid_dim_y):
            for j in range(self.grid_dim_x):
                if self.grid[i, j] == 1:
                    x_min = self.map_x_min + j * self.cell_size
                    y_min = self.map_y_min + i * self.cell_size
                    rect = plt.Rectangle(
                        (x_min, y_min), self.cell_size, self.cell_size, 
                        color='lightgray'
                    )
                    ax.add_patch(rect)

        # plot the exact obstacles
        for i in range(len(self.ox)):
            plt.fill(self.ox[i],self.oy[i],'k')

        # plot the grid (y axis)
        for i in range(self.grid_dim_y + 1):
            y = self.map_y_min + i * self.cell_size
            ax.plot([self.map_x_min, self.map_x_max], [y, y], 'gray', linewidth=0.5)

        # plot the grid (x axis)
        for j in range(self.grid_dim_x + 1):
            x = self.map_x_min + j * self.cell_size
            ax.plot([x, x], [self.map_y_min, self.map_y_max], 'gray', linewidth=0.5)
        
        plt.pause(0.1)
        plt.show(block=False)


    def plot_map_init_poses(self, init_poses):
        fig, ax = plt.subplots()
        ax.set_aspect('equal', adjustable='box')
        plt.xlim(self.map_x_min, self.map_x_max)
        plt.ylim(self.map_y_min, self.map_y_max)
        
        # plot the cells considered as obstacles
        for i in range(self.grid_dim_y):
            for j in range(self.grid_dim_x):
                if self.grid[i, j] == 1:
                    x_min = self.map_x_min + j * self.cell_size
                    y_min = self.map_y_min + i * self.cell_size
                    rect = plt.Rectangle(
                        (x_min, y_min), self.cell_size, self.cell_size, 
                        color='lightgray'
                    )
                    ax.add_patch(rect)

        # plot the exact obstacles
        for i in range(len(self.ox)):
            plt.fill(self.ox[i],self.oy[i],'k')

        # plot the grid (y axis)
        for i in range(self.grid_dim_y + 1):
            y = self.map_y_min + i * self.cell_size
            ax.plot([self.map_x_min, self.map_x_max], [y, y], 'gray', linewidth=0.5)

        # plot the grid (x axis)
        for j in range(self.grid_dim_x + 1):
            x = self.map_x_min + j * self.cell_size
            ax.plot([x, x], [self.map_y_min, self.map_y_max], 'gray', linewidth=0.5)

        # plot robots init positions
        for i in range(len(init_poses)):
            ax.plot(init_poses[i][0], init_poses[i][1], 'ro')

        plt.pause(0.1)
        plt.show(block=False)



    def plot_map_traj(self, trajectories, nb_agents):
        fig, ax = plt.subplots()
        ax.set_aspect('equal', adjustable='box')
        plt.xlim(self.map_x_min, self.map_x_max)
        plt.ylim(self.map_y_min, self.map_y_max)
        
        # plot the cells considered as obstacles
        for i in range(self.grid_dim_y):
            for j in range(self.grid_dim_x):
                if self.grid[i, j] == 1:
                    x_min = self.map_x_min + j * self.cell_size
                    y_min = self.map_y_min + i * self.cell_size
                    rect = plt.Rectangle(
                        (x_min, y_min), self.cell_size, self.cell_size, 
                        color='lightgray'
                    )
                    ax.add_patch(rect)

        # plot the exact obstacles
        for i in range(len(self.ox)):
            plt.fill(self.ox[i],self.oy[i],'k')

        # plot the grid (y axis)
        for i in range(self.grid_dim_y + 1):
            y = self.map_y_min + i * self.cell_size
            ax.plot([self.map_x_min, self.map_x_max], [y, y], 'gray', linewidth=0.5)

        # plot the grid (x axis)
        for j in range(self.grid_dim_x + 1):
            x = self.map_x_min + j * self.cell_size
            ax.plot([x, x], [self.map_y_min, self.map_y_max], 'gray', linewidth=0.5)

        # plot robots trajectories
        for i in range(nb_agents):
            x_values = [state[0] for state in trajectories[i]]
            y_values = [state[1] for state in trajectories[i]]
            plt.plot(x_values, y_values, self.robot_base_colors[i%len(self.robot_base_colors)], marker = 'o', label=f'Robot {i}', markersize=1, linestyle='None')

        plt.title("Trajectory")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        plt.legend()
        plt.pause(0.1)
        plt.show()



    def adjust_color_brightness(self, color, factor=1.0):
        # adjust the brightness of a given color. return color in hex format.
        # convert color to RGB and scale by factor
        rgb = mcolors.to_rgb(color)
        adjusted_rgb = tuple(min(1, max(0, c * factor)) for c in rgb)
        return mcolors.to_hex(adjusted_rgb)

        
    def plot_rect_to_color(self, position, color):
        x = position[1]
        y = position[2]
        rect = plt.Rectangle(
            (x - self.cell_size / 2, y - self.cell_size / 2),
            self.cell_size, self.cell_size,
            color=color
        )
        return rect

    def plot_navigation_map(self, trajectories, init_poses, title = ""):
        _, ax = plt.subplots(figsize=(9, 5))
        ax.set_aspect('equal', adjustable='box')
        
        # plot the cells considered as obstacles
        for i in range(self.grid_dim_y):
            for j in range(self.grid_dim_x):
                if self.grid[i, j] == 1:
                    x_min = self.map_x_min + j * self.cell_size
                    y_min = self.map_y_min + i * self.cell_size
                    rect = plt.Rectangle(
                        (x_min, y_min), self.cell_size, self.cell_size, 
                        color='lightgray'
                    )
                    ax.add_patch(rect)

        # plot the grid (y axis)
        for i in range(self.grid_dim_y + 1):
            y = self.map_y_min + i * self.cell_size
            ax.plot([self.map_x_min, self.map_x_max], [y, y], 'gray', linewidth=0.5)

        # plot the grid (x axis)
        for j in range(self.grid_dim_x + 1):
            x = self.map_x_min + j * self.cell_size
            ax.plot([x, x], [self.map_y_min, self.map_y_max], 'gray', linewidth=0.5)


        # Define colors for the map and trajectories
        start_position_color = 'gold'
        start_orientation_color = 'black'
        status_color_ref = {  # Status colors
            coverage_planner.PlannerStatus.STANDBY: 'black',
            coverage_planner.PlannerStatus.COVERAGE_SEARCH: 'royalblue',
            coverage_planner.PlannerStatus.NEARST_UNVISITED_SEARCH: 'darkturquoise',
            coverage_planner.PlannerStatus.FOUND: 'mediumseagreen',
            coverage_planner.PlannerStatus.NOT_FOUND: 'red'
        }

        mode_brightness = {
            coverage_planner.PlannerStatus.NOT_FOUND: 0,
            coverage_planner.PlannerStatus.COVERAGE_SEARCH: 0.6,  # Darker
            coverage_planner.PlannerStatus.NEARST_UNVISITED_SEARCH: 2.0,  # Lighter
        }

        # Plot trajectories for each robot
        for robot_id, trajectory in enumerate(trajectories):
            base_color = self.robot_base_colors[robot_id % len(self.robot_base_colors)]

            # Start position
            rect_start = self.plot_rect_to_color(trajectory[0], start_position_color)
            ax.add_patch(rect_start)

            # End position
            rect_end = self.plot_rect_to_color(trajectory[-1], status_color_ref[coverage_planner.PlannerStatus.FOUND])
            ax.add_patch(rect_end)

            # Add trajectory arrows and mark status
            for i in range(len(trajectory) - 1):
                x = trajectory[i][1]
                y = trajectory[i][2]
                mov = [trajectory[i+1][1]-x, trajectory[i+1][2]-y]

                # Get the correspondent status color
                brightness = mode_brightness.get(trajectory[i][6], 0.0)
                arrow_color = self.adjust_color_brightness(base_color, factor=brightness)

                # Plot arrow for movement
                ax.arrow(x, y, mov[0], mov[1], width=0.1*self.cell_size, color=arrow_color, length_includes_head=True)

            # Plot the initial orientation of the robot
            ax.arrow(trajectory[0][1],
                     trajectory[0][2],
                     np.cos(init_poses[robot_id][2])*self.cell_size/3.0,
                     np.sin(init_poses[robot_id][2])*self.cell_size/3.0,
                     width=0.1*self.cell_size,
                     color=start_orientation_color,
                     length_includes_head=True)
        
        # plot the exact obstacles
        for i in range(len(self.ox)):
            plt.fill(self.ox[i],self.oy[i],'k')

        # legend
        legend_elements = [Line2D([0], [0], color=color, marker='o', label=f'Robot {i+1}')
                        for i, color in enumerate(self.robot_base_colors[:len(trajectories)])]
        legend_elements.append(Line2D([0], [0], marker='s', color='w', label='Obstacle',
                                    markerfacecolor='lightgray', markersize=15))
        legend_elements.append(Line2D([0], [0], marker='s', color='w', label='Init Pos',
                                    markerfacecolor=start_position_color, markersize=15))
        legend_elements.append(Line2D([0], [0], marker='s', color='w', label='End Pos',
                                    markerfacecolor=status_color_ref[coverage_planner.PlannerStatus.FOUND], markersize=15))
        legend_elements.append(Line2D([0], [1], color='black', marker='>', markersize=15, label="Init Orientation"))
        ax.legend(handles=legend_elements, loc='upper right')

        plt.title("Path Planned" + title)
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        plt.tight_layout()
        plt.pause(0.1)
        plt.show(block=False)


    def plot_assignement_map(self, areas):
        # Create a figure
        _, ax = plt.subplots(figsize=(9, 5))

        ax.set_aspect('equal', adjustable='box')

        # plot the cells considered as obstacles
        for i in range(len(areas[0])):
            for j in range(len(areas[0][i])):
                color = None
                if areas[0][i][j] == 1:  # obstacles
                    color = 'lightgray'
                if color is not None:
                    x_min = self.map_x_min + i * self.cell_size
                    y_min = self.map_y_min + j * self.cell_size
                    rect = plt.Rectangle((x_min, y_min), self.cell_size, self.cell_size, color=color)
                    ax.add_patch(rect)

        # plot assigned cells
        for agent in range(len(areas)):
            for i in range(len(areas[agent])):
                for j in range(len(areas[agent][i])):
                    color = None
                    if areas[agent][i][j] == 0:    # free space
                        color=self.robot_base_colors[agent%len(self.robot_base_colors)]
                    elif areas[agent][i][j] == 2:  # init pose
                        color = 'gold'
                    if color is not None:
                        x_min = self.map_x_min + i * self.cell_size
                        y_min = self.map_y_min + j * self.cell_size
                        rect = plt.Rectangle((x_min, y_min), self.cell_size, self.cell_size, color=color)
                        ax.add_patch(rect)

                    if areas[agent][i][j] == 2:  # init pose
                        circle_color = self.robot_base_colors[agent % len(self.robot_base_colors)]
                        x_center = self.map_x_min + (i + 0.5) * self.cell_size
                        y_center = self.map_y_min + (j + 0.5) * self.cell_size
                        # circle = plt.Circle((x_center, y_center), self.cell_size/4.0, color=circle_color)
                        circle = plt.Circle((x_center, y_center), self.robot_radius, facecolor=circle_color, edgecolor='black', linewidth=1)
                        ax.add_patch(circle)
        grid_color = 'gray'
        
        # plot the grid (y axis)
        for i in range(self.grid_dim_y + 1):
            y = self.map_y_min + i * self.cell_size
            ax.plot([self.map_x_min, self.map_x_max], [y, y], grid_color, linewidth=0.5)

        # plot the grid (x axis
        for j in range(self.grid_dim_x + 1):
            x = self.map_x_min + j * self.cell_size
            ax.plot([x, x], [self.map_y_min, self.map_y_max], grid_color, linewidth=0.5)

        # plot the exact obstacles
        for i in range(len(self.ox)):
            plt.fill(self.ox[i],self.oy[i],'k')

        # Add legend
        legend_elements = [Line2D([0], [0], marker='s', color='w', label=f'Robot {i+1}',
                                    markerfacecolor=color, markersize=15)
                                    for i, color in enumerate(self.robot_base_colors[:len(areas)])]
        legend_elements.append(Line2D([0], [0], marker='s', color='w', label='Obstacle',
                                    markerfacecolor='lightgray', markersize=15))
        legend_elements.append(Line2D([0], [0], marker='s', color='w', label='Init Pos',
                                    markerfacecolor="gold", markersize=15))
        ax.legend(handles=legend_elements, loc='upper right')

        # Add title and show the plot
        plt.title("Area Division")
        ax.set_xlabel("X [m]")
        ax.set_ylabel("Y [m]")
        plt.tight_layout()
        plt.pause(0.1)
        plt.show(block=False)



############################################################################################################################

if __name__ == '__main__':
    map_env = Map(cell_size=0.5)
    
    map_env.plot_movement_map()
    plt.pause(100)
    
