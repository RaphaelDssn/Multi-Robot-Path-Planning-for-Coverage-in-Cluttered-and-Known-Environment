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
import matplotlib.pyplot as plt
from enum import Enum, auto


MAP_SIZE = 5

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

def create_heuristic(heuristic_type):
    target_point = (2,2)
    heuristic = np.zeros((MAP_SIZE,MAP_SIZE))
    for x in range(len(heuristic)):
        for y in range(len(heuristic[0])):
            if heuristic_type == HeuristicType.MANHATTAN:
                heuristic[x][y] = abs(
                    x-target_point[0]) + abs(y-target_point[1])
            elif heuristic_type == HeuristicType.CHEBYSHEV:
                heuristic[x][y] = max(
                    abs(x-target_point[0]), abs(y-target_point[1]))
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



############################################################################################################################


cp_heuristics = [HeuristicType.VERTICAL,
                 HeuristicType.HORIZONTAL,
                 HeuristicType.CHEBYSHEV,
                 HeuristicType.MANHATTAN,
                 HeuristicType.EUCLIDEAN,
                 HeuristicType.DIAGONAL_RIGHT,
                 HeuristicType.DIAGONAL_LEFT,
                 HeuristicType.DIAGONAL_BOTH,
                 HeuristicType.HORIZONTAL_VERTICAL,
                 HeuristicType.NULL]




fig, axes = plt.subplots(2, 5, figsize=(16, 8))
axes = axes.flatten()

vmin, vmax = 0, 4  #range of color scale
heatmaps = [] 

for i, heuristic_type in enumerate(cp_heuristics):
    heuristic = create_heuristic(heuristic_type)
    ax = axes[i]
    heatmap = ax.imshow(heuristic, cmap="viridis", origin="upper", vmin=vmin, vmax=vmax)
    heatmaps.append(heatmap)
    
    # annotate cells with values
    for x in range(MAP_SIZE):
        for y in range(MAP_SIZE):
            ax.text(y, x, f"{heuristic[x, y]:.1f}", ha="center", va="center",
                    color="white" if heuristic[x, y] < 2.2 else "black")
    
    # draw the grid
    for x in range(MAP_SIZE + 1):
        ax.plot([x - 0.5, x - 0.5], [0 - 0.5, MAP_SIZE - 0.5], color='lightgray', lw=0.5)  # vertical lines
    for y in range(MAP_SIZE + 1):
        ax.plot([0 - 0.5, MAP_SIZE - 0.5], [y - 0.5, y - 0.5], color='lightgray', lw=0.5)  # horizontal lines
    
    ax.set_title(heuristic_type)
    ax.axis("off")  # Turn off axis for cleaner appearance

# Adjust layout
plt.tight_layout()
fig.suptitle("Heuristics", fontsize=16, fontweight='bold')
cbar = fig.colorbar(heatmaps[0], ax=axes, location='bottom', fraction=0.05, pad=0.05, aspect=50)
cbar.set_label("Heuristic Value")
plt.show(block=False)





############################################################################################################################


# constants
cell_size = 1
directions = [
    (0, 1),   # Up
    (1, 1),   # Up-right
    (1, 0),   # Right
    (1, -1),  # Down-right
    (0, -1),  # Down
    (-1, -1), # Down-left
    (-1, 0),  # Left
    (-1, 1),  # Up-left
]

# Calculate costs for each direction
costs = []
for dx, dy in directions:
    distance = np.sqrt(dx**2 + dy**2) * cell_size
    angle = (np.arctan2(dy, dx) - np.pi / 2) % (2 * np.pi)  # adjust angle to make 0 upwards
    angle_cost = min(angle, 2 * np.pi - angle)  # smallest angle to 0
    cost = distance / cell_size + angle_cost / (np.pi / 4)
    costs.append(cost)

fig, ax = plt.subplots(figsize=(8, 8))

# 3x3 grid
for x in range(-1, 2):
    for y in range(-1, 2):
        rect = plt.Rectangle((x - 0.5, y - 0.5), 1, 1, fill=None, edgecolor="gray", linewidth=1)
        ax.add_patch(rect)

# arrows and annotate costs
idx = 0
for (dx, dy), cost in zip(directions, costs):
    start_x, start_y = 0, 0  # center of the grid
    end_x, end_y = dx, dy  # neighbor cell
    if idx == 0:
        ax.quiver(start_x, start_y, dx, dy, angles='xy', scale_units='xy', scale=1, color="blue", label="next movement")
    else:
        ax.quiver(start_x, start_y, dx, dy, angles='xy', scale_units='xy', scale=1, color="blue")
    # annotate the cost
    offset = 0.1  # text offset
    ax.text(
        end_x + offset * dx, 
        end_y + offset * dy, 
        f"{cost:.2f}", 
        color="black", 
        fontsize=12, 
        ha="center", 
        va="center"
    )
    idx += 1

ax.quiver(0, 0, 0, 0.5, angles='xy', scale_units='xy', scale=1, color="red", label="current orientation")

ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)
ax.set_aspect('equal', adjustable='box')
ax.set_xticks(range(-1, 2))
ax.set_yticks(range(-1, 2))
ax.set_title("Action Costs", fontsize=14)

legend = ax.legend(loc="upper right", fontsize=12, frameon=True)
legend.get_frame().set_edgecolor("gray")

plt.grid(True, which='both', color="lightgray", linestyle="--", linewidth=0.5)
plt.show()
