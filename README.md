# Multi-Robot Path Planning for Coverage in Cluttered and Known Environment

Semester Project 2024 at SYCAMORE, EPFL under the supervision of Kai Ren and prof. Maryam Kamgarpour.

## Motivation

**DARP** [1] iteratively divides the environment into subareas based on the robots' initial positions, allowing **spanning tree coverage** (STC) for each robot independently. However, this method is ineffective in cluttered environments because **STC** relies on cell subdivision, which narrow passages can obstruct, and **DARP** fails to account for obstacles in its distance metrics, reducing its efficiency in complex areas.


## Previous Work

**A\* DARP** [2] extends **DARP** [1] by incorporating obstacle awareness, replacing Euclidean distance with **A\* path length** as the distance metric between cells.

An **&epsilon;\* based greedy single robot coverage path planner** (CPP) [3] was developped. This algorithm does not require cell subdivision, making it more suitable for cluttered environments than STC.




## Contribution

The contribution is detailled in the [report](report.pdf).

### Environment

In this project, the environment is defined as cluttered if it includes narrow passages with a smaller width than the width of a two grid cell (minimal width required for STC). The grid cell length represents the robot’s workspace, which may differ from its actual footprint.

Although the environment is represented as a grid, each cell maintains a **list of connected neighbors**. Two free neighboring cells may not be connected if a thin obstacle separates them (see image below).

The **Movement Map** is generated based on obstacles, the robot's footprint, and workspace (cell size) diameters. Obstacles are defined using the [Shapely Python library](https://shapely.readthedocs.io/en/2.0.6/reference/shapely.intersects.html) and can be modified in the `map.py` script.


<p align="center">
  <img src="images/movement_map.png" alt="Movement Map" width="400">
</p>



### Multi-robot coverage


#### **A\* DARP Extension**  
To assign cells to robots, two terms are computed:  
1. **Fair area division:** Balances assignments based on the number of cells per robot and their distance from the robot’s initial position.  
2. **Connected subareas:** Penalizes cells close to unconnected assigned cells to ensure all subareas are connected.

In the original DARP [1], both terms use Euclidean distances. We extend the implementation to **A\* DARP** [2], replacing Euclidean distance with **A\* path length** in the area division term and introducing **Breadth-First Search (BFS)** in the connectivity term to account for the Movement Map.

#### **&epsilon;\*-based CPP Extension**  
The &epsilon;\*-based coverage search [3] is a greedy approach that assigns cells by minimizing a cost function. If a dead-end is reached (no uncovered neighbor and incomplete coverage), an **A\* search** finds the nearest uncovered cell. The cost function combines:  
- **Action cost:** Penalizes turns.  
- **Heuristic:** Guides coverage (arbitrary cost).  

We extend the implementation by:
- Incorporating the **Movement Map** into the searches.
- Adding **diagonal motions** to the search and cost computations.  
- Introducing **six new heuristics**, expanding from the original four.

The final path is selected from **55 searches**, each with different heuristics and cost weightings. The chosen path is the one with the **lowest overlapping**. Coverage paths are planned for each robot independently.




## Installations

#### Install
```
git clone https://github.com/RaphaelDssn/Multi-Robot-Path-Planning-for-Coverage-in-Cluttered-and-Known-Environment.git
```


#### Requirements

This project was created in Python3 (3.12.3) using following library versions:

* numpy (2.1.3)
* tabulate (0.9.0)
* matplotlib (3.10.0)
* shapely (2.0.7)
* pygame (2.6.1)
* scikit-learn (1.6.1)
* numba (0.61.0)

Install the requirements using:
```
pip install -r requirements.txt
```



## Usage

### Execution

To run the area divison and path planning, run:

```
cd Multi-Robot-Path-Planning-for-Coverage-in-Cluttered-and-Known-Environment
python3 main.py
```

### Output

This execution should produce following area division and path planning:


<p align="center">
  <img src="images/area_division.png" alt="Area Division" width="400">
</p>


<p align="center">
  <img src="images/path_planned.png" alt="Path Planned" width="400">
</p>


### Options

<div style="text-align: left;">
  <table style="width: auto; margin-left: auto; margin-right: auto;">
    <tr>
      <th>Option</th>
      <th>Description</th>
      <th>Default Value</th>
    </tr>
    <tr>
      <td><code>-h, --help</code></td>
      <td>Show help message and exit</td>
      <td>N/A</td>
    </tr>
    <tr>
      <td><code>-cell</code></td>
      <td>Cell dimension (side length) [in meters]</td>
      <td><code>0.5</code></td>
    </tr>
    <tr>
      <td><code>-robot_radius</code></td>
      <td>Robot radius [in meters]</td>
      <td><code>0.1</code></td>
    </tr>
    <tr>
      <td><code>-MaxIter</code></td>
      <td>Maximum number of iterations for DARP</td>
      <td><code>2000</code></td>
    </tr>
  </table>
</div>

### Modifications

Modify the environment in ```map.py``` and the robot's initial position and orientation in ```main.py```in the [config dictionnary](main.py#L308).


### Visualization

To visualize the action cost and the heuristics, run:

```
python3 plot_heuristic_and_costs.py
```



## References

See the [report](report.pdf) for the complete bibliography.

[1] Alice-St. Alice-st/darp. https://github.com/alice-st/DARP, n.d. Accessed: 21 December 2024.

[2] Yufan Huang, Man Li, and Tao Zhao. A multi-robot coverage path planning algorithm based on improved DARP algorithm, 2023. Available at https://arxiv.org/abs/2304.09741.

[3] Rodriguesrenato. Rodriguesrenato/coverage-path-planning: A coverage path planning algorithm that combines multiple search algorithms to find a full coverage trajectory with the lowest cost. https://github.com/rodriguesrenato/coverage-path-planning, n.d. Accessed: 21 December 2024.



## License

- [1] is covered under the [Creative Commons Attribution-NonCommercial 4.0 International License](http://creativecommons.org/licenses/by-nc/4.0/)
- [3] is covered under the MIT License

This work is therefore also covered by the [Creative Commons Attribution-NonCommercial 4.0 International License](http://creativecommons.org/licenses/by-nc/4.0/).


## Cite as
```
@techreport{mCPP_cluttered,
  author      = {Raphaël Dousson},
  title       = {Multi-Robot Path Planning for Coverage in Cluttered and Known Environment},
  year        = {2024},
  type        = {Semester Project},
  institution = {Sycamore Lab, EPFL},
  url         = {https://github.com/RaphaelDssn/Multi-Robot-Path-Planning-for-Coverage-in-Cluttered-and-Known-Environment},
  note        = {Supervised by Kai Ren and Prof. Maryam Kamgarpour}
}
```