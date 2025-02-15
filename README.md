# Multi-Robot Path Planning for Coverage in Cluttered and Known Environment

## Motivation

DARP [1] divides in an iterative manner the areas based on the robots initial positions on a known environment to compute a single robot spanning tree coverage (STC) on each subarea. This approach does not suit cluttered environment as STC requires cell subdivision and DARP does not take obstacles into account in the distance metrics.


## Previous Work

DARP was extended to A* DARP [2] to take the obstacles into account, replacing the euclidean distance by the A* path length as distance metric between two cells.

An &epsilon;* based greedy single robot coverage path planner (CPP) [3] was developped. It does not require cell subdivision and is used to replace the STC initially used by DARP and A* DARP.


## Contribution

The contribution is detailled in the [report](report.pdf).

### Environment

The environment is considered as cluttered if narrow passages have the same width as the length of a grid cell. This grid cell length represents the work area of the robot and can be different than its actual footprint.

The environment is represented by a grid but each cell has connected cells list, because two free neighbour cells are not necessary connected, as they can be separated by a thin obstacle. The Movement Map below is computed based on the obstacles, robot's footprint and workspace (cell size) diameters. The obstacles are built using the [Shapely python library](https://shapely.readthedocs.io/en/2.0.6/reference/shapely.intersects.html) and can be changed in the ```map.py``` script.

<p align="center">
  <img src="images/movement_map.png" alt="Movement Map" width="400">
</p>



### Multi-robot coverage


A* DARP and &epsilon;* based CPP are extended to incorporate the introduced Movement Map. To assign a cell to a robot, two terms are computed. The first term ensures a fair work division across the robots, based on the number of cells assigned to each robot and the distance of the cell to the robot's initial position. The second term ensure connected subareas, based on the distance of cells to connected and unconnected cells for a each robot. DARP [1] uses euclidean distance in both terms. The DARP implementation [1] is extended to A* DARP [2] to replace the euclidean distance by A* path length in the first term. We introduce breath first search (BFS) in the second term to take the Movement Map into account.

&epsilon;* based CPP is a greedy coverage search that minimizes a cost for each new cell. An A* search to find the closest uncovered cell is perform in case of a dead-end (no uncovered neighbor cell and coverage incomplete). The cost is the weighted sum of an action cost penalizing turns and a heuristic. We extended the implementation to take the Movement Map into account. We implemented diagonal motions into the search and the costs and introduced six heuristics in addition to the initial four. Finally, the path planned is selected as the best one (lowest overlapping) among 55 searches (different heuristics and cost weighting). The path is planned for each robot individually.



## Installations

#### Install
```
git clone https://github.com/RaphaelDssn/Multi-Robot-Path-Planning-for-Coverage-in-Cluttered-and-Known-Environment.git
```


#### Requirements

This project was created using following library versions:

* Python3 (3.12.3)

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

Run following in a terminal: 

```
cd Multi-Robot-Path-Planning-for-Coverage-in-Cluttered-and-Known-Environment
python3 main.py
```

This execution should produce following area division and path planning:


<p align="center">
  <img src="images/area_division.png" alt="Area Division" width="400">
</p>


<p align="center">
  <img src="images/path_planned.png" alt="Path Planned" width="400">
</p>



Following options are available:

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


Modify the environment in ```map.py``` and the cell size, the robot's diameter, initial position and orientation in ```main.py```in the [config dictionnary](main.py#L308).


## References

See the [report](report.pdf) for a complete bibliography.

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