# Multi-Robot Path Planning for Coverage in Cluttered and Known Environment

## Motivation




## Previous Work

This project is based on [1] and [2]


## Contribution



## Installations

#### Requirements

This project was created using following library versions:

* Python3 (3.12.3)

* numpy (1.26.0)
* tabulate (0.9.0)
* matplotlib (3.9.2)
* shapely (2.0.6)
* pygame (2.0.1)
* scikit-learn (1.5.2)
* numba (0.60.0)

Install the requirements using:
```
pip install -r requirements.txt
```

#### Install
```
git clone https://github.com/RaphaelDssn/Multi-Robot-Path-Planning-for-Coverage-in-Cluttered-and-Known-Environment.git
```


## Usage

Run following in a terminal: 

```
python3 main.py
```

<table style="width:100%; text-align:center;">
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


## References


[1] Alice-St. Alice-st/darp. https://github.com/alice-st/DARP, n.d. Accessed: 21 December 2024.

[2] Rodriguesrenato. Rodriguesrenato/coverage-path-planning: A coverage path planning algorithm that combines multiple search algorithms to find a full coverage trajectory with the lowest cost. https://github.com/rodriguesrenato/coverage-path-planning, n.d. Accessed: 21 December 2024.


## Cite as

