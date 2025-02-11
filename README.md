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


| Option          | Description                                            | Default Value |
|-----------------|--------------------------------------------------------|---------------|
| `-h, --help`    | Show help message and exit                             | N/A           |
| `-cell`         | Cell dimension (side length) [in meters]               | `0.5`         |
| `-robot_radius` | Robot radius [in meters]                               | `0.1`         |
| `-MaxIter`      | Covering factor: Maximum number of iterations for DARP | `2000`        |


## References


[1] Alice-St. Alice-st/darp. https://github.com/alice-st/DARP, n.d. Accessed: 21 December 2024.

[2] Rodriguesrenato. Rodriguesrenato/coverage-path-planning: A coverage path planning algorithm that combines multiple search algorithms to find a full coverage trajectory with the lowest cost. https://github.com/rodriguesrenato/coverage-path-planning, n.d. Accessed: 21 December 2024.


## Cite as

