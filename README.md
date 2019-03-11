# IF-Matching
MATLAB implementation of [IF-Matching ](https://ieeexplore.ieee.org/abstract/document/7590096) Algorithm for map matching problem. (one proposed optimization based on [ST-Matching ](https://github.com/sam1016yu/ST-Matching) )
## Paper Abstract

> With the advance of various location-acquisition technologies, a myriad of GPS trajectories can be collected every day. However, the raw coordinate data captured by sensors often cannot reflect real positions due to many physical constraints and some rules of law. How to accurately match GPS trajectories to roads on a digital map is an important issue. The problem of map-matching is fundamental for many applications. Unfortunately, many existing methods still cannot meet stringent performance requirements in engineering. In particular, low/unstable sampling rate and noisy/lost data are usually big challenges. Information fusion of different data sources is becoming increasingly promising nowadays. As in practice, some other measurements such as speed and moving direction are collected together with the spatial locations acquired, we can make use of not only location coordinates but all data collected. In this paper, we propose a novel model using the related meta-information to describe a moving object, and present an algorithm called IF-Matching for map-matching. It can handle many ambiguous cases which cannot be correctly matched by existing methods. We run our algorithm with taxi trajectory data on a city-wide road network. Compared with two state-of-the-art algorithms of ST-Matching and the winner of GIS Cup 2012, our approach achieves more accurate results.

## Algorithm pseudocode
![Algorithm 1. Candidate Projection](
        candidateProjection.PNG
      )
![Algorithm 2. History Speed Mining](
        historySpeedMining.PNG
      )
![Algorithm 3. Surrounding Speed Estimation](
surroundingSpeedEstimation.PNG
)
![Algorithm 4. IF-Matching](
        IFMatching.PNG
      )
      
## Implementation Steps
### Entry point:
IF-matching.m
### Data preparation:
There are two sets of input files:
- road network file (containing information of edges and nodes, usually from Open Street Map)
- gps trajectory files

Format of **road network** file:

EdgeID | Node1ID | Node2ID | Longitude of Node1 | Latitude of Node1 |  Longitude of Node2 | Latitude of Node2 | Road Type
---|--- |--- |--- |--- |--- | -- | --

Format of **gps trajactory** files:

GPS Device ID | Timestamp | Longitude | Latitude | Speed (km/h) | Direction
---|--- |--- |--- |--- |--- 

### Step1: split road into smaller cells
define length of each cell(in km),then call ***splitRoadtoCell***

code sample:
```
roadnetworkfilename = 'RoadNetwork_Beijing.txt';
cell_size = 0.1;
[road_network,speed_limits,road_cells,road_ids,grid_size] = splitRoad2Cell(roadnetworkfilename, cell_size);
```
output of this code include 
- a matrix of raw road network ***road_network*** (*m* x *7*,same as input road network file except last column)
- a vector of speed limist of each roads ***speed_limits*** translated from road type (last column of input road network)
- information of each cell ***road_cells*** (*m* x *5*)

cellID | startLon | endLon | startLat | endLat
---|---|---|---|---
- size of grid ***grid_size*** (*m* x *n* cells from entire road network)

### Step2: split raw gps trajactory into smaller segments
function: **splitGPS2line**

Splitting criteria: 
- assume a standard sampling interval $\Deltat_n$  and a standard length for each trajectory segment ***l***
- the length of a trajectory should be less or equal than ***l***
- $sum \Delta t <= 5\Deltat_n*l$
- cut as few trajectory as possible

code sample:

```
gpsfilename = 'GPS_Beijing.txt';
raw_gps_points = splitGPS2line(gpsfilename, 6, 5);
```

output result will be added as a tag additional to the original gps trajectory file, so that the format of ***raw_gps_points*** (*m* x *7*) will be

GPS Device ID | Timestamp | Longitude | Latitude | Speed (km/h) | Direction | Tag
---|--- |--- |--- |--- |--- | ---

### Step3: get candidate points of all trajectory

With the cutted road network, we first find candidate edges around raw sample points, then project sample points to that edge. (for speed consideration, only the top 5 closest candidate edges are considered) 

- Script for this step can be seen in function ***getCandidatePoints***.
- Line projection is vector based and can be seen in the sub-function ***project2Line***

### Step4: mine history speed for all trajactory
Instead of directly use speed limits of each road as in ST-Matching, historical speed for all candidate road segments are mined thus provide a more accurate reference for temporal analysis. Each day was cutted into 12 segments(2 hours per segnemt).

### Step5: mine surrounding speed for trajactory
In additional to history speed,a more accurate speed is used to take advantage of road condition. A group of candidate points within a time threshold and have a similar moving direction are grouped together, the mean of which is treated as the surrounding speed.

### Step6: cut the road network for specific trajectory
The entire road_network obtained in step 1 is considered too big for an individual trajectory, hence we implement a cutting on the original big road network. 
- script for this step can be seen in function ***cutGridforTrajactory***
- The cutted network contains all the **candidate edges** of trajectory points.
- a graph ***G*** is constructed for path search in the next step
- a table ***node_table*** recording relationship between original **NodeID** and actual node ID used in graph ***G*** is generated

### Step7: find matched sequence
see ***IF-Matching*** above for general procedure of this step.
both **spatial(transmission * observation)** and **temporal** probability are calculated between candidate points.
#### post-processing
In our experiment, we find some issue particularly at crossing roads. This is due to the imbalance of two probabilities. For example, when transmission probability is very close, observation probability become the dominant factor. We did a potential fix for simple crossing error.
### Step6: validation
see ***validation.m*** and picture in the **result** folder for detail. Sample points are plotted in blue points, while the matched points are plotted in red cross. The path between matched points are also plotted. 