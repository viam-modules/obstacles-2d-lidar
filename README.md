# Obstacles detection from planar lidar
[Viam module](https://docs.viam.com/extend/modular-resources/) for 2D obstacles detection as a vision service. 

## How to Configure the Module

[add link to docs]

## Getting started

All you need is a calibrated camera.

This module implements the method `GetObjectPointClouds()` of the [vision service API](https://docs.viam.com/services/vision/#api).
### Installation with `pip install`
 /!\ Not ready yet /!\

```
pip install -r requirements.txt
```

## Config
### Example config 
 /!\ Not ready yet /!\

### Attributes description
The following attributes are available to configure your 2d obstacles detection module:

| Name | Type | Inclusion | Default | Description |
| ---- | ---- | --------- | --------| ------------ |
| `camera_name` | string | **Required** | | Camera name (planar lidar) to be used as input for detecting the obstacles. |
| `dbscan_eps` | float | Optional | `0.05` | The maximum distance between two samples for one to be considered as in the neighborhood of the other. This is not a maximum bound on the distances of points within a cluster. This is the most important DBSCAN parameter to choose appropriately for your data set and distance function.|
|`dbscan_min_samples`| int | Optional | `2` |The number of samples (or total weight) in a neighborhood for a point to be considered as a core point. This includes the point itself. If min_samples is set to a higher value, DBSCAN will find denser clusters, whereas if it is set to a lower value, the found clusters will be more sparse. |
|`min_points_cluster`| int | Optional | `2` | Minimum number of points to define a cluster (obstacle)  |
|`min_bbox_area`| float | Optional | `0.15` | If the area of the cluster found by dbscan in the normalized axis coordinate is bigger than `min_bbox_area`, try to refine the cluster with 2d RANSAC. |

See the [Scikit Learn DBSCAN documentation](https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html) for more details.

## Deeper dive
 /!\ Not ready yet /!\
The module works as follow:
  1. Get pcd
  2. DBSCAN
  3. Refine if big cluster
  4. Find convex hull with graham scan
  5. Find minimum fitting bounding box with rotating calipers
  6. Encode clusters as pcd bytes


## References

## Troubleshooting



