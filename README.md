# Obstacles detection from planar lidar
[Viam module](https://docs.viam.com/extend/modular-resources/) for 2D obstacles detection as a vision service. 

<p align="center">
 <img src="https://github.com/Rob1in/obstacles-2d-lidar/blob/module/img/results.png" width=100%, height=100%>
 </p>



## Getting started

TODO: Add link to Rplidar module..
This module implements the method `GetObjectPointClouds()` of the [vision service API](https://docs.viam.com/services/vision/#api).

### Installation with `pip install`
 /!\ Not ready yet /!\

```
pip install -r requirements.txt
```

## Config
### Example config 
```json
{
  "services": [
    {
      "namespace": "rdk",
      "model": "viam:vision:obstacles-2d-detector",
      "attributes": {
        "min_points_cluster": 4,
        "min_bbox_area": 0.2,
        "camera_name": "rplidar"
      },
      "name": "detector-module",
      "type": "vision"
    }
  ],
  "components": [
    {
      "namespace": "rdk",
      "type": "camera",
      "model": "viam:lidar:rplidar",
      "attributes": {
        "device_path": "/dev/tty.usbserial-0001"
      },
      "name": "rplidar",
      "depends_on": []
    }
  ],
  "modules": [
    {
      "name": "mydetector",
      "type": "local",
      "executable_path": "/path/to/your/run.sh"
    },
    {
      "executable_path": "/opt/homebrew/bin/rplidar-module",
      "name": "rplidar_module"
    }
  ]
}
```

### Attributes description

The following attributes are available to configure your 2d obstacles detection module:

| Name | Type | Inclusion | Default | Description |
| ---- | ---- | --------- | --------| ------------ |
| `camera_name` | string | **Required** | | Camera name (planar lidar) to be used as input for detecting the obstacles. |
| `dbscan_eps` | float | Optional | `0.05` | The maximum distance between two samples for one to be considered as in the neighborhood of the other. This is not a maximum bound on the distances of points within a cluster. This is the most important DBSCAN parameter to choose appropriately for your data set and distance function.|
|`dbscan_min_samples`| int | Optional | `2` |The number of samples (or total weight) in a neighborhood for a point to be considered as a core point. This includes the point itself. If min_samples is set to a higher value, DBSCAN will find denser clusters, whereas if it is set to a lower value, the found clusters will be more sparse. |
|`min_points_cluster`| int | Optional | `2` | Minimum number of points to define a cluster (obstacle)  |
|`min_bbox_area`| float | Optional | `0.15` | If the area of the cluster found by dbscan in the normalized axis coordinate is bigger than `min_bbox_area`, try to refine the cluster with 2d RANSAC. |
|`ransac_min_samples`| int | Optional | `2` | Minimum number of samples chosen randomly from original data. |
|`ransac_residual_threshold`| float | Optional | `0.2` | Maximum residual for a data sample to be classified as an inlier. By default the threshold is chosen as the MAD (median absolute deviation) of the target values y. Points whose residuals are strictly equal to the threshold are considered as inliers. |
|`ransac_stop_probability`| float | Optional | `0.99` | RANSAC iteration stops if at least one outlier-free set of the training data is sampled in RANSAC. This requires to generate at least N samples (iterations): $N = \frac{\log(1-p)}{\log(1-w^n)}$, where $n$ is the number of points, $w$ the ratio of inliers to total points, and $p$ is `ransac_stop_probability`. |


## Deeper dive
 /!\ In progress... /!\

The module works as follow:
  1. Get pcd
  2. DBSCAN
  3. Refine if big cluster
  4. Find convex hull with graham scan
  5. Find minimum fitting bounding box with rotating calipers
  6. Encode clusters as pcd bytes

## References

 1. [Scikit Learn DBSCAN documentation](https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html)
 2. [Scikit Learn Linear RANSAC documentation](https://scikit-learn.org/stable/modules/generated/sklearn.linear_model.RANSACRegressor.html)
 3. [Graham Scan Algorithm](https://en.wikipedia.org/wiki/Graham_scan)
 4. [Rotating Calipers](https://en.wikipedia.org/wiki/Rotating_calipers)
 5. [**P**oint **C**loud **D**ata File Format](https://pointclouds.org/documentation/tutorials/pcd_file_format.html)

## Troubleshooting



