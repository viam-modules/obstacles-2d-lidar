import numpy as np

from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt





def get_dbscan(X, Y):
    combined_array = np.concatenate((X, Y), axis=1)
    db = DBSCAN(eps=0.05, min_samples=2).fit(combined_array)
    labels = db.labels_

    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)
    return db, n_clusters_, n_noise_

def plot_clusters(db: DBSCAN, X:np.ndarray):
    labels = db.labels_
    unique_labels = set(labels)
    core_samples_mask = np.zeros_like(labels, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True

    colors = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]
    for k, col in zip(unique_labels, colors):
        if k == -1:
            # Black used for noise.
            col = [0, 0, 0, 1]

        class_member_mask = labels == k

        # xy = X[class_member_mask & core_samples_mask]
        xy = X[class_member_mask]
        plt.plot(
            xy[:, 0],
            xy[:, 1],
            "o",
            markerfacecolor=tuple(col),
            markeredgecolor="k",
            markersize=5,
        )

        xy = X[class_member_mask & ~core_samples_mask]
        plt.plot(
            xy[:, 0],
            xy[:, 1],
            "o",
            markerfacecolor=tuple(col),
            markeredgecolor="k",
            markersize=1,
        )

    # plt.title(f"Estimated number of clusters: {n_clusters_}")
    plt.axis("equal")
    plt.show()
    
class Point:
	def __init__(self, x = None, y = None):
		self.x = x
		self.y = y

class Cluster:
    def __init__(self, points:list[Point] = None, label:int=None):
        self.points = points
        self.label = label
    

  
def build_clusters(db: DBSCAN, X) -> list[Cluster]:
    """ 
        Function that returns a list of labelled clusters from a DBSCAN 
        object and planar pointcloud
    Args:
        db (DBSCAN): dbscan already fitted
        X (np.ndarray): Pointcloud. If there are n points in the planar pointcloud X 
        should be of size (n,2)
    """
    res = []
    labels = db.labels_
    # print(labels)
    unique_labels = set(labels)
    core_samples_mask = np.zeros_like(labels, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    
    
    for l in unique_labels:
        class_member_mask = labels == l
        points = X[class_member_mask & core_samples_mask]
        # print(f"Points type is {type(points)}")
        # print(f"points shape is {points.shape}")
        res.append(Cluster(points, l))
        
    return res
        
        