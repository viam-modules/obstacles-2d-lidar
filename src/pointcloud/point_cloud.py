import numpy as np
import matplotlib.pyplot as plt
import struct
from .encode_pcd import *


class PlanarPointCloud():
    '''
    self.X array of shape (n_points, 1)
    self.Y array of shape (n_points, 1)
    '''
    def __init__(self, X: np.array,  Y: np.array, points=[]) -> None:
        self.X = X
        self.Y = Y
        self.points = points
            
    def normalize_point_cloud(self, centered = False):
        self.scale = max(abs(self.X.max()), abs(self.X.min()), abs(self.Y.max()), abs(self.Y.min()))
        self.X_norm, self.Y_norm = self.X/self.scale, self.Y/self.scale
        
        
            
    def plot_point_cloud(self, normalized = False):
        if normalized:
            plt.scatter(self.X_norm, self.Y_norm, s=10, c='b', marker='o', label='Normalized Point Cloud')
            
        else:
            plt.scatter(self.X, self.Y, s=10, c='b', marker='o', label='Point Cloud')

        # Add labels and title
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.title('Point Cloud Plot')

        # Add a legend
        plt.legend()

        # Show the plot
        plt.grid(True)
        plt.show()


class PointCloud():
    '''
    self.points is a list of list of coordinates
    should be ordered like  'x', 'y','z'
    '''
    def __init__(self, points:list, metadata: dict=None) -> None:
        self.points =  points
        self.metadata  = metadata
        
    def get_planar_from_3D(self) -> PlanarPointCloud:
        #TODO: Add a logic to detect the projection plan 
        """_summary_
        
        Projects a 3D pointcloud in 2D. Assume that 3D point cloud is ordered like 'x', 'y', 'z' 
        and returns the  XY projection. 

        Args:
            pc (PointCloud): _description_

        Returns:
            PlanarPointCloud: _description_
        """    
        x, y = [], []
        for point in self.points:
            x.append(point[0])
            y.append(point[1])
        return PlanarPointCloud(X = np.array(x).reshape(-1, 1), Y = np.array(y ).reshape(-1, 1), points=[point[:2] for point in self.points])
        

    
    
    def get_pcd_bytes(self, mask = None):
        #TODO: This is BS
        #TODO: create a new metadata dict and change width, height, size
        return encode_pointcloud_to_pcd_bytes(self.metadata, self.points[mask])