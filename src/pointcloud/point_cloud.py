import numpy as np
import matplotlib.pyplot as plt
import struct
# from .encode_pcd import *


class PlanarPointCloud():
    '''
    self.X array of shape (n_points, 1)
    self.Y array of shape (n_points, 1)
    '''
    def __init__(self, points:list=[], normalize:bool = False) -> None:
        # self.points = np.array(points)
        """_summary_

        Args:
            points (list, optional): _description_. Defaults to [].
            normalized (bool, optional): Defines if the input point passed 
            to the constructor are normalized. Defaults to False.
            scale (float, optional): None if not normalized.
        """        
        
        self.points = np.array(points)
        
        if normalize:
            self.normalize_point_cloud()
            
        self.scale = None #indicates if the pointcloud has been normalized yet
    
    @property
    def X(self):
        ''' 
        self.X array of shape (n_points, 1)
        '''
        X = self.points[:,0]
        return X.reshape(-1, 1)
    
    @property
    def Y(self):
        ''' 
        self.Y array of shape (n_points, 1)
        '''
        Y = self.points[:,1]
        return Y.reshape(-1, 1)
       
    @property 
    def X_norm(self):
        if self.scale is None:
            raise AttributeError("Planar poincloud must be normalized first")
        return self.X/self.scale
    
    @property 
    def Y_norm(self):
        if self.scale is None:
            raise AttributeError("Planar poincloud must be normalized first")
        return self.Y/self.scale 
    
    # @property
    # def points_norm(self):
    #     if self.scale is None:
    #         raise AttributeError("Planar poincloud must be normalized first")
    #     return self.points_norm

    def normalize_point_cloud(self, centered = False):
        self.scale = max(abs(self.X.max()), abs(self.X.min()), abs(self.Y.max()), abs(self.Y.min()))
        self.points_norm = self.points/self.scale
    
    def get_ppc_from_mask(self, mask, keep_scale=True):
        res = PlanarPointCloud(points=self.points[mask])
        if keep_scale:
            if self.scale is None:
                self.normalize_point_cloud()
            else:
                res.scale = self.scale
                res.points_norm = self.points_norm[mask]
        return res
    
    def plot(self, normalized = False):
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
        self.points =  np.array(points)
        self.metadata  = metadata
        
    def get_planar_from_3D(self, axis_normal:str='z') -> PlanarPointCloud:
        #TODO: Add a logic to detect the projection plan 
        """_summary_
        
        Projects a 3D pointcloud in 2D. Assume that 3D point cloud is ordered like 'x', 'y', 'z' 
        and returns the  XY projection. 

        Args:
            pc (PointCloud): input

        Returns:
            PlanarPointCloud: ouput
        """
        
        if axis_normal == 'z':
            return PlanarPointCloud(self.points[:, :2])
        if axis_normal == 'y':
            return PlanarPointCloud(self.points[:, 1:2])
        if axis_normal == 'x':
            return PlanarPointCloud(self.points[:, 0:])
        
        else:
            raise ValueError("Invalid normal axis. Please provide 'x', 'y' or 'z' or 2 as argument for normal axis.")
        

    
    
    # def get_pcd_bytes(self, mask = None):
    #     #TODO: create a new metadata dict and change width, height, size
    #     return encode_pointcloud_to_pcd_bytes(self.metadata, self.points[mask])
    

def get_pc_from_pcc(ppc: PlanarPointCloud, z:float=0, metadata:dict=None, scale_to:float=1):
    n_points = ppc.points.shape[0]
    z_values = np.ones((n_points, 1))*z
    points = np.concatenate((ppc.points*scale_to, z_values), axis=1)
    return PointCloud(points=points.tolist(), metadata=metadata)