import numpy as np
from sklearn import linear_model

class RansacRegressor():
    def __init__(self,
                 min_samples, 
                 residual_threshold, 
                 stop_probability) -> None:
        self.regressor =  linear_model.RANSACRegressor(min_samples=min_samples,
                                                    residual_threshold=residual_threshold,
                                                    stop_probability= stop_probability)
        
    def fit(self, X, Y) -> None:
        self.regressor.fit(X,Y)
    
    def get_one_wall(self) -> (linear_model.RANSACRegressor, list, list):
        """
        Model should be fitted before.
        """        
        inlier_mask = self.regressor.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)
        return self.regressor, inlier_mask, outlier_mask
        