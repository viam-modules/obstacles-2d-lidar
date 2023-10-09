import numpy as np
from matplotlib import pyplot as plt

from sklearn import linear_model

# # Robustly fit linear model with RANSAC algorithm
# ransac = linear_model.RANSACRegressor()
# ransac.fit(X, y)
# inlier_mask = ransac.inlier_mask_
# outlier_mask = np.logical_not(inlier_mask)


# Robustly fit linear model with RANSAC algorithm
def get_ransac(X, Y) -> linear_model.RANSACRegressor:
    ransac = linear_model.RANSACRegressor(residual_threshold=.2)
    ransac.fit(X, Y)
    return ransac

def get_one_wall(X, Y)-> (linear_model.RANSACRegressor, list, list):
    ransac_regressor = get_ransac(X, Y)
    inlier_mask = ransac_regressor.inlier_mask_
    outlier_mask = np.logical_not(inlier_mask)
    return ransac_regressor, inlier_mask, outlier_mask
