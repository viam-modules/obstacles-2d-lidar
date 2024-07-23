from typing import Tuple

import numpy as np
from sklearn import linear_model


class RansacRegressor:
    """
    A class that encapsulates the RANSAC regression model  from sklearn.

    Args:
        min_samples (int): Minimum number of samples required to fit the model.
        residual_threshold (float): Maximum residual for a data point
        to be classified as an inlier.
        stop_probability (float): Probability that at least
        one outlier-free subset of the data is found.
    """

    def __init__(self, min_samples, residual_threshold, stop_probability) -> None:
        self.regressor = linear_model.RANSACRegressor(
            min_samples=min_samples,
            residual_threshold=residual_threshold,
            stop_probability=stop_probability,
        )

    def fit(self, X, Y) -> None:  # pylint: disable=invalid-name
        """
        Fits the RANSAC regressor model using the given data.

        Args:
            X (array-like): The input data.
            Y (array-like): The target values.
        """
        self.regressor.fit(X, Y)

    def get_one_wall(self) -> Tuple[linear_model.RANSACRegressor, list, list]:
        """
        Retrieves the fitted RANSAC regressor model and the inlier and outlier masks.

        Returns:
            Tuple[linear_model.RANSACRegressor, list, list]:
                - The fitted RANSAC regressor model.
                - The mask indicating inliers.
                - The mask indicating outliers.
        """
        inlier_mask = self.regressor.inlier_mask_
        outlier_mask = np.logical_not(inlier_mask)
        return self.regressor, inlier_mask, outlier_mask
