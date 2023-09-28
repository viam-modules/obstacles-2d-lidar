


import numpy as np
import matplotlib.pyplot as plt

def plot_point_cloud(X, Y):
    # Create a scatter plot of the points
    plt.scatter(X, Y, s=10, c='b', marker='o', label='Point Cloud')

    # Add labels and title
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Point Cloud Plot')

    # Add a legend
    plt.legend()

    # Show the plot
    plt.grid(True)
    plt.show()

def create_projection_from_list(points: np.ndarray):
    number_of_points = points.shape[0]
    X = []
    Y =[]
    for i in range(number_of_points):
        X.append(points[i][0])
        Y.append(points[i][1])
    return X, Y

def normalize_pcd(X,Y):
    maxi = max(abs(X.max()), abs(X.min()), abs(Y.max()), abs(Y.min()))
    print(f"maxi is {maxi}")
    print(f"X shape is {X.shape}")
    for i in range(X.shape[0]):
        X[i] = X[i]/maxi
        Y[i] = Y[i]/maxi
    return X, Y
        
    
    
def plot_inliers_and_outliers(X, Y, inlier_mask=None):
    X =X.flatten()
    outlier_mask = np.logical_not(inlier_mask)
    plt.scatter(
    X[inlier_mask], Y[inlier_mask], color="yellowgreen", marker=".", label="Inliers"
    )
    plt.scatter(
        X[outlier_mask], Y[outlier_mask], color="gold", marker=".", label="Outliers"
    )
    plt.legend()

    # Show the plot
    plt.grid(True)
    plt.show() 

