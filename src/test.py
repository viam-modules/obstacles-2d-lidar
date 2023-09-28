import numpy as np
from sklearn.decomposition import PCA, FastICA
import matplotlib.pyplot as plt
X = np.array([[0, 0],[ 1, 2], [2, 4], [3, 6], [4, 8], [5, 10], [1, -2], [2, -4], [3,-6]])
pca = PCA(n_components=2)
pca.fit(X)
ica = FastICA(whiten="unit-variance")
ica.fit(X)
print(pca.explained_variance_ratio_)
print(ica.components_)
X_1 = pca.transform(X)

x_coords = X[:, 0]
y_coords = X[:, 1]

x1_coords = X_1[:, 0]
y1_coords = X_1[:, 1]
N = np.arange(5)
N2 = []

plt.arrow(0,0, 2*ica.components_[0][0], 2*ica.components_[0][1], width = .2)
plt.arrow(0,0, 2*ica.components_[1][0], 2*ica.components_[1][1], width = .2)
# for n in N:
#     np.dot(np.dpca.components_[0]
# Create a scatter plot
plt.scatter(x_coords, y_coords, label='Data Points', c='b')
plt.scatter(x1_coords, y1_coords, label='Transform Points', c='r')

# Add labels and a legend
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.axis('equal')
plt.grid()

# Show the plot
plt.show()