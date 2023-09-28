import numpy as np
import matplotlib.pyplot as plt
import struct

class PointCloud():
    '''
    self.points is a list of list of coordinates
    should be ordered like  'x', 'y','z'
    '''
    def __init__(self, points:list, metadata: dict=None) -> None:
        self.points =  points
        self.metadata  = metadata
        

class PlanarPointCloud():
    '''
    self.X array of shape (n_points, 1)
    self.Y array of shape (n_points, 1)
    '''
    def __init__(self, X: np.array,  Y: np.array) -> None:
        self.X = X
        self.Y = Y
            
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

        



def tuple_to_ordered_lists(data_tuple, fields=None):
    if len(data_tuple) % 3 != 0:
        raise ValueError("Tuple length must be a multiple of 3")

    if fields is None:
        # Default order: 'x', 'y', 'z'
        fields = ['x', 'y', 'z']
    elif sorted(fields) != ['x', 'y', 'z']:
        raise ValueError("Invalid field order. Use 'x', 'y', 'z' in any order.")

    result = []

    for i in range(0, len(data_tuple), 3):
        sublist = [data_tuple[i + fields.index('x')], data_tuple[i + fields.index('y')], data_tuple[i + fields.index('z')]]
        result.append(sublist)
    return result


def decode_pcd_bytes(pcd_bytes) -> PointCloud:
    """ Returns PointCloud object from pcd bytes
    

    Args:
        pcd_bytes 

    Returns:
        PointCloud: _description_
    """    
    # returns PointCloud from pcd_bytes
    metadata = {}
    header_lines = pcd_bytes.split(b'\n')
    for line in header_lines:
        line = line.decode('utf-8', errors='ignore')  # Ignore non-UTF-8 characters
        if line.startswith('FIELDS'):
            metadata['fields'] = line.split(' ')[1:]
        elif line.startswith('SIZE'):
            metadata['sizes'] = [int(size) for size in line.split(' ')[1:]]
        elif line.startswith('COUNT'):
            metadata['counts'] = [int(count) for count in line.split(' ')[1:]]
        elif line.startswith('DATA'):
            metadata['data'] = line.split(' ')[1]

    # Find the start of binary data, the following won't work if DATA ascii
    data_start = header_lines.index(b'DATA binary') + 1

    # Extract binary data
    binary_data = b'\n'.join(header_lines[data_start:])
    num_floats = len(binary_data) // 4
    pcd_data = struct.unpack('f' * num_floats, binary_data)
    
    points = tuple_to_ordered_lists(pcd_data, metadata['fields'])
    
    return PointCloud(points= points, metadata=metadata)


def get_planar_from_3D(pc:PointCloud) -> PlanarPointCloud:
    """_summary_
    
    Projects a 3D pointcloud in 2D. Assume that 3D point cloud is ordered like 'x', 'y', 'z' 
    and returns the  XY projection. 

    Args:
        pc (PointCloud): _description_

    Returns:
        PlanarPointCloud: _description_
    """    
    x, y = [], []
    for point in pc.points:
        x.append(point[0])
        y.append(point[1])
    return PlanarPointCloud(X = np.array(x).reshape(-1, 1), Y = np.array(y ).reshape(-1, 1))

