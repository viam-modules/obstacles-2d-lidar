from .point_cloud import PointCloud
import math
import struct

TO_M = .001

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


def decode_pcd_bytes(pcd_bytes, min_range_mm=None) -> PointCloud:
    """
    returns PointCloud from pcd_bytes
    Args:
        pcd_bytes 

    Returns:
        PointCloud: _description_
    """    
  
    metadata = {}
    header_lines = pcd_bytes.split(b'\n')
    for line in header_lines:
        line = line.decode('utf-8', errors='ignore')  # Ignore non-UTF-8 characters
        if line.startswith('VERSION'):
            metadata['VERSION'] = float(line.split(' ')[1])
        elif line.startswith('FIELDS'):
            metadata['FIELDS'] = line.split(' ')[1:]
        elif line.startswith('SIZE'):
            metadata['SIZE'] = [int(size) for size in line.split(' ')[1:]]
        elif line.startswith('TYPE'):
            metadata['TYPE'] = line.split(' ')[1:]
        elif line.startswith('COUNT'):
            metadata['COUNT'] = [int(count) for count in line.split(' ')[1:]]
        elif line.startswith('WIDTH'):
            metadata['WIDTH'] = int(line.split(' ')[1])
        elif line.startswith('HEIGHT'):
            metadata['HEIGHT'] = int(line.split(' ')[1])
        elif line.startswith('VIEWPOINT'):
            metadata['VIEWPOINT'] = [int(count) for count in line.split(' ')[1:]]
        elif line.startswith('POINTS'):
            metadata['POINTS'] = int(line.split(' ')[1])
        elif line.startswith('DATA'):
            metadata['DATA'] = line.split(' ')[1]

    if metadata['DATA'] != 'binary':
        raise ValueError(f"DATA format is not supported. DATA should be binary but got: {metadata['DATA']}")
    # find the start of binary data, the following won't work if DATA ascii
    data_start = header_lines.index(b'DATA binary') + 1
    # extract binary data
    binary_data = b'\n'.join(header_lines[data_start:])
    num_floats = len(binary_data) // metadata['SIZE'][0]
    pcd_data = struct.unpack('f' * num_floats, binary_data)
    
    points = tuple_to_ordered_lists(pcd_data, metadata['FIELDS'])
    
    if min_range_mm is None:
        return PointCloud(points= points, metadata=metadata)

    points_filtered = []
    for point in points:
        if math.sqrt(point[0]**2+point[1]**2+point[2]**2) > min_range_mm * TO_M:
            points_filtered.append(point)

    metadata["POINTS"] = len(points_filtered)
    metadata["WIDTH"] = len(points_filtered)
    
    return PointCloud(points= points_filtered, metadata=metadata)
