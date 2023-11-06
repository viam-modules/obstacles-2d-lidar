from .point_cloud import PlanarPointCloud, PointCloud

class Encoder():
    def __init__(self, metadata:dict) -> None:
        self.metadata = metadata
        self.header = metadata_to_pcd_header(metadata)
        
        
    def get_new_header(self, pc:PointCloud):
        '''
        retuns a list of lines that must be concatenated with the actual data
        '''
        metadata = self.metadata
        metadata['WIDTH'] = pc.points.shape[0]
        metadata['HEIGHT'] = 1
        metadata['POINTS'] = pc.points.shape[0]
        return metadata_to_pcd_header(metadata)
    
    def encode(self, pc:PointCloud):
        lines  = self.get_new_header(pc)
        data_lines = encode_data_as_ascii(pc.points.tolist())
        lines += data_lines
        pcd_content = '\n'.join(lines)
        return pcd_content.encode('ascii', 'replace')
        
        

def metadata_to_pcd_header(metadata, data_type="ascii"):
    pcd_lines = []

    pcd_lines.append(f"VERSION {metadata['VERSION']}")
    pcd_lines.append(f"FIELDS {' '.join(metadata['FIELDS'])}")   
    pcd_lines.append(f"SIZE {' '.join(map(str, metadata['SIZE']))}")
    pcd_lines.append(f"TYPE {' '.join(metadata['TYPE'])}")
    pcd_lines.append(f"COUNT {' '.join(map(str, metadata['COUNT']))}")
    pcd_lines.append(f"WIDTH {metadata['WIDTH']}")
    pcd_lines.append(f"HEIGHT {metadata['HEIGHT']}")
    pcd_lines.append(f"VIEWPOINT {' '.join(map(str, metadata.get('VIEWPOINT', [0, 0, 0, 1, 0, 0, 0])))}")
    pcd_lines.append(f"POINTS {metadata['POINTS']}")
    
    if data_type == "ascii":
        pcd_lines.append(f"DATA ascii")
    else:
        raise ValueError(f"DATA {data_type} is not yet supported")    
    return pcd_lines


def encode_data_as_ascii(points:list):
    #TODO: change this because nothing is encoded here
    pcd_data_lines = []
    for point in points:
        pcd_data_lines.append(f"{point[0]} {point[1]} {point[2]}")
    return pcd_data_lines
        
        
def encode_pointcloud_to_pcd_bytes(metadata:dict, points:list):
    pcd_lines= metadata_to_pcd_header(metadata)
    pcd_data_lines = encode_data_as_ascii(points)
   #TODO: should concatenate bytes here instead of encoding the whole thing together.
   #header should always be ascii
    pcd_lines += pcd_data_lines
    pcd_content = '\n'.join(pcd_lines)
    return pcd_content.encode('ascii', 'replace')