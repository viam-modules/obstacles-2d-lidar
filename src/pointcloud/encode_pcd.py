

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
    pcd_data_lines = []
    for point in points:
        pcd_data_lines.append(f"{point[0]} {point[1]} {point[2]}")
    return pcd_data_lines
        
        
def encode_pointcloud_to_pcd_bytes(metadata:dict, points:list):
    pcd_lines= metadata_to_pcd_header(metadata)
    pcd_data_lines = encode_data_as_ascii(points)
    pcd_lines += pcd_data_lines
    pcd_content = '\n'.join(pcd_lines)
    return pcd_content.encode('ascii', 'replace')