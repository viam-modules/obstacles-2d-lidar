import numpy as np
import math 
import matplotlib.pyplot as plt


def get_rotating_caliper_bbox_list(hull_points_2d):
    """

    hull_points_2d: array of hull points. each element should have [x,y] format
    """

    # Compute edges (x2-x1,y2-y1)
    edges = np.zeros( (len(hull_points_2d)-1,2) ) # empty 2 column array
    for i in range( len(edges) ):
        edge_x = hull_points_2d[i+1,0] - hull_points_2d[i,0]
        edge_y = hull_points_2d[i+1,1] - hull_points_2d[i,1]
        edges[i] = [edge_x,edge_y]

    # Calculate edge angles   atan2(y/x)
    edge_angles = np.zeros( (len(edges)) ) # empty 1 column array
    for i in range( len(edge_angles) ):
        edge_angles[i] = np.arctan2( edges[i,1], edges[i,0] )

    # Check for angles in 1st quadrant
    for i in range( len(edge_angles) ):
        edge_angles[i] = np.abs( edge_angles[i] % (np.pi/2) ) # want strictly positive answers
    #print "Edge angles in 1st Quadrant: \n", edge_angles

    # Remove duplicate angles
    edge_angles = np.unique(edge_angles)
    #print "Unique edge angles: \n", edge_angles

    bbox_list=[]
    for i in range( len(edge_angles) ):

        # Create rotation matrix to shift points to baseline
        # R = [ cos(theta)      , cos(theta-PI/2)
        #       cos(theta+PI/2) , cos(theta)     ]
        R = np.array([ [ np.cos(edge_angles[i]), np.cos(edge_angles[i]-(np.pi/2)) ], [ np.cos(edge_angles[i]+(np.pi/2)), np.cos(edge_angles[i]) ] ])

        # Apply this rotation to convex hull points
        rot_points = np.dot(R, np.transpose(hull_points_2d) ) # 2x2 * 2xn

        # Find min/max x,y points
        min_x = np.nanmin(rot_points[0], axis=0)
        max_x = np.nanmax(rot_points[0], axis=0)
        min_y = np.nanmin(rot_points[1], axis=0)
        max_y = np.nanmax(rot_points[1], axis=0)

        # Calculate height/width/area of this bounding rectangle
        width = max_x - min_x
        height = max_y - min_y
        area = width*height


        # Calculate center point and restore to original coordinate system
        center_x = (min_x + max_x)/2
        center_y = (min_y + max_y)/2
        center_point = np.dot( [ center_x, center_y ], R )

        # Calculate corner points and restore to original coordinate system
        corner_points = np.zeros( (4,2) ) # empty 2 column array
        corner_points[0] = np.dot( [ max_x, min_y ], R )
        corner_points[1] = np.dot( [ min_x, min_y ], R )
        corner_points[2] = np.dot( [ min_x, max_y ], R )
        corner_points[3] = np.dot( [ max_x, max_y ], R )


        bbox_info = [edge_angles[i], area, width, height, min_x, max_x, min_y, max_y, corner_points, center_point]
        bbox_list.append(bbox_info)


    return bbox_list



def get_minimum_bounding_box(hull_points_2d):
    
    # Compute edges (x2-x1,y2-y1)
    edges = np.zeros( (len(hull_points_2d)-1,2) ) # empty 2 column array
    for i in range( len(edges) ):
        edge_x = hull_points_2d[i+1,0] - hull_points_2d[i,0]
        edge_y = hull_points_2d[i+1,1] - hull_points_2d[i,1]
        edges[i] = [edge_x,edge_y]

    # Calculate edge angles   atan2(y/x)
    edge_angles = np.zeros( (len(edges)) ) # empty 1 column array
    for i in range( len(edge_angles) ):
        edge_angles[i] = np.arctan2( edges[i,1], edges[i,0] )

    # Check for angles in 1st quadrant
    for i in range( len(edge_angles) ):
        edge_angles[i] = np.abs( edge_angles[i] % (np.pi/2) ) # want strictly positive answers
    #print "Edge angles in 1st Quadrant: \n", edge_angles

    # Remove duplicate angles
    edge_angles = np.unique(edge_angles)
    #print "Unique edge angles: \n", edge_angles
    
    min_area = math.inf 
    for i in range( len(edge_angles) ):

        # Create rotation matrix to shift points to baseline
        # R = [ cos(theta)      , cos(theta-PI/2)
        #       cos(theta+PI/2) , cos(theta)     ]
        R = np.array([ [ np.cos(edge_angles[i]), np.cos(edge_angles[i]-(np.pi/2)) ], [ np.cos(edge_angles[i]+(np.pi/2)), np.cos(edge_angles[i]) ] ])

        # Apply this rotation to convex hull points
        rot_points = np.dot(R, np.transpose(hull_points_2d) ) # 2x2 * 2xn

        # Find min/max x,y points
        min_x = np.nanmin(rot_points[0], axis=0)
        max_x = np.nanmax(rot_points[0], axis=0)
        min_y = np.nanmin(rot_points[1], axis=0)
        max_y = np.nanmax(rot_points[1], axis=0)

        # Calculate height/width/area of this bounding rectangle
        width = max_x - min_x
        height = max_y - min_y
        area = width*height
        if area < min_area:
            min_area = area

            # Calculate center point and restore to original coordinate system
            center_x = (min_x + max_x)/2
            center_y = (min_y + max_y)/2
            center_point = np.dot( [ center_x, center_y ], R )

            # Calculate corner points and restore to original coordinate system
            corner_points = []
            corner_points.append(np.dot( [ max_x, min_y ], R ))
            corner_points.append(np.dot( [ min_x, min_y ], R ))
            corner_points.append(np.dot( [ min_x, max_y ], R ))
            corner_points.append(np.dot( [ max_x, max_y ], R ))
            
            bb = BoundingBox(corner_points, area = area)
    
    print(f"Bounding box are is {bb.area}")
    return bb
    
    


class BoundingBox:
    def __init__(self, edges: list, min_x=None, max_x = None, min_y = None, max_y = None, area=None):
        self.edges = edges
        self.area = area
        
    
    def get_area(self):
        if self.area is None:
            self.area = (self.max_x-self.min_x)*(self.max_y-self.min_y)
        return self.area
    
    
    def plot(self):
        x = [edge[0] for edge in self.edges]
        y = [edge[1] for edge in self.edges]

        # Close the bounding box by adding the first point at the end
        x.append(x[0])
        y.append(y[0])

        # Create a figure and axis

        # Plot the bounding box
        plt.plot(x, y, marker='x')

        # Add labels and title

        # Show the plot
        plt.grid(True)
        # plt.gca().set_aspect('equal', adjustable='box')
        # plt.show()
# plt.show()







    