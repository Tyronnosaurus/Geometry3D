"""Matplotlib Renderer Module"""
from ..geometry.point import Point
from ..geometry.segment import Segment
from ..geometry.polygon import ConvexPolygon
from ..geometry.polyhedron import ConvexPolyhedron
from .arrow import Arrow
from ..utils.logger import get_main_logger
import numpy as np


class MatplotlibRenderer():
    """ Renderer module to visualize geometries"""

    def __init__(self):
        """
        **Input:**
        - No Input

        Initialize matplotlib
        """
        self.point_set = set()
        self.segment_set = set()
        self.arrow_set = set()


    def show(self):
        """
        Draw the image
        """
        from matplotlib import pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        fig = plt.figure()
        ax = Axes3D(fig)

        #Set X,Y,Z axis labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        get_main_logger().info('Showing geometries with %d points, %d segments, %d arrows using matplotlib' %(len(self.point_set),len(self.segment_set),len(self.arrow_set)))
        for point_tuple in self.point_set:
            point = point_tuple[0]
            color = point_tuple[1]
            size = point_tuple[2]
            ax.scatter(point.x,point.y,point.z,c=color,s=size)
        
        for segment_tuple in self.segment_set:
            segment = segment_tuple[0]
            color = segment_tuple[1]
            size = segment_tuple[2]
            x = [segment.start_point.x,segment.end_point.x]
            y = [segment.start_point.y,segment.end_point.y]
            z = [segment.start_point.z,segment.end_point.z]
            ax.plot(x,y,z,color=color,linewidth=size)

        for arrow_tuple in self.arrow_set:
            x,y,z,u,v,w,length = arrow_tuple[0].get_tuple()
            color = arrow_tuple[1]
            size = arrow_tuple[1]
            ax.quiver(x,y,z,u,v,w,color = color,length = length)

        self.ForceEqualAxisScaling(ax)
        plt.show()
        

    def add(self,obj,normal_length = 0, add_vertices=False):
        """
        **Input:**
        
        - obj: a tuple (object,color,size)
        
        - normal_length: the length of normal arrows for ConvexPolyhedron.
        For other objects, normal_length should be zero.
        If you don't want to show the normal arrows for a ConvexPolyhedron, you can set normal_length to 0.

        object can be Point, Segment, ConvexPolygon or ConvexPolyhedron

        - add_vertices: Draw a point on each vertex of a polygon
        """
        if isinstance(obj[0],Point):
            self.point_set.add(obj)
        elif isinstance(obj[0],Segment):
            self.segment_set.add(obj)
        elif isinstance(obj[0],Arrow):
            self.arrow_set.add(obj)
        elif isinstance(obj[0],ConvexPolygon):
            if (add_vertices):
                for point in obj[0].points:
                    self.add((point,obj[1],obj[2]))
            for segment in obj[0].segments():
                self.add((segment,obj[1],obj[2]))
            if normal_length > 0:
                cpg = obj[0]
                plane = cpg.plane
                normal = plane.n.normalized()
                array = Arrow(cpg.center_point.x,cpg.center_point.y,cpg.center_point.z,normal[0],normal[1],normal[2],normal_length)
                self.add((array,obj[1],obj[2]))
    
        elif isinstance(obj[0],ConvexPolyhedron):
            for cpg in obj[0].convex_polygons:
                self.add((cpg,obj[1],obj[2]),normal_length = normal_length)
        else:
            raise ValueError('Cannot add object with type:{}'.format(type(obj[0])))





    def ForceEqualAxisScaling(self, ax):
        ''' See: https://stackoverflow.com/a/31364297
        Matplotlib plots get automatically squished to use up as much of the drawn volume as possible.
        For this application, this might cause us to erroneously think that the objects drawn are incorrect.
        This function forces a 1:1:1 aspect ratio in all axes, centers the objects,
        and changes the view to orthographic perspective (AKA no perspective). '''

        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()

        x_range = abs(x_limits[1] - x_limits[0])
        x_middle = np.mean(x_limits)
        y_range = abs(y_limits[1] - y_limits[0])
        y_middle = np.mean(y_limits)
        z_range = abs(z_limits[1] - z_limits[0])
        z_middle = np.mean(z_limits)

        #The plot bounding box is a sphere in the sense of the infinity norm,
        #hence I call half the max range the plot radius
        plot_radius = 0.5*max([x_range, y_range, z_range])

        ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
        ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
        ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

        #Set orthographic perspective. This might help prevent visual defects (e.g. 2 equal distances looking unequal)
        ax.set_proj_type('ortho')