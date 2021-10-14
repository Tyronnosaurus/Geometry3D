# -*- coding: utf-8 -*-
"""Polygon Module"""
from .body import GeoBody
from .point import Point
from ..utils.vector import Vector
from .line import Line
from .segment import Segment
from .plane import Plane
from ..utils.constant import *
from ..utils.vector import x_unit_vector,y_unit_vector

import copy
import math



class Polygon(GeoBody):
    '''
    Should not be instanced.
    Parent class to ConvexPolygon and ConcavePolygon.
    '''

    class_level = 4 # the class level of Polygon

    def __init__(self):
        raise NotImplementedError("Do not use Polygon class. Use ConvexPolygon or ConcavePolygon instead.")


    def _get_center_point(self):
        """
        **Input:**
        
        - points: tuple of points

        **Output:**

        - The center point of given points
        """
        x,y,z =(0,0,0)
        num_points = len(self.points)
        for point in self.points:
            x += point.x
            y += point.y
            z += point.z
        return Point(float(x) / num_points,float(y) / num_points,float(z) / num_points)


    def segments(self):
        """
        **Input:**
        
        - self

        **Output:**

        - iterator of segments
        """
        for i in range(len(self.points)):
            index_0 = i
            if i == len(self.points) - 1:
                index_1 = 0
            else:
                index_1 = i + 1
            yield Segment(self.points[index_0],self.points[index_1])


    def length(self):
        """return the total length of ConvexPolygon""" 
        length = 0
        for segment in self.segments():
            length += segment.length()
        return length

    def in_(self,other):
        """
        **Input:**
        
        - self: ConvexPolygon
        
        - other: Plane

        **Output:**

        - whether self in other
        """
        if isinstance(other,Plane):
            return self.plane == other
        else:
            raise NotImplementedError("")


    def __eq__(self,other):
        if isinstance(other,type(self)):
            return (hash(self) == hash(other))
        else:
            return False


    def move(self,v):
        """Return the ConvexPolygon that you get when you move self by vector v, self is also moved"""
        if isinstance(v,Vector):
            point_list = []
            for point in self.points:
                point_list.append(point.move(v))
            self.points = tuple(point_list)
            self.plane = Plane(self.points[0],self.points[1],self.points[2])
            self.center_point = self._get_center_point()
            polyClass = type(self) #Return polygon using same class as original (either ConvexPolygon or ConcavePolygon)
            return polyClass(self.points)
        else:
            raise NotImplementedError("The second parameter for move function must be Vector")




def get_circle_point_list(center,normal,radius,n=10):
    if n <= 2:
        raise ValueError("n must be at least 3 to construct an inscribed polygon for a circle")
    import math,copy
    if normal.angle(x_unit_vector()) < SMALL_ANGLE:
        base_vector = y_unit_vector()
        if normal.angle(y_unit_vector()) < SMALL_ANGLE:
            raise ValueError("Bug detected! please contact the author")
    else:
        base_vector = x_unit_vector()
    v1 = normal.normalized().cross(base_vector).normalized() 
    v2 = normal.normalized().cross(v1)
    v1 = v1 * radius
    v2 = v2 * radius
    point_list = []
    for i in range(n):
        angle_i = math.pi * 2 / n * i
        point_list.append(copy.deepcopy(center).move(v1 * math.cos(angle_i) + v2 * math.sin(angle_i)))
    return point_list

def get_triangle_area(pa,pb,pc):
    """
    **Input:**
    
    - pa: a Point
    
    - pb: a Point
    
    - pc: a Point

    **Output:**
    
    The area of the triangle composed by pa, pb and pc.

    Heron's formula is used
    """
    a = pa.distance(pb)
    b = pb.distance(pc)
    c = pc.distance(pa)
    p = (a + b + c) / 2
    return math.sqrt(p * (p - a) * (p - b) * (p - c))


class ConvexPolygon(Polygon):

    @classmethod
    def Circle(cls,center,normal,radius,n=10):
        """
        A special function for creating an inscribed convex polygon of a circle

        **Input:**

        - Center: The center point of the circle

        - normal: The normal vector of the circle

        - radius: The radius of the circle

        - n=10: The number of Points of the ConvexPolygon

        **Output:**

        - An inscribed convex polygon of a circle.
        """
        return cls(get_circle_point_list(center,normal,radius,n))

    @classmethod
    def Parallelogram(cls,base_point,v1,v2):
        """
        A special function for creating Parallelogram

        **Input:**

        - base_point: a Point

        - v1, v2: two Vectors

        **Output:**

        - A parallelogram which is a ConvexPolygon instance.
        """
        if isinstance(base_point,Point) and isinstance(v1,Vector) and isinstance(v2,Vector):
            if v1.length() == 0 or v2.length == 0:
                raise ValueError("The length for the two vector shouldn't be zero")
            elif v1.parallel(v2):
                raise ValueError("The two vectors shouldn't be parallel to each other")
            else:
                return cls((
                    base_point,
                    copy.deepcopy(base_point).move(v1),
                    copy.deepcopy(base_point).move(v2),
                    copy.deepcopy(base_point).move(v1).move(v2)
                ))
        else:
            raise TypeError("Parallelogram should be initialized with Point, Vector and Vector, but the given types are %s, %s and %s" %(type(base_point),type(v1),type(v2)))
    

    def __init__(self,pts,reverse = False, check_convex=False):
        """
        - ConvexPolygons(points)
        points: a tuple of points.

        The points needn't be in order. They'll get automatically reordered to form the convex polygon.

        The convexity should be guaranteed. This function **will not** check the convexity.
        If the Polygon is not convex, there might be errors.
        """
        # merge same points
        points = copy.deepcopy(pts)
        self.points = sorted(set(points),key=points.index)
        if len(points) < 3:
            raise ValueError('Cannot build a polygon with number of points smaller than 3')
        if reverse:
            self.plane = -Plane(self.points[0],self.points[1],self.points[2])
        else:
            self.plane = Plane(self.points[0],self.points[1],self.points[2])
    
        self.center_point = self._get_center_point()

        self._check_and_sort_points()
        
    
    def area(self):
        """
        **Input:**
        
        - self

        **Output:**
        
        - The area of the convex polygon
        """
        area = 0
        for i in range(len(self.points)):
            index_0 = i
            if i == len(self.points) - 1:
                index_1 = 0
            else:
                index_1 = i + 1
            area += get_triangle_area(self.center_point,self.points[index_0],self.points[index_1])
        return area

    def _check_and_sort_points(self):
        """
        **Input:**
        
        -self

        **Output:**

        - True for check passed
        
        - False for check not passed

        This is only a **weak** check, passing the check doesn't guarantee it is a convex polygon
        """
        the_normal = (self.plane.n).normalized()
        v0 = Vector(self.center_point,self.points[0]).normalized()
        v1 = the_normal.cross(v0)
        angle_point_dict = dict() # dict[angle] = Point
        for point in self.points:
            if not point in self.plane:
                raise ValueError('Convex Check Fails Because {} Is Not On {}'.format(point,self.plane))
            pv = point.pv() - self.center_point.pv()
            y_coordinate = pv * v0
            z_coordinate = pv * v1
            # the range of vector_angle is [0,2*pi)
            vector_angle = math.atan2(z_coordinate,y_coordinate)
            if vector_angle < 0:
                vector_angle += 2 * math.pi
            angle_point_dict[vector_angle] = point
        point_list = [angle_point_dict[angle] for angle in sorted(angle_point_dict)]
        self.points = tuple(point_list)
        return True
    
    def __repr__(self):
        return "ConvexPolygon({})".format(self.points)

    def __contains__(self, other):
        """Checks if a point or segment lies in a ConvexPolygon"""
        if isinstance(other,Point):
            r1 = other in self.plane
            # requirement 1: the point is on the plane
            the_normal = self.plane.n.normalized()
            r2 = True
            # requirement 2: the point is inside the polygon
            for i in range(len(self.points)):
                # check if the point lies in the inside direction of every segment
                index_0 = i
                if i == len(self.points) - 1:
                    index_1 = 0
                else:
                    index_1 = i + 1
                # self.points[index_0] and self.points[index_1] is the i^th segment
                v0 = Vector(self.points[index_0],self.points[index_1])
                v1 = the_normal.cross(v0)
                vec = Vector(self.points[index_0],other)
                if vec * v1 < - get_eps():
                    r2 = False
                    break
            return r1 and r2
        
        elif isinstance(other,Segment):
            return (other.start_point in self) and (other.end_point in self)
        else:
            return NotImplementedError("")
    
    
    
    def _get_point_hash_sum(self):
        """return the sum of hash of all points"""
        hash_sum = 0
        for point in self.points:
            hash_sum += hash(point)
        return hash_sum

    # the hash function is not accurate
    # in some extreme case, this function may fail
    # which means it's vulnerable to attacks.
    def __hash__(self):
        """return the has of the convexpolygon"""
        return hash(("ConvexPolygon",
        round(self._get_point_hash_sum(),SIG_FIGURES), 
        hash(self.plane) + hash(-self.plane),
        hash(self.plane) * hash(-self.plane)
        ))


    def eq_with_normal(self,other):
        """return whether self equals with other considering the normal"""
        if isinstance(other,ConvexPolygon):
            return (self.hash_with_normal() == other.hash_with_normal())
        else:
            return False

    def hash_with_normal(self):
        """return the hash value considering the normal"""
        return hash(("ConvexPolygon",
            round(self._get_point_hash_sum(),SIG_FIGURES-5),
            hash(self.plane)
            ))

    def __neg__(self):
        """return the negative ConvexPolygon by reverting the normal"""
        return ConvexPolygon(self.points,reverse=True)


Parallelogram = ConvexPolygon.Parallelogram
Circle = ConvexPolygon.Circle




class ConcavePolygon(Polygon):

    def __init__(self,pts,reverse = False, check_convex=False):
        """
        - ConcavePolygons(points)
        points: a tuple of points.

        The points need to be in order.
        """
        points = copy.deepcopy(pts)

        # Convert to list, remove duplicates
        self.points = sorted(set(points),key=points.index)

        #TODO: Remove redundant points (those which are aligned between previous and next point)

        if len(points) < 3:
            raise ValueError('Cannot build a polygon with number of points smaller than 3')
        if reverse:
            self.plane = -Plane(self.points[0],self.points[1],self.points[2])
        else:
            self.plane = Plane(self.points[0],self.points[1],self.points[2])
    
        self.center_point = self._get_center_point()


    def _get_points_hash(self):
        #Returns hash of a polygon's ordered list of vertices. We have to keep in mind that:
        #  abcde != ceabd    Unlike with convexPolygon, vertices don't get automatically ordered. Therefore, order matters.
        #  abcde == cdeab    Vertices list can start at any of the vertices.
        #  abcde == edcba    Vertices list can be clockwise or counterclockwise
        #TODO: Surely this process can be optimized

        standarizedVerts = self._getStandarizedVerticesList()   #Reorder vertices so that different presentations of the same polygon end
                                                                #up with the same standard presentation (shift & rotate only, no zig-zags)
        return( hash(tuple(standarizedVerts)) )                 #Lists can't be hashed. Tuples can, and the order of their items matters



    def _getStandarizedVerticesList(self):
        #Our startegy will be to reorder the list of vertices in a standarized way that always has the same
        #starting point and orientation regardless of how it was originally ordered.

        #To choose start vertex, we use a function that always returns the same one no matter how the vertices are ordered
        i_start = self._pointWithMinXYZ(self.points)

        #Decide orientation (akin to clockwise/counterclowise, but in 3D)
        #If the start vertex was at position i, here we choose wether to continue advancing towards the vertex i-1 or i+1
        #Use the same method to select one of the two vertices next to the start vertex.
        vert_before = self.points[i_start-1] if i_start>0                  else self.points[-1]   #Point to the left of start point (roll over to the end if necessary)
        vert_after  = self.points[i_start+1] if i_start<len(self.points)-1 else self.points[0]    #Point to the right of start point (roll over to the beginning if necessary)
        i_second = self._pointWithMinXYZ([vert_before, vert_after])                          #Choose which has lower X (if equal, lower Y, then lower Z)
        reversed = (i_second == 0)                                                              #If it's the one on the left, we must reverse the order


        #Rebuild the vertices list, using list slicing
        newPointsList = []
        if(not reversed):
            #Shift list by slicing in two halves (cutting at the start vertex S). For example:  abcSde --> Sde+abc --> Sdeabc
            newPointsList.extend( self.points[i_start:] )
            newPointsList.extend( self.points[:i_start] )
        else:
            newPointsList.extend( self.points[0:i_start+1][::-1] )  #Similar method, but we also reverse the sliced cuts with [::-1]
            newPointsList.extend( self.points[i_start+1:][::-1] )
        

        return(newPointsList)


    def _pointWithMinXYZ(self, pointList):
        '''Given a list of Points, returns index of point with lowest X.
           If multiple are found, use the one with lowest Y. Same goes for Z.'''
        verts = pointList
        minX = min(v.x for v in verts)              #Find min value of x
        verts = [v for v in verts if v.x == minX]   #Keep only vertices that have that minX
        minY = min(v.y for v in verts)              #Idem for minY and minZ
        verts = [v for v in verts if v.y == minY]
        minZ = min(v.z for v in verts)
        verts = [v for v in verts if v.z == minZ]
        p = verts[0]     #Return the first (and theoretically only) point left after filtering

        for i, v in enumerate(pointList):   #Find index of p in the original list
            if v.__eq__(p):
                p_index = i

        return(p_index)


    def __hash__(self):
        """Return the hash of the concavePolygon"""
        return hash(("ConcavePolygon",
                     self._get_points_hash(),   #Hashes vertices list (for same polygon, will yield same hash regardless of shifting or orientation of verts list)
                     hash(self.plane) + hash(-self.plane),
                     hash(self.plane) * hash(-self.plane)
        ))


    def __neg__(self):
        """return the negative ConvexPolygon by reverting the normal. Usable by adding a minus sign: -MyConcavePolygon"""
        return ConcavePolygon(self.points,reverse=True)



__all__ = ("ConvexPolygon","ConcavePolygon","Parallelogram","get_circle_point_list","Circle")