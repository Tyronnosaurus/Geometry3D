# -*- coding: utf-8 -*-
import math
import unittest

from numpy.polynomial.hermite import poly2herm
from Geometry3D import *
import Geometry3D

a = origin()
b = Point(1,0,1)
c = Point(0,1,1)
d = Point(1,1,2)

class ConvexPolygonTest(unittest.TestCase):
    def test_polygon_length(self):
        self.assertAlmostEqual(
            ConvexPolygon((a,b,c,d)).length(),
            4 * math.sqrt(2)
        )
    def test_polygon_area(self):
        self.assertAlmostEqual(
            ConvexPolygon((a,b,c,d)).area(),
            math.sqrt(3)
        )

    def test_polygon_in_plane(self):
        self.assertTrue(
            ConvexPolygon((a,b,c,d)) in Plane(origin(),Vector(1,1,-1))
        )
        self.assertFalse(
            ConvexPolygon((a,b,c,d)) in Plane(origin(),Vector(1,1.1,-1))
        )
    
    def test_polygon_contains_point(self):
        self.assertTrue(
            origin() in ConvexPolygon((a,b,c,d))
        )

        self.assertTrue(
            Point(0.5,0.5,1) in ConvexPolygon((a,b,c,d))
        )

    def test_polygon_contains_segment(self):
        self.assertTrue(
            Segment(origin(),Point(0.5,0.5,1)) in ConvexPolygon((a,b,c,d))
        )
        self.assertTrue(
            Segment(origin(),Point(1,1,2)) in ConvexPolygon((a,b,c,d))
        )
        self.assertFalse(
            Segment(origin().move(Vector(0,0,1)),
            Point(1,1,2).move(Vector(0,0,1))
            ) in ConvexPolygon((a,b,c,d))
        )

    def test_polygon_eq(self):
        self.assertEqual(
            ConvexPolygon((a,b,d,c)),
            ConvexPolygon((c,a,b,d))
        )
        # self.assertNotEqual(
        #     ConvexPolygon((a,b,d,c)),
        #     -ConvexPolygon((c,a,b,d))
        # )

    def test_polygon_eq_without_normal(self):
        self.assertTrue(
            ConvexPolygon((a,b,d,c)) == (ConvexPolygon((c,a,b,d)))
        )
        self.assertTrue(
            ConvexPolygon((a,b,d,c)) == (-ConvexPolygon((c,a,b,d)))
        )
    
    def test_polygon_hash(self):
        s = set()
        s.add(ConvexPolygon((a,b,d,c)))
        s.add(ConvexPolygon((c,a,b,d)))
        self.assertEqual(len(s),1)

    def test_polygon_move(self):
        v = Vector(1,2,3)
        cpg0 = ConvexPolygon((a,b,d,c))
        cpg1 = ConvexPolygon((a.move(v),b.move(v),c.move(v),d.move(v)))
        self.assertEqual(cpg0.move(v),cpg1)

    def test_polygon_Parallelogram(self):
        self.assertTrue(Parallelogram(origin(),Vector(1,0,0),Vector(2,0,1)) == (ConvexPolygon((origin(),Point(3,0,1),Point(1,0,0),Point(2,0,1)))))
        self.assertTrue(Parallelogram(origin(),Vector(1,0,0),Vector(2,0,1)) == (ConvexPolygon((origin(),Point(2,0,1),Point(1,0,0),Point(3,0,1)))))




e = origin()
f = Point(0,2,2)
g = Point(2,2,2)
h = Point(2,0,0)
i = Point(1,1,1)
poly = ConcavePolygon((e,f,g,h,i))

class ConcavePolygonTest(unittest.TestCase):

    def test_polygon_length(self):
        #Compare with lenght of perimeter, calculated manually
        self.assertAlmostEqual( poly.length() , 2*math.sqrt(8)+2+2*math.sqrt(3) )

    def test_polygon_in_plane(self):
        self.assertTrue( poly in Plane(origin(),Vector(0,1,-1)) )
        self.assertFalse( poly in Plane(origin(),Vector(0,1,0)) )    #Random plane

    # def test_polygon_contains_point(self):    #Need to implement __contains__() first
    #     self.assertTrue(
    #         origin() in poly
    #     )

    #     self.assertTrue(
    #         Point(0.5,0.5,1) in ConvexPolygon((a,b,c,d))
    #     )

    def test_polygon_eq(self):
        self.assertEqual( poly , ConcavePolygon((i,h,g,f,e)) )  #Check equality for different presentations of same polygon
        # self.assertNotEqual(      #Need to implement != method for ConcavePolygon
        #     poly,
        #     ConcavePolygon((e,f,g,i,h))
        # )
        
    
    def test_polygon_eq_without_normal(self):
        self.assertTrue( poly == ConcavePolygon((h,i,e,f,g)) )  #Check equality when verts are shifted
        self.assertTrue( poly == -ConcavePolygon((h,i,e,f,g)) ) #Check equality when negating (reversing the normal)


    def test_polygon_hash(self):
        self.assertEqual( poly.__hash__() , poly.__hash__() )  #Must be equal to itself

        self.assertEqual( poly.__hash__() , ConcavePolygon((g,h,i,e,f)).__hash__() ) #Same polygon, but shifted

        self.assertEqual( poly.__hash__() , ConcavePolygon((i,h,g,f,e)).__hash__() ) #Same polygon, but reversed

        #Same polygon, but reversed & shifted. Test all shift possibilities, because _getStandarizedVerticesList() has some tricky list slicing which might have bugs
        self.assertEqual( poly.__hash__() , ConcavePolygon((f,e,i,h,g)).__hash__() )
        self.assertEqual( poly.__hash__() , ConcavePolygon((g,f,e,i,h)).__hash__() )
        self.assertEqual( poly.__hash__() , ConcavePolygon((h,g,f,e,i)).__hash__() )
        self.assertEqual( poly.__hash__() , ConcavePolygon((i,h,g,f,e)).__hash__() )
        self.assertEqual( poly.__hash__() , ConcavePolygon((e,i,h,g,f)).__hash__() )

        self.assertNotEqual( poly.__hash__() , ConcavePolygon((e,f,g,i,h)).__hash__() )  #Same vertices but different polygon
        
        #Check that equal polygons are identified correctly and discarded in a set
        s = set()
        s.add(poly)
        s.add(ConcavePolygon((i,h,g,f,e)))  #Same polygon, different vertex order
        self.assertEqual(len(s),1)
        s.add(ConcavePolygon((e,f,g,i,h)))  #Different polygon, should have a different hash
        self.assertEqual(len(s),2)


    def test_polygon_move(self):
        v = Vector(1,2,3)
        cpg0 = poly
        cpg1 = ConcavePolygon((e.move(v),f.move(v),g.move(v),h.move(v),i.move(v)))
        self.assertEqual(cpg0.move(v),cpg1)