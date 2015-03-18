
#ifndef MARCHINGCUBE_HPP
#define MARCHINGCUBE_HPP

#include <iostream>
#include <vector>

#include <rw/geometry/PlainTriMesh.hpp>
//#include <rw/geometry/Face.hpp>
#include <rw/math/Vector3D.hpp>
#include "OcVoxel.hpp"

static const char test3[24] = {
/*   5: 0,    2,                 */    5,
/*  33: 0,             5,        */    1,
/* 129: 0,                   7,  */    4,
/*  10:    1,    3,              */    5,
/*  18:    1,       4,           */    1,
/*  66:    1,             6,     */    2,
/*  36:       2,       5,        */    2,
/* 132:       2,             7,  */    3,
/*  24:          3, 4,           */    4,
/*  72:          3,       6,     */    3,
/*  80:             4,    6,     */    6,
/* 160:                5,    7,  */    6,
/*  95: 0, 1, 2, 3, 4,    6,     */   -6,
/* 175: 0, 1, 2, 3,    5,    7,  */   -6,
/* 183: 0, 1, 2,    4, 5,    7,  */   -3,
/* 231: 0, 1, 2,       5, 6, 7,  */   -4,
/* 123: 0, 1,    3, 4, 5, 6,     */   -3,
/* 219: 0, 1,    3, 4,    6, 7,  */   -2,
/* 189: 0,    2, 3, 4, 5,    7,  */   -2,
/* 237: 0,    2, 3,    5, 6, 7,  */   -1,
/* 245: 0,    2,    4, 5, 6, 7,  */   -5,
/* 126:    1, 2, 3, 4, 5, 6,     */   -4,
/* 222:    1, 2, 3, 4,    6, 7,  */   -1,
/* 250:    1,    3, 4, 5, 6, 7,  */   -5
};

static const char tiling3_1[24][6] = {
/*   5: 0,    2,                 */  {  0,  8,  3,  1,  2, 10 },
/*  33: 0,             5,        */  {  9,  5,  4,  0,  8,  3 },
/* 129: 0,                   7,  */  {  3,  0,  8, 11,  7,  6 },
/*  10:    1,    3,              */  {  1,  9,  0,  2,  3, 11 },
/*  18:    1,       4,           */  {  0,  1,  9,  8,  4,  7 },
/*  66:    1,             6,     */  {  9,  0,  1,  5, 10,  6 },
/*  36:       2,       5,        */  {  1,  2, 10,  9,  5,  4 },
/* 132:       2,             7,  */  { 10,  1,  2,  6, 11,  7 },
/*  24:          3, 4,           */  {  8,  4,  7,  3, 11,  2 },
/*  72:          3,       6,     */  {  2,  3, 11, 10,  6,  5 },
/*  80:             4,    6,     */  {  5, 10,  6,  4,  7,  8 },
/* 160:                5,    7,  */  {  4,  9,  5,  7,  6, 11 },
/*  95: 0, 1, 2, 3, 4,    6,     */  {  5,  9,  4, 11,  6,  7 },
/* 175: 0, 1, 2, 3,    5,    7,  */  {  6, 10,  5,  8,  7,  4 },
/* 183: 0, 1, 2,    4, 5,    7,  */  { 11,  3,  2,  5,  6, 10 },
/* 231: 0, 1, 2,       5, 6, 7,  */  {  7,  4,  8,  2, 11,  3 },
/* 123: 0, 1,    3, 4, 5, 6,     */  {  2,  1, 10,  7, 11,  6 },
/* 219: 0, 1,    3, 4,    6, 7,  */  { 10,  2,  1,  4,  5,  9 },
/* 189: 0,    2, 3, 4, 5,    7,  */  {  1,  0,  9,  6, 10,  5 },
/* 237: 0,    2, 3,    5, 6, 7,  */  {  9,  1,  0,  7,  4,  8 },
/* 245: 0,    2,    4, 5, 6, 7,  */  {  0,  9,  1, 11,  3,  2 },
/* 126:    1, 2, 3, 4, 5, 6,     */  {  8,  0,  3,  6,  7, 11 },
/* 222:    1, 2, 3, 4,    6, 7,  */  {  4,  5,  9,  3,  8,  0 },
/* 250:    1,    3, 4, 5, 6, 7,  */  {  3,  8,  0, 10,  2,  1 }
};

static const char tiling3_2[24][12] = {
/*   5: 0,    2,                 */  { 10,  3,  2, 10,  8,  3, 10,  1,  0,  8, 10,  0 },
/*  33: 0,             5,        */  {  3,  4,  8,  3,  5,  4,  3,  0,  9,  5,  3,  9 },
/* 129: 0,                   7,  */  {  6,  8,  7,  6,  0,  8,  6, 11,  3,  0,  6,  3 },
/*  10:    1,    3,              */  { 11,  0,  3, 11,  9,  0, 11,  2,  1,  9, 11,  1 },
/*  18:    1,       4,           */  {  7,  9,  4,  7,  1,  9,  7,  8,  0,  1,  7,  0 },
/*  66:    1,             6,     */  {  6,  1, 10,  6,  0,  1,  9,  0,  6,  9,  6,  5 },
/*  36:       2,       5,        */  {  4, 10,  5,  4,  2, 10,  4,  9,  1,  2,  4,  1 },
/* 132:       2,             7,  */  {  7,  2, 11,  7,  1,  2,  7,  6, 10,  1,  7, 10 },
/*  24:          3, 4,           */  {  2,  7, 11,  2,  4,  7,  2,  3,  8,  4,  2,  8 },
/*  72:          3,       6,     */  {  5, 11,  6,  5,  3, 11,  5, 10,  2,  3,  5,  2 },
/*  80:             4,    6,     */  {  8,  6,  7,  8, 10,  6,  8,  4,  5, 10,  8,  5 },
/* 160:                5,    7,  */  { 11,  5,  6, 11,  9,  5, 11,  7,  4,  9, 11,  4 },
/*  95: 0, 1, 2, 3, 4,    6,     */  {  6,  5, 11,  5,  9, 11,  4,  7, 11,  4, 11,  9 },
/* 175: 0, 1, 2, 3,    5,    7,  */  {  7,  6,  8,  6, 10,  8,  5,  4,  8,  5,  8, 10 },
/* 183: 0, 1, 2,    4, 5,    7,  */  {  6, 11,  5, 11,  3,  5,  2, 10,  5,  2,  5,  3 },
/* 231: 0, 1, 2,       5, 6, 7,  */  { 11,  7,  2,  7,  4,  2,  8,  3,  2,  8,  2,  4 },
/* 123: 0, 1,    3, 4, 5, 6,     */  { 11,  2,  7,  2,  1,  7, 10,  6,  7, 10,  7,  1 },
/* 219: 0, 1,    3, 4,    6, 7,  */  {  5, 10,  4, 10,  2,  4,  1,  9,  4,  1,  4,  2 },
/* 189: 0,    2, 3, 4, 5,    7,  */  { 10,  1,  6,  1,  0,  6,  6,  0,  9,  5,  6,  9 },
/* 237: 0,    2, 3,    5, 6, 7,  */  {  4,  9,  7,  9,  1,  7,  0,  8,  7,  0,  7,  1 },
/* 245: 0,    2,    4, 5, 6, 7,  */  {  3,  0, 11,  0,  9, 11,  1,  2, 11,  1, 11,  9 },
/* 126:    1, 2, 3, 4, 5, 6,     */  {  7,  8,  6,  8,  0,  6,  3, 11,  6,  3,  6,  0 },
/* 222:    1, 2, 3, 4,    6, 7,  */  {  8,  4,  3,  4,  5,  3,  9,  0,  3,  9,  3,  5 },
/* 250:    1,    3, 4, 5, 6, 7,  */  {  2,  3, 10,  3,  8, 10,  0,  1, 10,  0, 10,  8 }
};


class MarchingCube {
 public:

    static void Triangulate(rw::math::Vector3D<float>* p,
			    unsigned char type,OcVoxel* start,
			    rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<float> >& triangles);


    static void PrintStat();
 private:

    //  double xmin, xmax, ymin, ymax, zmin, zmax;
    //  unsigned int xint, yint, zint;
    //  double coefx, coefy, coefz;
    //    double deltax, deltay, deltaz;
    static int stat[256];

	static bool test_face(char face,rw::math::Vector3D<float> *p);
    //  int*** corners;

    //  std::vector<CTriangle> triangles;
    static rw::math::Vector3D<float> edges[];

    static void CreateTriangle(const rw::math::Vector3D<float>& p1,
			       const rw::math::Vector3D<float>& p2,
			       const rw::math::Vector3D<float>& p3,
			       rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<float> >& triangles);

  static void TrianglesAlongEdge(unsigned char type, rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<float> >& triangles);
  static void TrianglesInCorners(unsigned char type,OcVoxel* start, rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<float> >& triangles);

  static void TrianglesAlongEdgeRoot(unsigned char type, rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<float> >& triangles);
   static void TrianglesInCornersRoot(unsigned char type, rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<float> >& triangles);
};

#endif //#ifndef MARCHINGCUBE_H
