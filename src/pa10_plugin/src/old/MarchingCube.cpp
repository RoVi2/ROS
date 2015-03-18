#include "MarchingCube.hpp"


const unsigned char BIT1 = 0x01;
const unsigned char BIT2 = 0x02;
const unsigned char BIT3 = 0x04;
const unsigned char BIT4 = 0x08;

const unsigned char BIT5 = 0x10;
const unsigned char BIT6 = 0x20;
const unsigned char BIT7 = 0x40;
const unsigned char BIT8 = 0x80;

//const int XY_PLANE = 0;
//const int YZ_PLANE = 1;
//const int ZX_PLANE = 2;

using namespace rw::math;
using namespace rw::geometry;


Vector3D<float> MarchingCube::edges[12];
int MarchingCube::stat[256];

void MarchingCube::PrintStat() {
  for (unsigned int i = 0; i<256; i++) {
    if (stat[i] > 0)
      std::cout<<"Stat["<<i<<"]="<<stat[i]<<std::endl;
  }
}

void MarchingCube::CreateTriangle(const Vector3D<float>& p1,
				  const Vector3D<float>& p2,
				  const Vector3D<float>& p3,
				  rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<float> >& triangles ) {
    
    Vector3D<float> normal = cross(Vector3D<float>(p2-p1), Vector3D<float>(p3-p1));
	
	triangles.add(TriangleN1<float>(p1,p2,p3,normal));
}

bool MarchingCube::test_face(char face, Vector3D<float>* p)
//-----------------------------------------------------------------------------
{
/* 
	float A,B,C,D ;

  switch( face )
  {
  case -1 : case 1 :  A = (*p)[0] ;  B = -(*p)[4] ;  C = (*p)[5] ;  D = -(*p)[1] ;  break ;
  case -2 : case 2 :  A = (*p)[1] ;  B = -(*p)[5] ;  C = (*p)[6] ;  D = -(*p)[2] ;  break ;
  case -3 : case 3 :  A = (*p)[2] ;  B = -(*p)[6] ;  C = (*p)[7] ;  D = -(*p)[3] ;  break ;
  case -4 : case 4 :  A = (*p)[3] ;  B = -(*p)[7] ;  C = (*p)[4] ;  D = -(*p)[0] ;  break ;
  case -5 : case 5 :  A = (*p)[0] ;  B = -(*p)[3] ;  C = (*p)[2] ;  D = -(*p)[1] ;  break ;
  case -6 : case 6 :  A = (*p)[4] ;  B = -(*p)[7] ;  C = (*p)[6] ;  D = -(*p)[5] ;  break ;
  default : std::cout << "Invalid face code %d\n" << std::endl;
  };

  if( fabs( A*C - B*D ) < FLT_EPSILON ) //DBL_EPSILON
    return face >= 0 ;
  return face * A * ( A*C - B*D ) >= 0  ;  // face and A invert signs
*/
  return 1;
}

void MarchingCube::Triangulate(Vector3D<float>* p,
			       unsigned char type,OcVoxel* start,
				   rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<float> >& triangles) {
    stat[(unsigned char)type]++;
    //    edges = new Vector3D<float>[12];

    edges[0] = Vector3D<float>((p[0]+p[1])/2.0);
    edges[1] = Vector3D<float>((p[1]+p[2])/2.0);
    edges[2] = Vector3D<float>((p[2]+p[3])/2.0);
    edges[3] = Vector3D<float>((p[3]+p[0])/2.0);
    edges[4] = Vector3D<float>((p[4]+p[5])/2.0);
    edges[5] = Vector3D<float>((p[5]+p[6])/2.0);
    edges[6] = Vector3D<float>((p[6]+p[7])/2.0);
    edges[7] = Vector3D<float>((p[7]+p[4])/2.0);
    edges[8] = Vector3D<float>((p[0]+p[4])/2.0);
    edges[9] = Vector3D<float>((p[1]+p[5])/2.0);
    edges[10] = Vector3D<float>((p[3]+p[7])/2.0);
    edges[11] = Vector3D<float>((p[2]+p[6])/2.0);


    int cnt = 0;
    if (type & BIT1)
	++cnt;
    if (type & BIT2)
	++cnt;
    if (type & BIT3)
	++cnt;
    if (type & BIT4)
	++cnt;
    if (type & BIT5)
	++cnt;
    if (type & BIT6)
	++cnt;
    if (type & BIT7)
	++cnt;
    if (type & BIT8)
	++cnt;

    int triangleCount = triangles.size();
    bool flipNormals = false;
	int index = 0;

    if (cnt > 4) {
	type = 0xFF^type;
	flipNormals = true;
    }

    switch (type) {
    case 0x00:
	break;
    case 0x03:
    case 0x06:
    case 0x0C:
    case 0x09:
    case 0x30:
    case 0x60:
    case 0xC0:
    case 0x90:
    case 0x11:
    case 0x22:
    case 0x44:
    case 0x88:
	TrianglesAlongEdge(type, triangles);
	break;

	//Triangulation Case 3 section
//	5: 0,    2,                 */    5,
//  33: 0,             5,        */    1,
// 129: 0,                   7,  */    4,
//  10:    1,    3,              */    5,
//  18:    1,       4,           */    1,
//  66:    1,             6,     */    2,
//  36:       2,       5,        */    2,
// 132:       2,             7,  */    3,
//  24:          3, 4,           */    4,
//  72:          3,       6,     */    3,
//  80:             4,    6,     */    6,
// 160:                5,    7,  */    6,
	//10,  3,  2, 10,  8,  3, 10,  1,  0,  8, 10,  0
	case 0x05: //tiling3_2[0][12] v0-v2
		index = 0;
		if( test_face( test3[index],p) ){
		CreateTriangle(edges[0], edges[8], edges[3], triangles);
		CreateTriangle(edges[1], edges[2], edges[11], triangles);
		CreateTriangle(edges[1], edges[0], edges[2], triangles);
		CreateTriangle(edges[0], edges[3], edges[2], triangles);
		}
		else
		{
		CreateTriangle(edges[0], edges[8], edges[3], triangles);
		CreateTriangle(edges[1], edges[2], edges[11], triangles);
		std::cout <<"case 3.1 "<< std::endl;
		}
	break;
	//3,  4,  8,  3,  5,  4,  3,  0,  9,  5,  3,  9	
	case 33: //v0-v5
	    index = 1;
		if( test_face( test3[index],p) ){
		CreateTriangle(edges[0], edges[8], edges[3], triangles);
		CreateTriangle(edges[4], edges[9], edges[5], triangles);
		CreateTriangle(edges[0], edges[9], edges[8], triangles);
		CreateTriangle(edges[4], edges[8], edges[9], triangles);
		}
		else
		{
		CreateTriangle(edges[0], edges[8], edges[3], triangles);
		CreateTriangle(edges[4], edges[9], edges[5], triangles);
		std::cout <<"case 3.1 "<< std::endl;
		}
	break;
	/*
	// 6,  8,  7,  6,  0,  8,  6, 11,  3,  0,  6,  3 
	case 129:
		index = 2;
		if( test_face( test3[index],*p) ){
		CreateTriangle(edges[tiling3_2[index][0]], edges[tiling3_2[index][1]], edges[tiling3_2[index][2]], triangles);
		CreateTriangle(edges[tiling3_2[index][3]], edges[tiling3_2[index][4]], edges[tiling3_2[index][5]], triangles);
		CreateTriangle(edges[tiling3_2[index][6]], edges[tiling3_2[index][7]], edges[tiling3_2[index][8]], triangles);
		CreateTriangle(edges[tiling3_2[index][9]], edges[tiling3_2[index][10]], edges[tiling3_2[index][11]], triangles);
		}
		else
		{
		CreateTriangle(edges[tiling3_1[index][0]], edges[tiling3_1[index][1]], edges[tiling3_1[index][2]], triangles);
		CreateTriangle(edges[tiling3_1[index][3]], edges[tiling3_1[index][4]], edges[tiling3_1[index][5]], triangles);
		std::cout <<"case 3.1 "<< std::endl;
		}
	break;
	//11,  0,  3, 11,  9,  0, 11,  2,  1,  9, 11,  1 
	*/
	case 10:
		index = 3;
		if( test_face( test3[index],p) ){
		CreateTriangle(edges[0], edges[1], edges[9], triangles);
	    CreateTriangle(edges[2], edges[3], edges[10], triangles);
		CreateTriangle(edges[0], edges[3], edges[1], triangles);
		CreateTriangle(edges[2], edges[1], edges[3], triangles);
		}
		else
		{
		CreateTriangle(edges[0], edges[1], edges[9], triangles);
	    CreateTriangle(edges[2], edges[3], edges[10], triangles);
		std::cout <<"case 3.1 "<< std::endl;
		}
	break;
/*		if (type & BIT1)
	CreateTriangle(edges[0], edges[8], edges[3], triangles);
    if (type & BIT2)
	CreateTriangle(edges[0], edges[1], edges[9], triangles);
    if (type & BIT3)
	CreateTriangle(edges[1], edges[2], edges[11], triangles);
    if (type & BIT4)
	CreateTriangle(edges[2], edges[3], edges[10], triangles);

    if (type & BIT5)
	CreateTriangle(edges[4], edges[7], edges[8], triangles);
    if (type & BIT6)
	CreateTriangle(edges[4], edges[9], edges[5], triangles);
    if (type & BIT7)
	CreateTriangle(edges[5], edges[11], edges[6], triangles);
    if (type & BIT8)
	CreateTriangle(edges[6], edges[10], edges[7], triangles);*/
	case 18: //V1 v4
		index = 4;
		if( test_face( test3[index],p) ){
		CreateTriangle(edges[0], edges[1], edges[9], triangles);
		CreateTriangle(edges[4], edges[7], edges[8], triangles);
		CreateTriangle(edges[1], edges[9], edges[4], triangles);
		CreateTriangle(edges[4], edges[8], edges[0], triangles);
		}
		else
		{
		CreateTriangle(edges[0], edges[1], edges[9], triangles);
		CreateTriangle(edges[4], edges[7], edges[8], triangles);
		std::cout <<"case 3.1 "<< std::endl;
		}
	break;
	case 66: //v1-v6
		index = 5;
		if( test_face( test3[index],p) ){
		CreateTriangle(edges[0], edges[1], edges[9], triangles);
		CreateTriangle(edges[5], edges[11], edges[6], triangles);
		CreateTriangle(edges[1], edges[9], edges[5], triangles);
		CreateTriangle(edges[5], edges[11], edges[1], triangles);
		}
		else
		{
		CreateTriangle(edges[0], edges[1], edges[9], triangles);
		CreateTriangle(edges[5], edges[11], edges[6], triangles);
		std::cout <<"case 3.1 "<< std::endl;
		}
	break;

	case 36: //v2-v5
		index = 6;
		if( test_face( test3[index],p) ){
		CreateTriangle(edges[1], edges[2], edges[11], triangles);
		CreateTriangle(edges[4], edges[9], edges[5], triangles);
		CreateTriangle(edges[9], edges[1], edges[11], triangles);
		CreateTriangle(edges[11], edges[5], edges[1], triangles);
		}
		else
		{
		CreateTriangle(edges[1], edges[2], edges[11], triangles);
		CreateTriangle(edges[4], edges[9], edges[5], triangles);
		std::cout <<"case 3.1 "<< std::endl;
		}
	break;
	/*
	case 132:
		index = 7;
		if( test_face( test3[index],*p) ){
		CreateTriangle(edges[tiling3_2[index][0]], edges[tiling3_2[index][1]], edges[tiling3_2[index][2]], triangles);
		CreateTriangle(edges[tiling3_2[index][3]], edges[tiling3_2[index][4]], edges[tiling3_2[index][5]], triangles);
		CreateTriangle(edges[tiling3_2[index][6]], edges[tiling3_2[index][7]], edges[tiling3_2[index][8]], triangles);
		CreateTriangle(edges[tiling3_2[index][9]], edges[tiling3_2[index][10]], edges[tiling3_2[index][11]], triangles);
		}
		else
		{
		CreateTriangle(edges[tiling3_1[index][0]], edges[tiling3_1[index][1]], edges[tiling3_1[index][2]], triangles);
		CreateTriangle(edges[tiling3_1[index][3]], edges[tiling3_1[index][4]], edges[tiling3_1[index][5]], triangles);
		std::cout <<"case 3.1 "<< std::endl;
		}
	break;
	case 24:
		index = 8;
		if( test_face( test3[index],*p) ){
		CreateTriangle(edges[tiling3_2[index][0]], edges[tiling3_2[index][1]], edges[tiling3_2[index][2]], triangles);
		CreateTriangle(edges[tiling3_2[index][3]], edges[tiling3_2[index][4]], edges[tiling3_2[index][5]], triangles);
		CreateTriangle(edges[tiling3_2[index][6]], edges[tiling3_2[index][7]], edges[tiling3_2[index][8]], triangles);
		CreateTriangle(edges[tiling3_2[index][9]], edges[tiling3_2[index][10]], edges[tiling3_2[index][11]], triangles);
		}
		else
		{
		CreateTriangle(edges[tiling3_1[index][0]], edges[tiling3_1[index][1]], edges[tiling3_1[index][2]], triangles);
		CreateTriangle(edges[tiling3_1[index][3]], edges[tiling3_1[index][4]], edges[tiling3_1[index][5]], triangles);
		std::cout <<"case 3.1 "<< std::endl;
		}
	break;
	case 72:
		index = 9;
		if( test_face( test3[index],*p) ){
		CreateTriangle(edges[tiling3_2[index][0]], edges[tiling3_2[index][1]], edges[tiling3_2[index][2]], triangles);
		CreateTriangle(edges[tiling3_2[index][3]], edges[tiling3_2[index][4]], edges[tiling3_2[index][5]], triangles);
		CreateTriangle(edges[tiling3_2[index][6]], edges[tiling3_2[index][7]], edges[tiling3_2[index][8]], triangles);
		CreateTriangle(edges[tiling3_2[index][9]], edges[tiling3_2[index][10]], edges[tiling3_2[index][11]], triangles);
		}
		else
		{
		CreateTriangle(edges[tiling3_1[index][0]], edges[tiling3_1[index][1]], edges[tiling3_1[index][2]], triangles);
		CreateTriangle(edges[tiling3_1[index][3]], edges[tiling3_1[index][4]], edges[tiling3_1[index][5]], triangles);
		std::cout <<"case 3.1 "<< std::endl;
		}
	break;
	case 80:
		index = 10;
		if( test_face( test3[index],*p) ){
		CreateTriangle(edges[tiling3_2[index][0]], edges[tiling3_2[index][1]], edges[tiling3_2[index][2]], triangles);
		CreateTriangle(edges[tiling3_2[index][3]], edges[tiling3_2[index][4]], edges[tiling3_2[index][5]], triangles);
		CreateTriangle(edges[tiling3_2[index][6]], edges[tiling3_2[index][7]], edges[tiling3_2[index][8]], triangles);
		CreateTriangle(edges[tiling3_2[index][9]], edges[tiling3_2[index][10]], edges[tiling3_2[index][11]], triangles);
		}
		else
		{
		CreateTriangle(edges[tiling3_1[index][0]], edges[tiling3_1[index][1]], edges[tiling3_1[index][2]], triangles);
		CreateTriangle(edges[tiling3_1[index][3]], edges[tiling3_1[index][4]], edges[tiling3_1[index][5]], triangles);
		std::cout <<"case 3.1 "<< std::endl;
		}
	break;
	case 160:
		index = 11;
		if( test_face( test3[index],*p) ){
		CreateTriangle(edges[tiling3_2[index][0]], edges[tiling3_2[index][1]], edges[tiling3_2[index][2]], triangles);
		CreateTriangle(edges[tiling3_2[index][3]], edges[tiling3_2[index][4]], edges[tiling3_2[index][5]], triangles);
		CreateTriangle(edges[tiling3_2[index][6]], edges[tiling3_2[index][7]], edges[tiling3_2[index][8]], triangles);
		CreateTriangle(edges[tiling3_2[index][9]], edges[tiling3_2[index][10]], edges[tiling3_2[index][11]], triangles);
		}
		else
		{
		CreateTriangle(edges[tiling3_1[index][0]], edges[tiling3_1[index][1]], edges[tiling3_1[index][2]], triangles);
		CreateTriangle(edges[tiling3_1[index][3]], edges[tiling3_1[index][4]], edges[tiling3_1[index][5]], triangles);
		std::cout <<"case 3.1 "<< std::endl;
		}
	break;
*/
	//Triangulation Case 5 section A
    case 0x07:
	CreateTriangle(edges[11], edges[9], edges[8], triangles);
	CreateTriangle(edges[11], edges[8], edges[3], triangles);
	CreateTriangle(edges[11], edges[3], edges[2], triangles);
	break;
    case 0x0E:
	CreateTriangle(edges[10], edges[11], edges[9], triangles);
	CreateTriangle(edges[10], edges[9], edges[0], triangles);
	CreateTriangle(edges[10], edges[0], edges[3], triangles);
	break;
    case 0x0D:
	CreateTriangle(edges[8], edges[10], edges[11], triangles);
	CreateTriangle(edges[8], edges[11], edges[1], triangles);
	CreateTriangle(edges[8], edges[1], edges[0], triangles);
	break;
    case 0x0B:
	CreateTriangle(edges[9], edges[8], edges[10], triangles);
	CreateTriangle(edges[9], edges[10], edges[2], triangles);
	CreateTriangle(edges[9], edges[2], edges[1], triangles);
	break;

	//Triangulation Case 5 section B
    case 0x70:
	CreateTriangle(edges[11], edges[6], edges[7], triangles);
	CreateTriangle(edges[11], edges[7], edges[8], triangles);
	CreateTriangle(edges[11], edges[8], edges[9], triangles);
	break;
    case 0xE0:
	CreateTriangle(edges[9], edges[11], edges[10], triangles);
	CreateTriangle(edges[9], edges[10], edges[7], triangles);
	CreateTriangle(edges[9], edges[7], edges[4], triangles);
	break;
    case 0xD0:
	CreateTriangle(edges[11], edges[10], edges[8], triangles);
	CreateTriangle(edges[11], edges[8], edges[4], triangles);
	CreateTriangle(edges[11], edges[4], edges[5], triangles);
	break;
    case 0xB0:
	CreateTriangle(edges[10], edges[8], edges[9], triangles);
	CreateTriangle(edges[10], edges[9], edges[5], triangles);
	CreateTriangle(edges[10], edges[5], edges[6], triangles);
	break;

	//Triangulation Case 5 section C
    case 0x46:
	CreateTriangle(edges[0], edges[2], edges[6], triangles);
	CreateTriangle(edges[0], edges[6], edges[5], triangles);
	CreateTriangle(edges[0], edges[5], edges[9], triangles);
	break;
    case 0x64:
	CreateTriangle(edges[2], edges[6], edges[4], triangles);
	CreateTriangle(edges[2], edges[4], edges[9], triangles);
	CreateTriangle(edges[2], edges[9], edges[1], triangles);
	break;
    case 0x62:
	CreateTriangle(edges[6], edges[4], edges[0], triangles);
	CreateTriangle(edges[6], edges[0], edges[1], triangles);
	CreateTriangle(edges[6], edges[1], edges[11], triangles);
	break;
    case 0x26:
	CreateTriangle(edges[4], edges[0], edges[2], triangles);
	CreateTriangle(edges[4], edges[2], edges[11], triangles);
	CreateTriangle(edges[4], edges[11], edges[5], triangles);
	break;

	//Triangulation Case 5 section D
    case 0x91:
	CreateTriangle(edges[0], edges[4], edges[6], triangles);
	CreateTriangle(edges[0], edges[6], edges[10], triangles);
	CreateTriangle(edges[0], edges[10], edges[3], triangles);
	break;
    case 0x98:
	CreateTriangle(edges[4], edges[6], edges[2], triangles);
	CreateTriangle(edges[4], edges[2], edges[3], triangles);
	CreateTriangle(edges[4], edges[3], edges[8], triangles);
	break;
    case 0x89:
	CreateTriangle(edges[6], edges[2], edges[0], triangles);
	CreateTriangle(edges[6], edges[0], edges[8], triangles);
	CreateTriangle(edges[6], edges[8], edges[7], triangles);
	break;
    case 0x19:
	CreateTriangle(edges[2], edges[0], edges[4], triangles);
	CreateTriangle(edges[2], edges[4], edges[7], triangles);
	CreateTriangle(edges[2], edges[7], edges[10], triangles);
	break;

	//Triangulation Case 5 section E
    case 0x23:
	CreateTriangle(edges[3], edges[1], edges[5], triangles);
	CreateTriangle(edges[3], edges[5], edges[4], triangles);
	CreateTriangle(edges[3], edges[4], edges[8], triangles);
	break;
    case 0x32:
	CreateTriangle(edges[1], edges[5], edges[7], triangles);
	CreateTriangle(edges[1], edges[7], edges[8], triangles);
	CreateTriangle(edges[1], edges[8], edges[0], triangles);
	break;
    case 0x31:
	CreateTriangle(edges[5], edges[7], edges[3], triangles);
	CreateTriangle(edges[5], edges[3], edges[0], triangles);
	CreateTriangle(edges[5], edges[0], edges[9], triangles);
	break;
    case 0x13:
	CreateTriangle(edges[7], edges[3], edges[1], triangles);
	CreateTriangle(edges[7], edges[1], edges[9], triangles);
	CreateTriangle(edges[7], edges[9], edges[4], triangles);
	break;

	//Triangulation Case 5 section F
    case 0xC4:
	CreateTriangle(edges[7], edges[5], edges[1], triangles);
	CreateTriangle(edges[7], edges[1], edges[2], triangles);
	CreateTriangle(edges[7], edges[2], edges[10], triangles);
	break;
    case 0xC8:
	CreateTriangle(edges[3], edges[7], edges[5], triangles);
	CreateTriangle(edges[3], edges[5], edges[11], triangles);
	CreateTriangle(edges[3], edges[11], edges[2], triangles);
	break;
    case 0x8C:
	CreateTriangle(edges[1], edges[3], edges[7], triangles);
	CreateTriangle(edges[1], edges[7], edges[6], triangles);
	CreateTriangle(edges[1], edges[6], edges[11], triangles);
	break;
    case 0x4C:
	CreateTriangle(edges[5], edges[1], edges[3], triangles);
	CreateTriangle(edges[5], edges[3], edges[10], triangles);
	CreateTriangle(edges[5], edges[10], edges[6], triangles);
	break;

	//Triangulation Case 6
    case 0x43:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT7,start, triangles);
	break;
    case 0x83:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT8,start, triangles);
	break;
    case 0x16:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT5,start, triangles);
	break;
    case 0x86:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT8,start, triangles);
	break;
    case 0x1C:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT5,start, triangles);
	break;
    case 0x2C:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT6,start, triangles);
	break;
    case 0x29:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT6,start, triangles);
	break;
    case 0x49:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT7,start, triangles);
	break;
    case 0x34:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT3,start, triangles);
	break;
    case 0x38:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT4, start,triangles);
	break;
    case 0x61:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT1, start,triangles);
	break;
    case 0x68:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT4,start, triangles);
	break;
    case 0xC1:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT1, start,triangles);
	break;
    case 0xC2:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT2,start, triangles);
	break;
    case 0x92:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT2, start,triangles);
	break;
    case 0x94:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT3,start, triangles);
	break;
    case 0x15:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT3, start,triangles);
	break;
    case 0x51:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT7, start,triangles);
	break;
    case 0x2A:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT4, start,triangles);
	break;
    case 0xA2:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT8, start,triangles);
	break;
    case 0x45:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT1, start,triangles);
	break;
    case 0x54:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT5, start,triangles);
	break;
    case 0x8A:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT2, start,triangles);
	break;
    case 0xA8:
	TrianglesAlongEdge(type, triangles);
	TrianglesInCorners(BIT6, start,triangles);
	break;

    //Triangulation Case 8:
    case 0x0F:
	CreateTriangle(edges[8], edges[10], edges[11], triangles);
	CreateTriangle(edges[8], edges[11], edges[9], triangles);
	break;
    case 0xF0:
	CreateTriangle(edges[11], edges[10], edges[8], triangles);
	CreateTriangle(edges[11], edges[8], edges[9], triangles);
	break;
    case 0x66:
	CreateTriangle(edges[2], edges[6], edges[4], triangles);
	CreateTriangle(edges[2], edges[4], edges[0], triangles);
	break;
    case 0x99:
	CreateTriangle(edges[4], edges[6], edges[2], triangles);
	CreateTriangle(edges[4], edges[2], edges[0], triangles);
	break;
    case 0x33:
	CreateTriangle(edges[3], edges[1], edges[5], triangles);
	CreateTriangle(edges[3], edges[5], edges[7], triangles);
	break;
    case 0xCC:
	CreateTriangle(edges[5], edges[1], edges[3], triangles);
	CreateTriangle(edges[5], edges[3], edges[7], triangles);
	break;

	//Triangulation Case 9 part A:
    case 0x1B:
	CreateTriangle(edges[10], edges[2], edges[1], triangles);
	CreateTriangle(edges[10], edges[1], edges[9], triangles);
	CreateTriangle(edges[10], edges[9], edges[7], triangles);
	CreateTriangle(edges[7], edges[9], edges[4], triangles);
	break;
    case 0x27:
	CreateTriangle(edges[4], edges[8], edges[3], triangles);
	CreateTriangle(edges[4], edges[3], edges[2], triangles);
	CreateTriangle(edges[4], edges[2], edges[5], triangles);
	CreateTriangle(edges[5], edges[2], edges[11], triangles);
	break;
    case 0x4E:
	CreateTriangle(edges[10], edges[6], edges[5], triangles);
	CreateTriangle(edges[10], edges[5], edges[9], triangles);
	CreateTriangle(edges[10], edges[9], edges[3], triangles);
	CreateTriangle(edges[3], edges[9], edges[0], triangles);
	break;
    case 0x8D:
	CreateTriangle(edges[0], edges[8], edges[7], triangles);
	CreateTriangle(edges[0], edges[7], edges[6], triangles);
	CreateTriangle(edges[0], edges[6], edges[1], triangles);
	CreateTriangle(edges[1], edges[6], edges[11], triangles);
	break;

	//Triangulation Case 9 part B:
    case 0xB1:
	CreateTriangle(edges[0], edges[9], edges[5], triangles);
	CreateTriangle(edges[0], edges[5], edges[6], triangles);
	CreateTriangle(edges[0], edges[6], edges[3], triangles);
	CreateTriangle(edges[3], edges[6], edges[10], triangles);
	break;
    case 0x72:
	CreateTriangle(edges[8], edges[0], edges[1], triangles);
	CreateTriangle(edges[8], edges[1], edges[11], triangles);
	CreateTriangle(edges[8], edges[11], edges[7], triangles);
	CreateTriangle(edges[7], edges[11], edges[6], triangles);
	break;
    case 0xE4:
	CreateTriangle(edges[2], edges[10], edges[7], triangles);
	CreateTriangle(edges[2], edges[7], edges[4], triangles);
	CreateTriangle(edges[2], edges[4], edges[1], triangles);
	CreateTriangle(edges[1], edges[4], edges[9], triangles);
	break;
    case 0xD8:
	CreateTriangle(edges[8], edges[4], edges[5], triangles);
	CreateTriangle(edges[8], edges[5], edges[11], triangles);
	CreateTriangle(edges[8], edges[11], edges[3], triangles);
	CreateTriangle(edges[3], edges[11], edges[2], triangles);
	break;
	//Triangulation Case 10
    case 0x69:
    case 0x96:
    case 0xC3:
    case 0x3C:
    case 0x55:
    case 0xAA:
	TrianglesAlongEdge(type, triangles);
	break;
	//Triangulation Case 11 and 14
    case 0x17:
	CreateTriangle(edges[9], edges[4], edges[11], triangles);
	CreateTriangle(edges[7], edges[2], edges[11], triangles);
	CreateTriangle(edges[11], edges[4], edges[7], triangles);
	CreateTriangle(edges[7], edges[3], edges[2], triangles);
	break;
    case 0x1D:
	CreateTriangle(edges[1], edges[0], edges[4], triangles);
	CreateTriangle(edges[10], edges[11], edges[7], triangles);
	CreateTriangle(edges[4], edges[7], edges[11], triangles);
	CreateTriangle(edges[4], edges[11], edges[1], triangles);
	break;
    case 0x2B:
	CreateTriangle(edges[1], edges[5], edges[2], triangles);
	CreateTriangle(edges[4], edges[8], edges[10], triangles);
	CreateTriangle(edges[10], edges[5], edges[4], triangles);
	CreateTriangle(edges[10], edges[2], edges[5], triangles);
	break;
    case 0x2E:
	CreateTriangle(edges[0], edges[3], edges[4], triangles);
	CreateTriangle(edges[10], edges[4], edges[3], triangles);
	CreateTriangle(edges[10], edges[5], edges[4], triangles);
	CreateTriangle(edges[10], edges[11], edges[5], triangles);
	break;
    case 0x36:
	CreateTriangle(edges[0], edges[2], edges[8], triangles);
	CreateTriangle(edges[8], edges[2], edges[7], triangles);
	CreateTriangle(edges[2], edges[11], edges[7], triangles);
	CreateTriangle(edges[7], edges[11], edges[5], triangles);
	break;
    case 0x39:
	CreateTriangle(edges[0], edges[9], edges[2], triangles);
	CreateTriangle(edges[9], edges[5], edges[2], triangles);
	CreateTriangle(edges[5], edges[7], edges[10], triangles);
	CreateTriangle(edges[5], edges[10], edges[2], triangles);
	break;
    case 0x47:
	CreateTriangle(edges[5], edges[9], edges[8], triangles);
	CreateTriangle(edges[8], edges[6], edges[5], triangles);
	CreateTriangle(edges[8], edges[3], edges[6], triangles);
	CreateTriangle(edges[3], edges[2], edges[6], triangles);
	break;
    case 0x4B:
	CreateTriangle(edges[9], edges[8], edges[10], triangles);
	CreateTriangle(edges[9], edges[10], edges[2], triangles);
	CreateTriangle(edges[9], edges[2], edges[1], triangles);
	CreateTriangle(edges[11], edges[6], edges[5], triangles);
	break;
    case 0x4D:
	CreateTriangle(edges[5], edges[1], edges[0], triangles);
	CreateTriangle(edges[10], edges[6], edges[8], triangles);
	CreateTriangle(edges[8], edges[6], edges[5], triangles);
	CreateTriangle(edges[5], edges[0], edges[8], triangles);
	break;
    case 0x63:
	CreateTriangle(edges[3], edges[1], edges[11], triangles);
	CreateTriangle(edges[6], edges[4], edges[8], triangles);
	CreateTriangle(edges[6], edges[8], edges[3], triangles);
	CreateTriangle(edges[3], edges[11], edges[6], triangles);
	break;
    case 0x6C:
	CreateTriangle(edges[1], edges[3], edges[9], triangles);
	CreateTriangle(edges[6], edges[4], edges[10], triangles);
	CreateTriangle(edges[10], edges[4], edges[3], triangles);
	CreateTriangle(edges[4], edges[9], edges[3], triangles);
	break;
    case 0x71:
	CreateTriangle(edges[6], edges[7], edges[3], triangles);
	CreateTriangle(edges[9], edges[11], edges[0], triangles);
	CreateTriangle(edges[3], edges[11], edges[6], triangles);
	CreateTriangle(edges[3], edges[0], edges[11], triangles);
	break;
    case 0x74:
	CreateTriangle(edges[2], edges[6], edges[7], triangles);
	CreateTriangle(edges[8], edges[9], edges[1], triangles);
	CreateTriangle(edges[8], edges[1], edges[2], triangles);
	CreateTriangle(edges[7], edges[8], edges[2], triangles);
	break;
    case 0x8B:
	CreateTriangle(edges[6], edges[2], edges[1], triangles);
	CreateTriangle(edges[9], edges[8], edges[7], triangles);
	CreateTriangle(edges[9], edges[7], edges[6], triangles);
	CreateTriangle(edges[9], edges[6], edges[1], triangles);
	break;
    case 0x8E:
	CreateTriangle(edges[3], edges[7], edges[0], triangles);
	CreateTriangle(edges[9], edges[6], edges[11], triangles);
	CreateTriangle(edges[9], edges[0], edges[7], triangles);
	CreateTriangle(edges[9], edges[7], edges[6], triangles);
	break;
    case 0x93:
	CreateTriangle(edges[3], edges[1], edges[10], triangles);
	CreateTriangle(edges[4], edges[6], edges[9], triangles);
	CreateTriangle(edges[1], edges[6], edges[10], triangles);
	CreateTriangle(edges[1], edges[9], edges[6], triangles);
	break;
    case 0x9C:
	CreateTriangle(edges[8], edges[1], edges[3], triangles);
	CreateTriangle(edges[4], edges[1], edges[8], triangles);
	CreateTriangle(edges[11], edges[1], edges[4], triangles);
	CreateTriangle(edges[6], edges[11], edges[4], triangles);
	break;
    case 0xB2:
	CreateTriangle(edges[8], edges[0], edges[10], triangles);
	CreateTriangle(edges[1], edges[5], edges[6], triangles);
	CreateTriangle(edges[0], edges[1], edges[10], triangles);
	CreateTriangle(edges[1], edges[6], edges[10], triangles);
	break;
    case 0xB8:
	CreateTriangle(edges[8], edges[9], edges[3], triangles);
	CreateTriangle(edges[6], edges[2], edges[5], triangles);
	CreateTriangle(edges[9], edges[2], edges[3], triangles);
	CreateTriangle(edges[9], edges[5], edges[2], triangles);
	break;
    case 0xC6:
	CreateTriangle(edges[0], edges[2], edges[10], triangles);
	CreateTriangle(edges[7], edges[5], edges[9], triangles);
	CreateTriangle(edges[0], edges[10], edges[7], triangles);
	CreateTriangle(edges[0], edges[7], edges[9], triangles);
	break;
    case 0xC9:
	CreateTriangle(edges[0], edges[11], edges[2], triangles);
	CreateTriangle(edges[7], edges[5], edges[8], triangles);
	CreateTriangle(edges[0], edges[8], edges[5], triangles);
	CreateTriangle(edges[0], edges[5], edges[11], triangles);
	break;
    case 0xD1:
	CreateTriangle(edges[11], edges[10], edges[3], triangles);
	CreateTriangle(edges[0], edges[4], edges[5], triangles);
	CreateTriangle(edges[3], edges[0], edges[11], triangles);
	CreateTriangle(edges[0], edges[5], edges[11], triangles);
	break;
    case 0xD4:
	CreateTriangle(edges[8], edges[2], edges[10], triangles);
	CreateTriangle(edges[1], edges[4], edges[5], triangles);
	CreateTriangle(edges[8], edges[1], edges[2], triangles);
	CreateTriangle(edges[8], edges[4], edges[1], triangles);
	break;
  case 0xE2:
      CreateTriangle(edges[10], edges[1], edges[11], triangles);
      CreateTriangle(edges[0], edges[7], edges[4], triangles);
      CreateTriangle(edges[0], edges[10], edges[7], triangles);
      CreateTriangle(edges[0], edges[1], edges[10], triangles);
      break;
    case 0xE8:
	CreateTriangle(edges[4], edges[3], edges[7], triangles);
	CreateTriangle(edges[9], edges[11], edges[2], triangles);
	CreateTriangle(edges[2], edges[3], edges[9], triangles);
	CreateTriangle(edges[3], edges[4], edges[9], triangles);
	break;
	//Triangulation Case 12
    case 0x1E:
	CreateTriangle(edges[9], edges[10], edges[11], triangles);
	CreateTriangle(edges[9], edges[3], edges[10], triangles);
	CreateTriangle(edges[9], edges[0], edges[3], triangles);
	CreateTriangle(edges[4], edges[7], edges[8], triangles);
	break;
    case 0x2D:
	CreateTriangle(edges[11], edges[8], edges[10], triangles);
	CreateTriangle(edges[11], edges[0], edges[8], triangles);
	CreateTriangle(edges[11], edges[1], edges[0], triangles);
	CreateTriangle(edges[5], edges[4], edges[9], triangles);
	break;
    case 0x35:
	CreateTriangle(edges[5], edges[7], edges[3], triangles);
	CreateTriangle(edges[5], edges[3], edges[0], triangles);
	CreateTriangle(edges[5], edges[0], edges[9], triangles);
	CreateTriangle(edges[2], edges[11], edges[1], triangles);
	break;
    case 0x3A:
	CreateTriangle(edges[1], edges[5], edges[7], triangles);
	CreateTriangle(edges[1], edges[7], edges[8], triangles);
	CreateTriangle(edges[1], edges[8], edges[0], triangles);
	CreateTriangle(edges[10], edges[2], edges[3], triangles);
	break;
    case 0x53:
	CreateTriangle(edges[7], edges[3], edges[1], triangles);
	CreateTriangle(edges[7], edges[1], edges[9], triangles);
	CreateTriangle(edges[7], edges[9], edges[4], triangles);
	CreateTriangle(edges[11], edges[6], edges[5], triangles);
	break;
    case 0x56:
	CreateTriangle(edges[0], edges[2], edges[6], triangles);
	CreateTriangle(edges[0], edges[6], edges[5], triangles);
	CreateTriangle(edges[0], edges[5], edges[9], triangles);
	CreateTriangle(edges[4], edges[7], edges[8], triangles);
	break;
    case 0x59:
	CreateTriangle(edges[2], edges[0], edges[4], triangles);
	CreateTriangle(edges[2], edges[4], edges[7], triangles);
	CreateTriangle(edges[2], edges[7], edges[10], triangles);
	CreateTriangle(edges[11], edges[6], edges[5], triangles);
	break;
    case 0x5C:
	CreateTriangle(edges[5], edges[1], edges[3], triangles);
	CreateTriangle(edges[5], edges[3], edges[10], triangles);
	CreateTriangle(edges[5], edges[10], edges[6], triangles);
	CreateTriangle(edges[4], edges[7], edges[8], triangles);
	break;
    case 0x65:
	CreateTriangle(edges[2], edges[6], edges[4], triangles);
	CreateTriangle(edges[2], edges[4], edges[9], triangles);
	CreateTriangle(edges[2], edges[9], edges[1], triangles);
	CreateTriangle(edges[0], edges[8], edges[3], triangles);
	break;
    case 0x6A:
	CreateTriangle(edges[6], edges[4], edges[0], triangles);
	CreateTriangle(edges[6], edges[0], edges[1], triangles);
	CreateTriangle(edges[6], edges[1], edges[11], triangles);
	CreateTriangle(edges[10], edges[2], edges[3], triangles);
	break;
    case 0x78:
	CreateTriangle(edges[8], edges[9], edges[11], triangles);
	CreateTriangle(edges[8], edges[11], edges[6], triangles);
	CreateTriangle(edges[8], edges[6], edges[7], triangles);
	CreateTriangle(edges[10], edges[2], edges[3], triangles);
	break;
    case 0x87:
	CreateTriangle(edges[11], edges[9], edges[8], triangles);
	CreateTriangle(edges[11], edges[8], edges[3], triangles);
	CreateTriangle(edges[11], edges[3], edges[2], triangles);
	CreateTriangle(edges[7], edges[6], edges[10], triangles);
	break;
    case 0x95:
	CreateTriangle(edges[0], edges[4], edges[6], triangles);
	CreateTriangle(edges[0], edges[6], edges[10], triangles);
	CreateTriangle(edges[0], edges[10], edges[3], triangles);
	CreateTriangle(edges[1], edges[2], edges[11], triangles);
	break;
    case 0x9A:
	CreateTriangle(edges[4], edges[6], edges[2], triangles);
	CreateTriangle(edges[4], edges[2], edges[3], triangles);
	CreateTriangle(edges[4], edges[3], edges[8], triangles);
	CreateTriangle(edges[1], edges[9], edges[0], triangles);
	break;
    case 0xA3:
	CreateTriangle(edges[3], edges[1], edges[5], triangles);
	CreateTriangle(edges[3], edges[5], edges[4], triangles);
	CreateTriangle(edges[3], edges[4], edges[8], triangles);
	CreateTriangle(edges[7], edges[6], edges[10], triangles);
	break;
    case 0xA6:
	CreateTriangle(edges[4], edges[0], edges[2], triangles);
	CreateTriangle(edges[4], edges[2], edges[11], triangles);
	CreateTriangle(edges[4], edges[11], edges[5], triangles);
	CreateTriangle(edges[7], edges[6], edges[10], triangles);
	break;
    case 0xA9:
	CreateTriangle(edges[6], edges[2], edges[0], triangles);
	CreateTriangle(edges[6], edges[0], edges[8], triangles);
	CreateTriangle(edges[6], edges[8], edges[7], triangles);
	CreateTriangle(edges[9], edges[5], edges[4], triangles);
	break;
    case 0xAC:
	CreateTriangle(edges[1], edges[3], edges[7], triangles);
	CreateTriangle(edges[1], edges[7], edges[6], triangles);
	CreateTriangle(edges[1], edges[6], edges[11], triangles);
	CreateTriangle(edges[5], edges[4], edges[9], triangles);
	break;
    case 0xB4:
	CreateTriangle(edges[10], edges[8], edges[9], triangles);
	CreateTriangle(edges[10], edges[9], edges[5], triangles);
	CreateTriangle(edges[10], edges[5], edges[6], triangles);
	CreateTriangle(edges[1], edges[2], edges[11], triangles);
	break;
    case 0xC5:
	CreateTriangle(edges[7], edges[5], edges[1], triangles);
	CreateTriangle(edges[7], edges[1], edges[2], triangles);
	CreateTriangle(edges[7], edges[2], edges[10], triangles);
	CreateTriangle(edges[8], edges[3], edges[0], triangles);
	break;
    case 0xCA:
	CreateTriangle(edges[3], edges[7], edges[5], triangles);
	CreateTriangle(edges[3], edges[5], edges[11], triangles);
	CreateTriangle(edges[3], edges[11], edges[2], triangles);
	CreateTriangle(edges[0], edges[1], edges[9], triangles);
	break;
    case 0xD2:
	CreateTriangle(edges[11], edges[10], edges[8], triangles);
	CreateTriangle(edges[11], edges[8], edges[4], triangles);
	CreateTriangle(edges[11], edges[4], edges[5], triangles);
	CreateTriangle(edges[0], edges[1], edges[9], triangles);
	break;
    case 0xE1:
	CreateTriangle(edges[9], edges[11], edges[10], triangles);
	CreateTriangle(edges[9], edges[10], edges[7], triangles);
	CreateTriangle(edges[9], edges[7], edges[4], triangles);
	CreateTriangle(edges[0], edges[8], edges[3], triangles);
	break;
    default:
	TrianglesInCorners(type,start,triangles);
	break;
    } //End switch (type)

    if (flipNormals) {
	for (unsigned int i = triangleCount; i<triangles.size(); i++) {
		//triangles.add(TriangleN1<float>(p1,p2,p3,normal));
		float v0[] = {triangles[i].getVertex(0)(0), triangles[i].getVertex(0)(1), triangles[i].getVertex(0)(2)};
	    float v2[] = {triangles[i].getVertex(2)(0), triangles[i].getVertex(2)(1), triangles[i].getVertex(2)(2)};
		float normal[] = {triangles[i].getFaceNormal()(0), triangles[i].getFaceNormal()(1), triangles[i].getFaceNormal()(2)};

	    triangles[i][0](0) = v2[0];
		triangles[i][0](1) = v2[1];
		triangles[i][0](2) = v2[2];

		triangles[i][2](0) = v0[0];
		triangles[i][2](1) = v0[1];
		triangles[i][2](2) = v0[2];

	    triangles[i][3](0) = -normal[0];
	    triangles[i][3](1)  = -normal[1];
	    triangles[i][3](2)  = -normal[2];
	}
    }


}



void MarchingCube::TrianglesInCorners(unsigned char type,OcVoxel* start, rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<float> >& triangles) {

	if (type & BIT1)
	CreateTriangle(edges[0], edges[8], edges[3], triangles);
    if (type & BIT2)
	CreateTriangle(edges[0], edges[1], edges[9], triangles);
    if (type & BIT3)
	CreateTriangle(edges[1], edges[2], edges[11], triangles);
    if (type & BIT4)
	CreateTriangle(edges[2], edges[3], edges[10], triangles);

    if (type & BIT5)
	CreateTriangle(edges[4], edges[7], edges[8], triangles);
    if (type & BIT6)
	CreateTriangle(edges[4], edges[9], edges[5], triangles);
    if (type & BIT7)
	CreateTriangle(edges[5], edges[11], edges[6], triangles);
    if (type & BIT8)
	CreateTriangle(edges[6], edges[10], edges[7], triangles);

	/*
	if(type & BIT1 && type & BIT3)
	{
		CreateTriangle(edges[0], edges[8], edges[3], triangles);
		CreateTriangle(edges[1], edges[2], edges[11], triangles);

		CreateTriangle(edges[1], edges[2], edges[0], triangles);
		CreateTriangle(edges[0], edges[2], edges[3], triangles);
		
		std::cout << "voxel 1 3 " << std::endl;
		for(int i = 0;i < 8; i++){ 
			std::cout << start->getVertexISOValue(i) << " ";
		}
		std::cout << std::endl << "type "<< type << std::endl;
	}
	if(type & BIT2 && type & BIT4)
	{
	CreateTriangle(edges[0], edges[1], edges[9], triangles);
	CreateTriangle(edges[2], edges[3], edges[10], triangles);
		std::cout << "voxel 2 4 " << std::endl;
		for(int i = 0;i < 8; i++){ 
			std::cout << start->getVertexISOValue(i) << " ";
		}
		std::cout << std::endl << "type "<< std::hex << (unsigned int)type << std::endl;
	}
	*/
}

void MarchingCube::TrianglesAlongEdge(unsigned char type, rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<float> >& triangles) {

	if (type & BIT1 && type & BIT2) {
	CreateTriangle(edges[3], edges[1], edges[8], triangles);
	CreateTriangle(edges[8], edges[1], edges[9], triangles);
    }
    if (type & BIT2 && type & BIT3) {
	CreateTriangle(edges[0], edges[2], edges[9], triangles);
	CreateTriangle(edges[9], edges[2], edges[11], triangles);
    }
    if (type & BIT3 && type & BIT4) {
	CreateTriangle(edges[1], edges[3], edges[11], triangles);
	CreateTriangle(edges[11], edges[3], edges[10], triangles);
    }
    if (type & BIT4 && type & BIT1) {
	CreateTriangle(edges[2], edges[0], edges[10], triangles);
	CreateTriangle(edges[10], edges[0], edges[8], triangles);
    }


    if (type & BIT5 && type & BIT6) {
	CreateTriangle(edges[5], edges[7], edges[9], triangles);
	CreateTriangle(edges[9], edges[7], edges[8], triangles);
    }
    if (type & BIT6 && type & BIT7) {
	CreateTriangle(edges[6], edges[4], edges[11], triangles);
	CreateTriangle(edges[11], edges[4], edges[9], triangles);
    }
    if (type & BIT7 && type & BIT8) {
	CreateTriangle(edges[7], edges[5], edges[10], triangles);
	CreateTriangle(edges[10], edges[5], edges[11], triangles);
    }
    if (type & BIT8 && type & BIT5) {
	CreateTriangle(edges[4], edges[6], edges[8], triangles);
	CreateTriangle(edges[8], edges[6], edges[10], triangles);
    }


    if (type & BIT1 && type & BIT5) {
	CreateTriangle(edges[0], edges[4], edges[3], triangles);
	CreateTriangle(edges[3], edges[4], edges[7], triangles);
    }
    if (type & BIT2 && type & BIT6) {
	CreateTriangle(edges[4], edges[0], edges[5], triangles);
	CreateTriangle(edges[5], edges[0], edges[1], triangles);
    }
    if (type & BIT3 && type & BIT7) {
	CreateTriangle(edges[2], edges[6], edges[1], triangles);
	CreateTriangle(edges[1], edges[6], edges[5], triangles);
    }
    if (type & BIT4 && type & BIT8) {
	CreateTriangle(edges[6], edges[2], edges[7], triangles);
	CreateTriangle(edges[7], edges[2], edges[3], triangles);
    }

}


void MarchingCube::TrianglesInCornersRoot(unsigned char type, rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<float> >& triangles) {
	if (type & BIT1){
		CreateTriangle(edges[8],(edges[8]+ edges[10])/2, (edges[9]+edges[11])/2, triangles);
	//	    	CreateTriangle(edges[8], (edges[9]+edges[11])/2, edges[9], triangles);
		    	CreateTriangle(edges[1], (edges[3]+edges[7])/2, edges[3], triangles);
	//	    	CreateTriangle(edges[1], (edges[1]+edges[5])/2, (edges[3]+edges[7])/2, triangles);
	}
		    	if (type & BIT2)
	CreateTriangle(edges[0], edges[1], edges[9], triangles);
    if (type & BIT3)
	CreateTriangle(edges[1], edges[2], edges[11], triangles);
    if (type & BIT4)
	CreateTriangle(edges[2], edges[3], edges[10], triangles);

    if (type & BIT5)
	CreateTriangle(edges[4], edges[7], edges[8], triangles);
    if (type & BIT6)
	CreateTriangle(edges[4], edges[9], edges[5], triangles);
    if (type & BIT7)
	CreateTriangle(edges[5], edges[11], edges[6], triangles);
    if (type & BIT8)
	CreateTriangle(edges[6], edges[10], edges[7], triangles);
}

void MarchingCube::TrianglesAlongEdgeRoot(unsigned char type, rw::geometry::PlainTriMesh<rw::geometry::TriangleN1<float> >& triangles) {
 //y max z max
	if (type & BIT1 && type & BIT2) {
    	CreateTriangle(edges[8],(edges[8]+ edges[10])/2, (edges[9]+edges[11])/2, triangles);
    	CreateTriangle(edges[8], (edges[9]+edges[11])/2, edges[9], triangles);
    	CreateTriangle(edges[1], (edges[3]+edges[7])/2, edges[3], triangles);
    	CreateTriangle(edges[1], (edges[1]+edges[5])/2, (edges[3]+edges[7])/2, triangles);
    }
//x min z max
    if (type & BIT2 && type & BIT3) {
    	CreateTriangle(edges[0], edges[2], (edges[2]+edges[6])/2, triangles);
    	CreateTriangle((edges[2]+edges[6])/2,(edges[0]+edges[4])/2,edges[0] , triangles);
    	CreateTriangle(edges[11], edges[9], (edges[0]+edges[4])/2, triangles);
		CreateTriangle((edges[0]+edges[4])/2,(edges[2]+edges[6])/2,edges[11] , triangles);
    }

//y min z max
    if (type & BIT3 && type & BIT4) {
		CreateTriangle(edges[10], edges[11], (edges[1]+ edges[5]) / 2, triangles);
		CreateTriangle(edges[10], (edges[1] + edges[5]) / 2, (edges[3] + edges[7]) / 2,triangles);
		CreateTriangle(edges[1], edges[3],(edges[3] + edges[7]) / 2, triangles);
		CreateTriangle(edges[1], (edges[3] + edges[7]) / 2, (edges[1]+ edges[5]) / 2, triangles);
	}
//x max z max
    if (type & BIT4 && type & BIT1) {
    	CreateTriangle(edges[0], (edges[2]+edges[6])/2, edges[2], triangles);
    	CreateTriangle((edges[2]+edges[6])/2,edges[0],(edges[0]+edges[4])/2, triangles);
    	CreateTriangle(edges[8], edges[10], (edges[2]+edges[6])/2, triangles);
    	CreateTriangle((edges[2]+edges[6])/2,(edges[0]+edges[4])/2,edges[8] , triangles);
	}
//y max z min
    if (type & BIT5 && type & BIT6) {
    	CreateTriangle((edges[1]+edges[5])/2, (edges[3]+edges[7])/2, edges[9], triangles);
    	CreateTriangle(edges[9], (edges[3]+edges[7])/2, edges[8], triangles);
    	CreateTriangle( edges[7],(edges[3]+edges[7])/2,edges[5], triangles);
		CreateTriangle(edges[5], (edges[3]+edges[7])/2,(edges[1]+edges[5])/2, triangles);
    }
//x min z min
    if (type & BIT6 && type & BIT7) {
		CreateTriangle(edges[11], (edges[0]+ edges[4]) / 2, edges[9] , triangles);
		CreateTriangle(edges[11], (edges[2] + edges[6]) / 2, (edges[0] + edges[4]) / 2,triangles);
		CreateTriangle(edges[6], edges[4],(edges[2] + edges[6]) / 2, triangles);
		CreateTriangle(edges[4], (edges[0] + edges[4]) / 2, (edges[2]+ edges[6]) / 2, triangles);
    }
//y min z min
    if (type & BIT7 && type & BIT8) {
    	CreateTriangle((edges[3]+edges[7])/2, edges[5], edges[10], triangles);
    	CreateTriangle(edges[10], (edges[5]+edges[1])/2, edges[11], triangles);
    	CreateTriangle(edges[7], edges[5], (edges[1]+edges[5])/2, triangles);
    	CreateTriangle(edges[7], (edges[5]+edges[1])/2, (edges[3]+edges[7])/2, triangles);
    }
//x max z min
    if (type & BIT8 && type & BIT5) {
    	CreateTriangle((edges[4]+edges[0])/2, (edges[6]+edges[2])/2, edges[8], triangles);
    	CreateTriangle(edges[8], (edges[6]+edges[2])/2, edges[10], triangles);
    	CreateTriangle((edges[6]+edges[2])/2,(edges[4]+edges[0])/2,edges[4], triangles);
    	CreateTriangle(edges[4], edges[6],(edges[2]+edges[6])/2, triangles);
    }
//x max y max
    if (type & BIT1 && type & BIT5) {
	CreateTriangle(edges[0], edges[4], (edges[3]+edges[1])/2, triangles);
	CreateTriangle((edges[3]+edges[1])/2, edges[4], (edges[7]+edges[5])/2, triangles);
	CreateTriangle(edges[3], (edges[5]+edges[7])/2,edges[7], triangles);
	CreateTriangle((edges[3]+edges[1])/2, (edges[7]+edges[5])/2, edges[3], triangles);
    }
//x min y max
    if (type & BIT2 && type & BIT6) {
	CreateTriangle(edges[4], edges[0], (edges[5]+edges[7])/2, triangles);
	CreateTriangle((edges[5]+edges[7])/2, edges[0], (edges[1]+edges[3])/2, triangles);
	CreateTriangle(edges[1], edges[5], (edges[5]+edges[7])/2, triangles);
	CreateTriangle((edges[5]+edges[7])/2, (edges[1]+edges[3])/2, edges[1], triangles);
    }

// x min y min
    if (type & BIT3 && type & BIT7) {
	CreateTriangle(edges[2], edges[6], (edges[1]+edges[3])/2, triangles);
	CreateTriangle((edges[1]+edges[3])/2, edges[6], (edges[5]+edges[7])/2, triangles);
	CreateTriangle(edges[5], edges[1], (edges[1]+edges[3])/2, triangles);
	CreateTriangle((edges[1]+edges[3])/2, (edges[5]+edges[7])/2,edges[5], triangles);
    }

//x max y min
    if (type & BIT4 && type & BIT8) {
	CreateTriangle(edges[6], edges[2], (edges[7]+edges[5])/2, triangles);
	CreateTriangle((edges[1]+edges[3])/2, (edges[7]+edges[5])/2,edges[2], triangles);
	CreateTriangle(edges[3], edges[7], (edges[7]+edges[5])/2, triangles);
		CreateTriangle((edges[1]+edges[3])/2, edges[3], (edges[7]+edges[5])/2, triangles);
    }

}


