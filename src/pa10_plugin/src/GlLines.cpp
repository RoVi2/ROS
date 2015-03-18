#include "GlLines.hpp"

#include <rw/math/LinearAlgebra.hpp>
#include <rw/math/Quaternion.hpp>
#include <rw/math/VelocityScrew6D.hpp>
#include <rw/math/Constants.hpp>
#include <rw/math/RPY.hpp>

#include <rw/math/Math.hpp>

#include <cmath>
#include <float.h>

using namespace rw::math;
using namespace rw::geometry;

using namespace boost::numeric::ublas;


GlLines::GlLines(const Transform3D<>& transform) {

	std::cout <<"Transform " << transform << std::endl;
	_currentTrans = cast<float>(transform);
}


GlLines::~GlLines() {
}



void GlLines::update(
        std::vector<std::pair<rw::math::Transform3D<float>,rw::math::Transform3D<float> > > &points,
        std::vector<std::pair<rw::math::Transform3D<float>,rw::math::Transform3D<float> > > &pointsAir,
	int index) {	
	_points.clear();	
	_pointsAir.clear();
   _points = points;
	_pointsAir = pointsAir;

}

void GlLines::update(
        std::vector<std::pair<rw::math::Transform3D<float>,rw::math::Transform3D<float> > > &points,
	int index) {
   _points.clear();	
	_points = points;
	_pointsAir.clear();
}

void GlLines::reset()
{
	_points.clear();
	_pointsAir.clear();
}




void GlLines::draw(const rw::graphics::DrawableNode::RenderInfo& info, rw::graphics::DrawableNode::DrawType type, double alpha) const{
//void OcTree::draw(DrawType type, double alpha) const{

    
    glBegin(GL_LINES);
    glLineWidth(100);
    glColor4f(0.0f, 1.0f, 0.0f, 1.0f);

    Vector3D<float> vertex1;
    Vector3D<float> vertex2;

    for (unsigned int i = 0; i < _points.size(); i++) {
	vertex1 = (_currentTrans*_points[i].first).P();
	vertex2 = (_currentTrans*_points[i].second).P();
	glVertex3f(vertex1(0), vertex1(1), vertex1(2));
	glVertex3f(vertex2(0), vertex2(1), vertex2(2));		
    }
    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
    for (unsigned int i = 0; i < _pointsAir.size(); i++) {
	vertex1 = (_currentTrans*_pointsAir[i].first).P();
	vertex2 = (_currentTrans*_pointsAir[i].second).P();
	glVertex3f(vertex1(0), vertex1(1), vertex1(2));
	glVertex3f(vertex2(0), vertex2(1), vertex2(2));		
    }
    glEnd();
}

