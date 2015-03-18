/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute, 
 * Faculty of Engineering, University of Southern Denmark 
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef GlLines_HPP
#define GlLines_HPP

#include <rw/geometry/PlainTriMesh.hpp>
#include <rw/math/Vector3D.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/sensor/Image.hpp>
#include <vector>
#include <rw/graphics/Render.hpp>
#include <rw/graphics/DrawableNode.hpp>
#include <utility>

class GlLines: public rw::graphics::Render {
public:

	GlLines(const rw::math::Transform3D<double>& transform);

	~GlLines();

	virtual void draw(const rw::graphics::DrawableNode::RenderInfo& info, rw::graphics::DrawableNode::DrawType type, double alpha) const;
 
    void update(
	std::vector<std::pair<rw::math::Transform3D<float>,rw::math::Transform3D<float> > > &points,
        std::vector<std::pair<rw::math::Transform3D<float>,rw::math::Transform3D<float> > > &pointsAir,
        int index);
    
void update(
	std::vector<std::pair<rw::math::Transform3D<float>,rw::math::Transform3D<float> > > &points,
        int index);

    void reset();


private:
	rw::math::Transform3D<float> _currentTrans;

        std::vector<std::pair<rw::math::Transform3D<float>,rw::math::Transform3D<float> > > _points;
        std::vector<std::pair<rw::math::Transform3D<float>,rw::math::Transform3D<float> > > _pointsAir;

};

#endif //GLLines_HPP
