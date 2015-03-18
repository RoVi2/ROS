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

#ifndef SamplePLUGIN_HPP
#define SamplePLUGIN_HPP


//#define NO_ROS 1
#include <rws/RobWorkStudioPlugin.hpp>

#include "SampleWidget.hpp"

class SamplePlugin: public rws::RobWorkStudioPlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )

public:
    SamplePlugin();
    virtual ~SamplePlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

     virtual void initialize();

private slots:
    void stateChangedListener(const rw::kinematics::State &state);

private:
    SampleWidget* _comWidget;

    void genericEvent(const std::string& str);
    bool _rosInitialized;
#ifndef NO_ROS
    ros::NodeHandle _nodeHandle;
#endif


};


#endif // SamplePLUGIN_HPP
