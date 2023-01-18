/*
 *    Copyright (C) 2022 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"
#include <string>
#include "topics.h"
// Gazebo
#include <gz/msgs.hh>
#include <gz/transport.hh>

using namespace std;
using namespace gz;

gz::transport::Node SpecificWorker::node;

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
 	QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	return true;
}

#pragma endregion Gazebo_CallbackFunctions

void SpecificWorker::initialize(int period)
{
    #pragma region Robocomp

	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
	}

    #pragma endregion Robocomp

    // Subscribe to depth_camera topic by registering a callback
    if (!node.Subscribe(ROBOCOMP_DEPTHCAMERA, &SpecificWorker::depth_camera_cb, this))
        cerr << "Error subscribing to topic [" << ROBOCOMP_DEPTHCAMERA << "]" << std::endl;
    else
        cout << "SpecificWorker suscribed to [" << ROBOCOMP_DEPTHCAMERA << "]" << std::endl;
}

void SpecificWorker::compute()
{
    // Doing nothing with the compute
}

///////////////////////////////////////////////////////////////////////////////////
#pragma region Gazebo_CallbackFunctions

/**
 * @brief Subscription callback for the LIDAR sensor in Gazebo.
 *
 * @param[in] _msg Data structure containing information about the LIDAR.
 */
void SpecificWorker::lidar_cb(const gz::msgs::LaserScan &_msg)
{
    gz::msgs::Twist dataMsg;

    bool allMore = true;
    for (int i = 0; i < _msg.ranges_size(); i++)
    {
        if (_msg.ranges(i) < 1.0)
        {
            allMore = false;
            break;
        }
    }
    if (allMore) // if all bigger than one
    {
        dataMsg.mutable_linear()->set_x(0.5);
        dataMsg.mutable_angular()->set_z(0.0);
    }
    else
    {
        dataMsg.mutable_linear()->set_x(0.0);
        dataMsg.mutable_angular()->set_z(0.5);
    }

    gz::transport::Node::Publisher pub = SpecificWorker::node.Advertise<gz::msgs::Twist>("/cmd_vel");
    pub.Publish(dataMsg);
}

/**
 * @brief Subscription callback for the depth camera sensor in Gazebo.
 *
 * @param[in] _msg Data structure containing information about the depth image.
 */
void SpecificWorker::depth_camera_cb(const gz::msgs::Image &_msg)
{
    RoboCompCameraRGBDSimple::TDepth newdepthImage;

    // Se establece el periodo de refresco de la imagen en milisegundos.
    newdepthImage.period = 100;

    // Obtener la resolución de la imagen.
    newdepthImage.width = _msg.width();
    newdepthImage.height = _msg.height();

    // Obtener la imagen y asignarla al tipo TImage de Robocomp
    newdepthImage.depth.assign(_msg.data().begin(), _msg.data().end());
    newdepthImage.compressed = false;

    // Asignamos el resultado final al atributo de clase
    SpecificWorker::depthImage = newdepthImage;
    fps.print("FPS:");
}

//////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region SimpleCameraRGBD

RoboCompCameraRGBDSimple::TRGBD SpecificWorker::CameraRGBDSimple_getAll(std::string camera)
{
//implementCODE

}

RoboCompCameraRGBDSimple::TDepth SpecificWorker::CameraRGBDSimple_getDepth(std::string camera)
{
    return SpecificWorker::depthImage;
}

RoboCompCameraRGBDSimple::TImage SpecificWorker::CameraRGBDSimple_getImage(std::string camera)
{
//implementCODE
   
}

RoboCompCameraRGBDSimple::TPoints SpecificWorker::CameraRGBDSimple_getPoints(std::string camera)
{
//implementCODE

}

#pragma endregion SimpleCameraRGBD

#pragma region OmniRobot

void SpecificWorker::OmniRobot_correctOdometer(int x, int z, float alpha)
{
//implementCODE

}

void SpecificWorker::OmniRobot_getBasePose(int &x, int &z, float &alpha)
{
//implementCODE

}

void SpecificWorker::OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state)
{
//implementCODE

}

void SpecificWorker::OmniRobot_resetOdometer()
{
//implementCODE

}

void SpecificWorker::OmniRobot_setOdometer(RoboCompGenericBase::TBaseState state)
{
//implementCODE

}

void SpecificWorker::OmniRobot_setOdometerPose(int x, int z, float alpha)
{
//implementCODE

}

void SpecificWorker::OmniRobot_setSpeedBase(float advx, float advz, float rot)
{
//implementCODE

}

void SpecificWorker::OmniRobot_stopBase()
{
//implementCODE

}

#pragma endregion OmniRobot

#pragma region LIDAR

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState)
{
//implementCODE

}

RoboCompLaser::LaserConfData SpecificWorker::Laser_getLaserConfData()
{
//implementCODE

}

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserData()
{
//implementCODE

}

#pragma endregion LIDAR

/**
 * @brief Subscription callback for the sendData method of the JoystickAdapter interface.
 *
 * @param[in] data Data structure containing information about the joystick buttons and axes.
 */
void SpecificWorker::JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data)
{
    // Declaration of the structure to be filled
    gz::msgs::Twist dataMsg;
    // Declaration of Gazebo publisher
    gz::transport::Node::Publisher pub = SpecificWorker::node.Advertise<gz::msgs::Twist>(ROBOCOMP_JOYSTICKADAPTER);

    // Iterate through the list of buttons in the data structure
    for (RoboCompJoystickAdapter::ButtonParams button : data.buttons) {
        // Currently does nothing with the buttons
    }

    // Iterate through the list of axes in the data structure
    for (RoboCompJoystickAdapter::AxisParams axis : data.axes){
        // Process the axis according to its name
        if(axis.name == "rotate") {
            dataMsg.mutable_angular()->set_z(axis.value);
        }
        else if (axis.name == "advance") {
            dataMsg.mutable_linear()->set_x(axis.value);
        }
        else if (axis.name == "side") {
            dataMsg.mutable_linear()->set_y(axis.value);
        }
        else {
            cout << "[ JoystickAdapter: ] Warning: Velocidad no ajustada." << endl;
        }
    }

    // Publish to Gazebo with the actual Joystick output.
    pub.Publish(dataMsg);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::Point3D
// RoboCompCameraRGBDSimple::TPoints
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompOmniRobot you can use this types:
// RoboCompOmniRobot::TMechParams

/**************************************/
// From the RoboCompJoystickAdapter you can use this types:
// RoboCompJoystickAdapter::AxisParams
// RoboCompJoystickAdapter::ButtonParams
// RoboCompJoystickAdapter::TData

