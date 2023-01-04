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
RoboCompCameraRGBDSimple::TDepth SpecificWorker::depthImage;

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
	// Uncomment if there's too many debug messages
	// but it removes the possibility to see the messages
	// shown in the console with qDebug()
//	QLoggingCategory::setFilterRules("*.debug=false\n");
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
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }

	return true;
}

#pragma region Gazebo_CallbackFunctions

void lidar_cb(const gz::msgs::LaserScan &_msg)
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

void depth_camera_cb(const gz::msgs::Image &_msg)
{
    RoboCompCameraRGBDSimple::TDepth newdepthImage;

    // Obtener la resolución de la imagen.
    newdepthImage.width = _msg.width();
    newdepthImage.height = _msg.height();

    // Obtener la imagen y asignarla al tipo TImage de Robocomp
    newdepthImage.depth.assign(_msg.data().begin(), _msg.data().end());

    // Asignamos el resultado final al atributo de clase
    SpecificWorker::depthImage = newdepthImage;
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

    #pragma region SubscribingGazeboNodeExample
    /*
    // Linking call back function to lidar sensor.
    string topic = "/lidar";
    // Subscribe to a topic by registering a callback
    if (!node.Subscribe(topic, &lidar_cb))
    {
        cerr << "Error subscribing to topic [" << topic << "]" << std::endl;
    }else{
        cout << "SpecificWorker suscribed to [" << topic << "]" << std::endl;
    }

    topic = "/depth_camera";
    // Subscribe to depth_camera topic by registering a callback
    if (!node.Subscribe(topic, &depth_camera_cb))
    {
        cerr << "Error subscribing to topic [" << topic << "]" << std::endl;
    }else{
        cout << "SpecificWorker suscribed to [" << topic << "]" << std::endl;
    }
     */
    #pragma endregion SubscribingGazeboNodeExample

    // Subscribe to depth_camera topic by registering a callback
    if (!node.Subscribe(ROBOCOMP_DEPTHCAMERA, &depth_camera_cb))
    {
        cerr << "Error subscribing to topic [" << ROBOCOMP_DEPTHCAMERA << "]" << std::endl;
    }else{
        cout << "SpecificWorker suscribed to [" << ROBOCOMP_DEPTHCAMERA << "]" << std::endl;
    }

}


void SpecificWorker::compute()
{
	//computeCODE
	//QMutexLocker locker(mutex);
	//try
	//{
	//  camera_proxy->getYImage(0,img, cState, bState);
	//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	//  searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	//  std::cout << "Error reading from Camera" << e << std::endl;
	//}

    JoystickAdapter2Gazebo();
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::JoystickAdapter2Gazebo(){

    // Joystick control
    gz::msgs::Twist dataMsg;

    dataMsg.mutable_linear()->set_x(advance);
    dataMsg.mutable_linear()->set_y(side);
    dataMsg.mutable_angular()->set_z(rotation);

    gz::transport::Node::Publisher pub = SpecificWorker::node.Advertise<gz::msgs::Twist>(ROBOCOMP_JOYSTICKADAPTER);
    pub.Publish(dataMsg);
}

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

/**
 * @brief Subscription callback for the sendData method of the JoystickAdapter interface.
 *
 * @param[in] data Data structure containing information about the joystick buttons and axes.
 */
void SpecificWorker::JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data)
{
    // Iterate through the list of buttons in the data structure
    for (RoboCompJoystickAdapter::ButtonParams button : data.buttons) {
        // Currently does nothing with the buttons
    }

    // Iterate through the list of axes in the data structure
    for (RoboCompJoystickAdapter::AxisParams axis : data.axes){
        // Process the axis according to its name
        if(axis.name == "rotate") {
            SetRotation(axis.value);
        }
        else if (axis.name == "advance") {
            SetAdvance(axis.value);
        }
        else if (axis.name == "side") {
            SetSide(axis.value);
        }
        else {
            cout << "[ JoystickAdapter: ] Warning: Velocidad no ajustada." << endl;
        }
    }
}

# pragma region Getters&Setters

void SpecificWorker::SetRotation(float newRotation) {
    rotation = newRotation;
}

float SpecificWorker::GetRotation() {
    return rotation;
}

void SpecificWorker::SetAdvance(float newAdvance) {
    advance = newAdvance;
}

float SpecificWorker::GetAdvance() {
    return advance;
}

void SpecificWorker::SetSide(float newSide) {
    side = newSide;
}

float SpecificWorker::GetSide() {
    return side;
}

# pragma endregion Getters&Setters



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

