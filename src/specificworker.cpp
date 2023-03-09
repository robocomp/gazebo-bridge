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
    odometryTargetName = params["odometry_target_name"].value;

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

    completeOdometryTopic = "/model/" + odometryTargetName + "/odometry";

    // TODO: Extraer llamadas de suscripción a un método.

    // Subscribe to depth_camera topic by registering a callback
    if (!node.Subscribe(ROBOCOMP_DEPTHCAMERA, &SpecificWorker::depth_camera_cb, this))
        cerr << "Error subscribing to topic [" << ROBOCOMP_DEPTHCAMERA << "]" << std::endl;
    else
        cout << "SpecificWorker suscribed to [" << ROBOCOMP_DEPTHCAMERA << "]" << std::endl;

    // Subscribe to LIDAR topic by registering a callback
    if (!node.Subscribe(ROBOCOMP_LIDAR, &SpecificWorker::lidar_cb, this))
        cerr << "Error subscribing to topic [" << ROBOCOMP_LIDAR << "]" << std::endl;
    else
        cout << "SpecificWorker suscribed to [" << ROBOCOMP_LIDAR << "]" << std::endl;

    // Subscribe to camera topic by registering a callback
    if (!node.Subscribe(ROBOCOMP_CAMERA, &SpecificWorker::camera_cb, this))
        cerr << "Error subscribing to topic [" << ROBOCOMP_CAMERA << "]" << std::endl;
    else
        cout << "SpecificWorker suscribed to [" << ROBOCOMP_CAMERA << "]" << std::endl;

    // Subscribe to imu topic by registering a callback
    if (!node.Subscribe(ROBOCOMP_IMU, &SpecificWorker::imu_cb, this))
        cerr << "Error subscribing to topic [" << ROBOCOMP_IMU << "]" << std::endl;
    else
        cout << "SpecificWorker suscribed to [" << ROBOCOMP_IMU << "]" << std::endl;

    // If target is specified
    if(odometryTargetName != "none") {
        // Subscribe to odometry topic by registering a callback
        if (!node.Subscribe(completeOdometryTopic, &SpecificWorker::odometry_cb, this))
            cerr << "Error subscribing to topic [" << completeOdometryTopic << "]" << std::endl;
        else
            cout << "SpecificWorker suscribed to [" << completeOdometryTopic << "]" << std::endl;
    }

    /*
    // ##################### PROBANDO CREACION DE ENTIDADES EN RUNTIME #################################
    gz::msgs::EntityFactory dataMsg;

    auto cylinderStr = R"(
    <?xml version="1.0" ?>
    <sdf version='1.6'>
      <model name='robolab_cylinder'>
      <pose>0 -1.5 0.5 0 0 0</pose>
      <link name='cylinder_link'>
        <visual name='cylinder_visual'>
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <length>1</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
            <specular>0 1 0 1</specular>
          </material>
        </visual>
        <pose>0 0 0 0 0 0</pose>
        <enable_wind>false</enable_wind>
      </link>
    </model>
    </sdf>
    )";
    dataMsg.set_sdf(cylinderStr);
    dataMsg.clear_pose();
    dataMsg.set_name("new_name");
    dataMsg.set_allow_renaming(true);

    gz::msgs::Boolean reply;
    bool result;
    const unsigned int timeout = 300;
    bool executed = node.Request("/world/basic/create", dataMsg, timeout, reply, result);

    if (executed)
        cout << "Service executed successfully" << endl;
    else
        cerr << "Service call timed out" << endl;
    */

    /*
    // ##################### PROBANDO BORRADO DE ENTIDADES EN RUNTIME #################################
    gz::msgs::Entity entity;
    gz::msgs::Boolean reply;
    bool result;
    const unsigned int timeout = 300;

    entity.set_name("new_name");
    entity.set_type(gz::msgs::Entity_Type::Entity_Type_MODEL);

    bool executed = node.Request("/world/basic/remove", entity, timeout, reply, result);

    if (executed)
        cout << "Service executed successfully" << endl;
    else
        cerr << "Service call timed out" << endl;
    */

    /*
    // ##################### PROBANDO MOVER ENTIDADES EN RUNTIME #################################
    gz::msgs::Pose pose;
    gz::msgs::Boolean reply;
    bool result;
    const unsigned int timeout = 300;

    pose.set_name("new_name");
    pose.mutable_position()->set_x(20);

    bool executed = node.Request("/world/basic/set_pose", pose, timeout, reply, result);

    if (executed)
        cout << "Service executed successfully" << endl;
    else
        cerr << "Service call timed out" << endl;
    */

}



void SpecificWorker::compute()
{
    // DEBUG: LIDAR
    /*
    std::cout << "Laser Data:" << std::endl;
    for (const auto& data : laserData)
    {
        std::cout << "Angle: " << data.angle << " degrees" << std::endl;
        std::cout << "Distance: " << data.dist << " meters" << std::endl;
    }
     */


}

///////////////////////////////////////////////////////////////////////////////////
#pragma region Gazebo_CallbackFunctions

/**
 * @brief Subscription callback for the odometry values.
 *
 * @param[in] _msg Data structure containing information about the odometer.
 */
void SpecificWorker::odometry_cb(const gz::msgs::Odometry &_msg)
{
    RoboCompGenericBase::TBaseState newOdometryData;
    // Valores de posición
    newOdometryData.x = _msg.pose().position().x();
    // En Gazebo el eje Y es el que representa el movimiento lateral.
    newOdometryData.z = _msg.pose().position().y();

    // Valores corregidos
    newOdometryData.correctedX = ceil(newOdometryData.x * 100.0) / 100.0;
    newOdometryData.correctedZ = ceil(newOdometryData.z * 100.0) / 100.0;

    // Valores de velocidad
    newOdometryData.advVx = _msg.twist().linear().x();
    newOdometryData.advVz = _msg.twist().linear().y();

    // Se está moviendo si alguna de las velocidades es mayor que 0
    newOdometryData.isMoving = (abs(newOdometryData.advVx)  > .01 || abs(newOdometryData.advVz)  > .01);

    // Valor de la velocidad de rotación
    newOdometryData.rotV = _msg.twist().angular().z();


    // TODO: Valor de rotación acumulada
    // Actualmente newOdometryData es la acumulación del valor redondeado de la velocidad de
    // rotación
    // newOdometryData.alpha = ?;

    odometryTargetState = newOdometryData;
}

/**
 * @brief Subscription callback for the LIDAR sensor in Gazebo.
 *
 * @param[in] _msg Data structure containing information about the LIDAR.
 */
void SpecificWorker::lidar_cb(const gz::msgs::LaserScan &_msg)
{
    RoboCompLaser::TLaserData newLaserData;
    RoboCompLaser::LaserConfData newLaserConfData;

    // ## DATOS DE CONFIGURACION DE LOS LASERES

    newLaserConfData.maxDegrees = _msg.angle_max();
    newLaserConfData.maxRange = _msg.range_max();
    newLaserConfData.minRange = _msg.range_min();
    newLaserConfData.angleRes = _msg.angle_step();

    // ## DATOS DE LOS LÁSERES

    // Iterate through ranges array in _msg and create TData structs
    for (int i = 0; i < _msg.ranges_size(); i++)
    {
        RoboCompLaser::TData data;
        data.angle = _msg.angle_min() + i * _msg.angle_step();
        data.dist = _msg.ranges(i);
        newLaserData.push_back(data);
    }

    laserData = newLaserData;
    laserDataConf = newLaserConfData;
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
    fps.print("Detph Camera FPS:");
}

/**
 * @brief Subscription callback for the CAMERA sensor in Gazebo.
 *
 * @param[in] _msg Data structure containing information about the CAMERA Image.
 */
void SpecificWorker::camera_cb(const gz::msgs::Image &_msg)
{
    RoboCompCameraRGBDSimple::TImage newImage;

    // Se establece el periodo de refresco de la imagen en milisegundos.
    newImage.period = 100;

    // Obtener la resolución de la imagen.
    newImage.width = _msg.width();
    newImage.height = _msg.height();

    // Obtener la imagen y asignarla al tipo TImage de Robocomp
    newImage.image.assign(_msg.data().begin(), _msg.data().end());
    newImage.compressed = false;

    // Asignamos el resultado final al atributo de clase
    this->cameraImage = newImage;
    fps.print("Camera FPS:");
}

/**
 * @brief Subscription callback for the IMU sensor in Gazebo.
 *
 * @param[in] _msg Data structure containing information about the IMU data.
 */
void SpecificWorker::imu_cb(const gz::msgs::IMU &_msg)
{

    // Aceleración linear
    RoboCompIMU::Acceleration newAcceleration;
    newAcceleration.XAcc = _msg.linear_acceleration().x();
    newAcceleration.YAcc = _msg.linear_acceleration().y();
    newAcceleration.ZAcc = _msg.linear_acceleration().z();
    imuAcceleration = newAcceleration;

    // Velocidad angular
    RoboCompIMU::Gyroscope newAngularVel;
    newAngularVel.XGyr = _msg.angular_velocity().x();
    newAngularVel.YGyr = _msg.angular_velocity().y();
    newAngularVel.ZGyr = _msg.angular_velocity().z();
    imuAngularVel = newAngularVel;

    // Orientation
    RoboCompIMU::Orientation newOrientation;
    newOrientation.Pitch = _msg.orientation().x();
    newOrientation.Roll = _msg.orientation().y();
    newOrientation.Yaw = _msg.orientation().z();
    imuOrientation = newOrientation;

    // Magnetic field
    RoboCompIMU::Magnetic newMagneticFields;
    // TODO: Recoger datos de magnetismo.

    // IMU Data
    imuDataImu.acc = newAcceleration;
    imuDataImu.gyro = newAngularVel;
    imuDataImu.rot = newOrientation;
    imuDataImu.mag = newMagneticFields;
}

#pragma endregion Gazebo_CallbackFunctions

//////////////////////////////////////////////////////////////////////////////////////////////////
#pragma region SimpleCameraRGBD

RoboCompCameraRGBDSimple::TRGBD SpecificWorker::CameraRGBDSimple_getAll(std::string camera)
{
    RoboCompCameraRGBDSimple::TRGBD newRGBD;

    newRGBD.image = this->cameraImage;
    newRGBD.depth = this->depthImage;
    // TODO: Que devuelva tambien la nube de puntos.

    return newRGBD;
}

RoboCompCameraRGBDSimple::TDepth SpecificWorker::CameraRGBDSimple_getDepth(std::string camera)
{
    return this->depthImage;
}

RoboCompCameraRGBDSimple::TImage SpecificWorker::CameraRGBDSimple_getImage(std::string camera)
{
    return this->cameraImage;
}

#pragma endregion SimpleCameraRGBD

#pragma region OmniRobot

void SpecificWorker::OmniRobot_correctOdometer(int x, int z, float alpha)
{
    // TODO: Implement if required
    printNotImplementedWarningMessage("OmniRobot_correctOdometer");
}

void SpecificWorker::OmniRobot_getBasePose(int &x, int &z, float &alpha)
{
    x = odometryTargetState.x;
    z = odometryTargetState.z;
    alpha = odometryTargetState.alpha;
}

void SpecificWorker::OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state)
{
    state = odometryTargetState;
}

void SpecificWorker::OmniRobot_resetOdometer()
{
    // Declaration of Gazebo Odometry message
    gz::msgs::Odometry dataMsg;
    // Declaration of Gazebo publisher
    gz::transport::Node::Publisher pub = SpecificWorker::node.Advertise<gz::msgs::Odometry>(completeOdometryTopic);

    // Clear odometry data. Maybe not necessary.
    dataMsg.clear_header();
    dataMsg.clear_pose();
    dataMsg.clear_twist();

    // Publish to Gazebo with the actual Joystick output.
    pub.Publish(dataMsg);
}

void SpecificWorker::OmniRobot_setOdometer(RoboCompGenericBase::TBaseState state)
{
    // Declaration of Gazebo Odometry message
    gz::msgs::Odometry dataMsg;
    // Declaration of Gazebo publisher
    gz::transport::Node::Publisher pub = SpecificWorker::node.Advertise<gz::msgs::Odometry>(completeOdometryTopic);

    // Valores de posición
    dataMsg.mutable_pose()->mutable_position()->set_x(state.correctedX);
    dataMsg.mutable_pose()->mutable_position()->set_y(state.correctedZ);

    // Valores de velocidad
    dataMsg.mutable_twist()->mutable_linear()->set_x(state.advVx);
    dataMsg.mutable_twist()->mutable_linear()->set_y(state.advVz);

    // Valor de la velocidad de rotación
    dataMsg.mutable_twist()->mutable_angular()->set_z(state.rotV);

    // Publish to Gazebo with the actual Joystick output.
    pub.Publish(dataMsg);
}

void SpecificWorker::OmniRobot_setOdometerPose(int x, int z, float alpha)
{
    // TODO: Implement
    printNotImplementedWarningMessage("OmniRobot_setOdometerPose");
}

void SpecificWorker::OmniRobot_setSpeedBase(float advx, float advz, float rot)
{
    // TODO: Implement
    printNotImplementedWarningMessage("OmniRobot_setSpeedBase");
}

void SpecificWorker::OmniRobot_stopBase()
{
    // TODO: Implement
    printNotImplementedWarningMessage("OmniRobot_stopBase");
}

#pragma endregion OmniRobot

#pragma region LIDAR

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState)
{
    bState = odometryTargetState;
    return laserData;
}

RoboCompLaser::LaserConfData SpecificWorker::Laser_getLaserConfData()
{
    return laserDataConf;
}

RoboCompLaser::TLaserData SpecificWorker::Laser_getLaserData()
{
    return SpecificWorker::laserData;
}

#pragma endregion LIDAR

#pragma region IMU

RoboCompIMU::Acceleration SpecificWorker::IMU_getAcceleration()
{
    return this->imuAcceleration;
}

RoboCompIMU::Gyroscope SpecificWorker::IMU_getAngularVel()
{
    return this->imuAngularVel;
}

RoboCompIMU::DataImu SpecificWorker::IMU_getDataImu()
{
    return this->imuDataImu;
}

RoboCompIMU::Magnetic SpecificWorker::IMU_getMagneticFields()
{
    return this->imuMagneticFields;
}

RoboCompIMU::Orientation SpecificWorker::IMU_getOrientation()
{
    return this->imuOrientation;
}

void SpecificWorker::IMU_resetImu()
{
    // Declaration of Gazebo IMU message
    gz::msgs::IMU dataMsg;
    // Declaration of Gazebo publisher
    gz::transport::Node::Publisher pub = SpecificWorker::node.Advertise<gz::msgs::IMU>(ROBOCOMP_IMU);

    // Clear imu data. Maybe not necessary.
    dataMsg.clear_orientation();
    dataMsg.clear_angular_velocity();
    dataMsg.clear_linear_acceleration();

    // Publish to Gazebo with the actual Joystick output.
    pub.Publish(dataMsg);
}

#pragma endregion IMU

#pragma region JointMotorSimple

RoboCompJointMotorSimple::MotorParams SpecificWorker::JointMotorSimple_getMotorParams(std::string motor)
{
    // TODO: Implement
    printNotImplementedWarningMessage("JointMotorSimple_getMotorParams");
}

RoboCompJointMotorSimple::MotorState SpecificWorker::JointMotorSimple_getMotorState(std::string motor)
{
    // TODO: Implement
    printNotImplementedWarningMessage("JointMotorSimple_getMotorState");
}

void SpecificWorker::JointMotorSimple_setPosition(std::string name, RoboCompJointMotorSimple::MotorGoalPosition goal)
{
    // TODO: Implement
    printNotImplementedWarningMessage("JointMotorSimple_setPosition");
}

void SpecificWorker::JointMotorSimple_setVelocity(std::string name, RoboCompJointMotorSimple::MotorGoalVelocity goal)
{
    // Declaration of the structure to be filled
    gz::msgs::Twist dataMsg;
    // Declaration of Gazebo publisher
    gz::transport::Node::Publisher pub = SpecificWorker::node.Advertise<gz::msgs::Twist>(ROBOCOMP_JOINTMOTORSIMPLE);

    // Setting the velocity
    dataMsg.mutable_linear()->set_x(goal.velocity);

    // Publish to Gazebo with the actual Joystick output.
    pub.Publish(dataMsg);
}

void SpecificWorker::JointMotorSimple_setZeroPos(std::string name)
{
    // TODO: Implement
    printNotImplementedWarningMessage("JointMotorSimple_setZeroPos");
}

#pragma endregion JointMotorSimple



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

void SpecificWorker::printNotImplementedWarningMessage(string functionName)
{
    cout << "Function not implemented used: " << "[" << functionName << "]" << std::endl;
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

/**************************************/
// From the RoboCompLaser you can use this types:
// RoboCompLaser::LaserConfData
// RoboCompLaser::TData


