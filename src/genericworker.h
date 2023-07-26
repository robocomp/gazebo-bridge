/*
 *    Copyright (C) 2023 by YOUR NAME HERE
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
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <stdint.h>
#include <qlog/qlog.h>
#include <CommonBehavior.h>

#include <Camera360RGB.h>
#include <CameraRGBDSimple.h>
#include <DifferentialRobot.h>
#include <Gazebo2Robocomp.h>
#include <GenericBase.h>
#include <IMU.h>
#include <JointMotorSimple.h>
#include <JoystickAdapter.h>
#include <Laser.h>
#include <Lidar3D.h>
#include <OmniRobot.h>


#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100


using TuplePrx = std::tuple<>;


class GenericWorker : public QObject
{
Q_OBJECT
public:
	GenericWorker(TuplePrx tprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;



	virtual RoboCompCamera360RGB::TImage Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight) = 0;
	virtual RoboCompCameraRGBDSimple::TRGBD CameraRGBDSimple_getAll(std::string camera) = 0;
	virtual RoboCompCameraRGBDSimple::TDepth CameraRGBDSimple_getDepth(std::string camera) = 0;
	virtual RoboCompCameraRGBDSimple::TImage CameraRGBDSimple_getImage(std::string camera) = 0;
	virtual RoboCompCameraRGBDSimple::TPoints CameraRGBDSimple_getPoints(std::string camera) = 0;
	virtual void DifferentialRobot_correctOdometer(int x, int z, float alpha) = 0;
	virtual void DifferentialRobot_getBasePose(int &x, int &z, float &alpha) = 0;
	virtual void DifferentialRobot_getBaseState(RoboCompGenericBase::TBaseState &state) = 0;
	virtual void DifferentialRobot_resetOdometer() = 0;
	virtual void DifferentialRobot_setOdometer(RoboCompGenericBase::TBaseState state) = 0;
	virtual void DifferentialRobot_setOdometerPose(int x, int z, float alpha) = 0;
	virtual void DifferentialRobot_setSpeedBase(float adv, float rot) = 0;
	virtual void DifferentialRobot_stopBase() = 0;
	virtual void Gazebo2Robocomp_createBoxEntity(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float size) = 0;
	virtual void Gazebo2Robocomp_createCapsuleEntity(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float length, float radius) = 0;
	virtual void Gazebo2Robocomp_createCylinderEntity(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float length, float radius) = 0;
	virtual void Gazebo2Robocomp_createEntity(std::string sdf) = 0;
	virtual void Gazebo2Robocomp_createHumanEntity(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation) = 0;
	virtual void Gazebo2Robocomp_createRandomBoxEntity(RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float size) = 0;
	virtual void Gazebo2Robocomp_createRandomCapsuleEntity(RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float length, float radius) = 0;
	virtual void Gazebo2Robocomp_createRandomCylinderEntity(RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float length, float radius) = 0;
	virtual void Gazebo2Robocomp_createRandomHumanEntity(RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation) = 0;
	virtual void Gazebo2Robocomp_createRandomSphereEntity(RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float radius) = 0;
	virtual void Gazebo2Robocomp_createSphereEntity(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float radius) = 0;
	virtual RoboCompGazebo2Robocomp::Vector3 Gazebo2Robocomp_getWorldPosition(std::string name) = 0;
	virtual void Gazebo2Robocomp_removeEntity(std::string name) = 0;
	virtual void Gazebo2Robocomp_setEntityPose(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation) = 0;
	virtual void Gazebo2Robocomp_setLinearVelocity(std::string name, RoboCompGazebo2Robocomp::Vector3 velocity) = 0;
	virtual RoboCompIMU::Acceleration IMU_getAcceleration() = 0;
	virtual RoboCompIMU::Gyroscope IMU_getAngularVel() = 0;
	virtual RoboCompIMU::DataImu IMU_getDataImu() = 0;
	virtual RoboCompIMU::Magnetic IMU_getMagneticFields() = 0;
	virtual RoboCompIMU::Orientation IMU_getOrientation() = 0;
	virtual void IMU_resetImu() = 0;
	virtual RoboCompJointMotorSimple::MotorParams JointMotorSimple_getMotorParams(std::string motor) = 0;
	virtual RoboCompJointMotorSimple::MotorState JointMotorSimple_getMotorState(std::string motor) = 0;
	virtual void JointMotorSimple_setPosition(std::string name, RoboCompJointMotorSimple::MotorGoalPosition goal) = 0;
	virtual void JointMotorSimple_setVelocity(std::string name, RoboCompJointMotorSimple::MotorGoalVelocity goal) = 0;
	virtual void JointMotorSimple_setZeroPos(std::string name) = 0;
	virtual RoboCompLaser::TLaserData Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState) = 0;
	virtual RoboCompLaser::LaserConfData Laser_getLaserConfData() = 0;
	virtual RoboCompLaser::TLaserData Laser_getLaserData() = 0;
	virtual RoboCompLidar3D::TData Lidar3D_getLidarData(std::string name, int start, int len, int decimationfactor) = 0;
	virtual void OmniRobot_correctOdometer(int x, int z, float alpha) = 0;
	virtual void OmniRobot_getBasePose(int &x, int &z, float &alpha) = 0;
	virtual void OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state) = 0;
	virtual void OmniRobot_resetOdometer() = 0;
	virtual void OmniRobot_setOdometer(RoboCompGenericBase::TBaseState state) = 0;
	virtual void OmniRobot_setOdometerPose(int x, int z, float alpha) = 0;
	virtual void OmniRobot_setSpeedBase(float advx, float advz, float rot) = 0;
	virtual void OmniRobot_stopBase() = 0;
	virtual void JoystickAdapter_sendData (RoboCompJoystickAdapter::TData data) = 0;

protected:

	QTimer timer;
	int Period;

private:


public slots:
	virtual void compute() = 0;
	virtual void initialize(int period) = 0;
	
signals:
	void kill();
};

#endif
