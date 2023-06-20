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

/**
	\brief
	@author authorname


*/

#pragma once

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <fps/fps.h>
#include <string>
#include <cmath>
#include "topics.h"

// Gazebo
#include <gz/msgs.hh>
#include <gz/transport.hh>

class SpecificWorker : public GenericWorker
{
    Q_OBJECT
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);

        // RGBD Camera
        RoboCompCameraRGBDSimple::TRGBD CameraRGBDSimple_getAll(std::string camera);
        RoboCompCameraRGBDSimple::TDepth CameraRGBDSimple_getDepth(std::string camera);
        RoboCompCameraRGBDSimple::TImage CameraRGBDSimple_getImage(std::string camera);
        RoboCompCameraRGBDSimple::TPoints CameraRGBDSimple_getPoints(std::string camera){};

	    // Camera 360 RGB
	    RoboCompCamera360RGB::TImage Camera360RGB_getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight);

        // LIDAR
        RoboCompLaser::TLaserData Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState);
        RoboCompLaser::LaserConfData Laser_getLaserConfData();
        RoboCompLaser::TLaserData Laser_getLaserData();

        // OMNIROBOT
        void OmniRobot_correctOdometer(int x, int z, float alpha);
        void OmniRobot_getBasePose(int &x, int &z, float &alpha);
        void OmniRobot_getBaseState(RoboCompGenericBase::TBaseState &state);
        void OmniRobot_resetOdometer();
        void OmniRobot_setOdometer(RoboCompGenericBase::TBaseState state);
        void OmniRobot_setOdometerPose(int x, int z, float alpha);
        void OmniRobot_setSpeedBase(float advx, float advz, float rot);
        void OmniRobot_stopBase();

	// DIFFERENTIALROBOT
	void DifferentialRobot_correctOdometer(int x, int z, float alpha);
	void DifferentialRobot_getBasePose(int &x, int &z, float &alpha);
	void DifferentialRobot_getBaseState(RoboCompGenericBase::TBaseState &state);
	void DifferentialRobot_resetOdometer();
	void DifferentialRobot_setOdometer(RoboCompGenericBase::TBaseState state);
	void DifferentialRobot_setOdometerPose(int x, int z, float alpha);
	void DifferentialRobot_setSpeedBase(float adv, float rot);
	void DifferentialRobot_stopBase();

        // IMU
        RoboCompIMU::Acceleration IMU_getAcceleration();
        RoboCompIMU::Gyroscope IMU_getAngularVel();
        RoboCompIMU::DataImu IMU_getDataImu();
        RoboCompIMU::Magnetic IMU_getMagneticFields();
        RoboCompIMU::Orientation IMU_getOrientation();
        void IMU_resetImu();

        // JOINTMOTORSIMPLE
        RoboCompJointMotorSimple::MotorParams JointMotorSimple_getMotorParams(std::string motor);
        RoboCompJointMotorSimple::MotorState JointMotorSimple_getMotorState(std::string motor);
        void JointMotorSimple_setPosition(std::string name, RoboCompJointMotorSimple::MotorGoalPosition goal);
        void JointMotorSimple_setVelocity(std::string name, RoboCompJointMotorSimple::MotorGoalVelocity goal);
        void JointMotorSimple_setZeroPos(std::string name);

        // GAZEBO2ROBOCOMP Interfaces
        void Gazebo2Robocomp_createBoxEntity(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float size);
        void Gazebo2Robocomp_createCapsuleEntity(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float length, float radius);
        void Gazebo2Robocomp_createCylinderEntity(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float length, float radius);
        void Gazebo2Robocomp_createEntity(std::string sdf);
        void Gazebo2Robocomp_createSphereEntity(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float radius);
	void Gazebo2Robocomp_createRandomBoxEntity(RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float size);
	void Gazebo2Robocomp_createRandomCapsuleEntity(RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float length, float radius);
	void Gazebo2Robocomp_createRandomCylinderEntity(RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float length, float radius);
	void Gazebo2Robocomp_createRandomHumanEntity(RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation);
	void Gazebo2Robocomp_createRandomSphereEntity(RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float radius);
        void Gazebo2Robocomp_removeEntity(std::string name);
        void Gazebo2Robocomp_setEntityPose(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation);
	RoboCompGazebo2Robocomp::Vector3 Gazebo2Robocomp_getWorldPosition(std::string name);
	void Gazebo2Robocomp_setLinearVelocity(std::string name, RoboCompGazebo2Robocomp::Vector3 velocity);
        void Gazebo2Robocomp_createHumanEntity(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation);

        // LIDAR 3D
        RoboCompLidar3D::TLidarData Lidar3D_getLidarData(std::string name, int start, int len, int decimationfactor);

        // JOYSTICK
        void JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data);

        // Gazebo Transport client
        gz::transport::Node node;

        // Data structure for the object data tracking funcionality.
        struct ObjectData{
            RoboCompGazebo2Robocomp::Vector3 position;
    };

    public slots:
        void compute();
        int startup_check();
        void initialize(int period);

    private:
        bool startup_check_flag;
        FPSCounter fps;

        // Components parameters
        std::string odometryTargetName;
        std::string gazeboWorldName;

        // Camera RGBD simple
        RoboCompCameraRGBDSimple::TDepth depthImage;
        RoboCompCameraRGBDSimple::TImage cameraImage;

        // Laser
        RoboCompLaser::TLaserData laserData;
        RoboCompLaser::LaserConfData laserDataConf;

        // Lidar3d
        RoboCompLidar3D::TLidarData lidar3dData;

        // Odometer
        RoboCompGenericBase::TBaseState odometryTargetState;
        string completeOdometryTopic;
        gz::msgs::Odometry lastOdometryValue;

        // Imu
        RoboCompIMU::Acceleration imuAcceleration;
        RoboCompIMU::Gyroscope imuAngularVel;
        RoboCompIMU::DataImu imuDataImu;
        RoboCompIMU::Magnetic imuMagneticFields;
        RoboCompIMU::Orientation imuOrientation;

        // Callbacks functions for sensors
        void depth_camera_cb(const gz::msgs::Image &_msg);
        void lidar_cb(const gz::msgs::LaserScan &_msg);
        void camera_cb(const gz::msgs::Image &_msg);
        void odometry_cb(const gz::msgs::Odometry &_msg);
        void imu_cb(const gz::msgs::IMU &_msg);

        // Objects for the object data tracking
        std::map<std::string, shared_ptr<ObjectData>> objectsData;
        void trackObject(const std::string& objectName);
        void trackObject_cb(const gz::msgs::Pose &_msg);



        // Auxiliar functions
        void printNotImplementedWarningMessage(string functionName);
        // Checks if an object is being tracked
        bool isTracking(const std::string& objectName) {
            return objectsData.count(objectName) > 0;
        }
};

#endif
