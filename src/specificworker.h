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

/**
	\brief
	@author authorname


*/

#pragma once

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <fps/fps.h>

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
	void Gazebo2Robocomp_removeEntity(std::string name);
	void Gazebo2Robocomp_setEntityPose(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation);


        // JOYSTICK
        void JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data);

        // Gazebo Transport client
        static gz::transport::Node node;

    public slots:
        void compute();
        int startup_check();
        void initialize(int period);

    private:
        std::shared_ptr < InnerModel > innerModel;
        bool startup_check_flag;
        FPSCounter fps;

        // Components parameters
        std::string odometryTargetName;

        // Camera RGBD simple
        RoboCompCameraRGBDSimple::TDepth depthImage;
        RoboCompCameraRGBDSimple::TImage cameraImage;

        // Laser
        RoboCompLaser::TLaserData laserData;
        RoboCompLaser::LaserConfData laserDataConf;

        // Odometer
        RoboCompGenericBase::TBaseState odometryTargetState;
        string completeOdometryTopic;

        // Imu
        RoboCompIMU::Acceleration imuAcceleration;
        RoboCompIMU::Gyroscope imuAngularVel;
        RoboCompIMU::DataImu imuDataImu;
        RoboCompIMU::Magnetic imuMagneticFields;
        RoboCompIMU::Orientation imuOrientation;

        // Callbacks functions
        void depth_camera_cb(const gz::msgs::Image &_msg);
        void lidar_cb(const gz::msgs::LaserScan &_msg);
        void camera_cb(const gz::msgs::Image &_msg);
        void odometry_cb(const gz::msgs::Odometry &_msg);
        void imu_cb(const gz::msgs::IMU &_msg);

        // Auxiliar functions
        void printNotImplementedWarningMessage(string functionName);



};

#endif
