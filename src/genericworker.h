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

#include <CameraRGBDSimple.h>
#include <GenericBase.h>
#include <JoystickAdapter.h>
#include <Laser.h>
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



	virtual RoboCompCameraRGBDSimple::TRGBD CameraRGBDSimple_getAll(std::string camera) = 0;
	virtual RoboCompCameraRGBDSimple::TDepth CameraRGBDSimple_getDepth(std::string camera) = 0;
	virtual RoboCompCameraRGBDSimple::TImage CameraRGBDSimple_getImage(std::string camera) = 0;
	virtual RoboCompCameraRGBDSimple::TPoints CameraRGBDSimple_getPoints(std::string camera) = 0;
	virtual RoboCompLaser::TLaserData Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState) = 0;
	virtual RoboCompLaser::LaserConfData Laser_getLaserConfData() = 0;
	virtual RoboCompLaser::TLaserData Laser_getLaserData() = 0;
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
