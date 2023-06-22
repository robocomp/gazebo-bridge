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
#ifndef GAZEBO2ROBOCOMP_H
#define GAZEBO2ROBOCOMP_H

// Ice includes
#include <Ice/Ice.h>
#include <Gazebo2Robocomp.h>

#include <config.h>
#include "genericworker.h"


class Gazebo2RobocompI : public virtual RoboCompGazebo2Robocomp::Gazebo2Robocomp
{
public:
	Gazebo2RobocompI(GenericWorker *_worker);
	~Gazebo2RobocompI();

	void createBoxEntity(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float size, const Ice::Current&);
	void createCapsuleEntity(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float length, float radius, const Ice::Current&);
	void createCylinderEntity(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float length, float radius, const Ice::Current&);
	void createEntity(std::string sdf, const Ice::Current&);
	void createHumanEntity(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, const Ice::Current&);
	void createRandomBoxEntity(RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float size, const Ice::Current&);
	void createRandomCapsuleEntity(RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float length, float radius, const Ice::Current&);
	void createRandomCylinderEntity(RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float length, float radius, const Ice::Current&);
	void createRandomHumanEntity(RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, const Ice::Current&);
	void createRandomSphereEntity(RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float radius, const Ice::Current&);
	void createSphereEntity(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float radius, const Ice::Current&);
	RoboCompGazebo2Robocomp::Vector3 getWorldPosition(std::string name, const Ice::Current&);
	void removeEntity(std::string name, const Ice::Current&);
	void setEntityPose(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, const Ice::Current&);
	void setLinearVelocity(std::string name, RoboCompGazebo2Robocomp::Vector3 velocity, const Ice::Current&);

private:

	GenericWorker *worker;

};

#endif
