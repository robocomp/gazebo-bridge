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
#include "gazebo2robocompI.h"

Gazebo2RobocompI::Gazebo2RobocompI(GenericWorker *_worker)
{
	worker = _worker;
}


Gazebo2RobocompI::~Gazebo2RobocompI()
{
}


void Gazebo2RobocompI::createBoxEntity(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float size, const Ice::Current&)
{
	worker->Gazebo2Robocomp_createBoxEntity(name, position, orientation, size);
}

void Gazebo2RobocompI::createCapsuleEntity(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float length, float radius, const Ice::Current&)
{
	worker->Gazebo2Robocomp_createCapsuleEntity(name, position, orientation, length, radius);
}

void Gazebo2RobocompI::createCylinderEntity(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float length, float radius, const Ice::Current&)
{
	worker->Gazebo2Robocomp_createCylinderEntity(name, position, orientation, length, radius);
}

void Gazebo2RobocompI::createEntity(std::string sdf, const Ice::Current&)
{
	worker->Gazebo2Robocomp_createEntity(sdf);
}

void Gazebo2RobocompI::createSphereEntity(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, float radius, const Ice::Current&)
{
	worker->Gazebo2Robocomp_createSphereEntity(name, position, orientation, radius);
}

void Gazebo2RobocompI::removeEntity(std::string name, const Ice::Current&)
{
	worker->Gazebo2Robocomp_removeEntity(name);
}

void Gazebo2RobocompI::setEntityPose(std::string name, RoboCompGazebo2Robocomp::Vector3 position, RoboCompGazebo2Robocomp::Quaternion orientation, const Ice::Current&)
{
	worker->Gazebo2Robocomp_setEntityPose(name, position, orientation);
}

