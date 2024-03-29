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


/** \mainpage RoboComp::Gazebo2Robocomp
 *
 * \section intro_sec Introduction
 *
 * The Gazebo2Robocomp component...
 *
 * \section interface_sec Interface
 *
 * interface...
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * ...
 *
 * \subsection install2_ssec Compile and install
 * cd Gazebo2Robocomp
 * <br>
 * cmake . && make
 * <br>
 * To install:
 * <br>
 * sudo make install
 *
 * \section guide_sec User guide
 *
 * \subsection config_ssec Configuration file
 *
 * <p>
 * The configuration file etc/config...
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/Gazebo2Robocomp --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */
#include <signal.h>

// QT includes
#include <QtCore>
#include <QtWidgets>

// ICE includes
#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>
#include <Ice/Application.h>

#include <rapplication/rapplication.h>
#include <sigwatch/sigwatch.h>
#include <qlog/qlog.h>

#include "config.h"
#include "genericmonitor.h"
#include "genericworker.h"
#include "specificworker.h"
#include "specificmonitor.h"
#include "commonbehaviorI.h"

#include <camera360rgbI.h>
#include <camerargbdsimpleI.h>
#include <differentialrobotI.h>
#include <gazebo2robocompI.h>
#include <imuI.h>
#include <jointmotorsimpleI.h>
#include <laserI.h>
#include <lidar3dI.h>
#include <omnirobotI.h>
#include <joystickadapterI.h>

#include <GenericBase.h>



class Gazebo2Robocomp : public RoboComp::Application
{
public:
	Gazebo2Robocomp (QString prfx, bool startup_check) { prefix = prfx.toStdString(); this->startup_check_flag=startup_check; }
private:
	void initialize();
	std::string prefix;
	TuplePrx tprx;
	bool startup_check_flag = false;

public:
	virtual int run(int, char*[]);
};

void ::Gazebo2Robocomp::initialize()
{
	// Config file properties read example
	// configGetString( PROPERTY_NAME_1, property1_holder, PROPERTY_1_DEFAULT_VALUE );
	// configGetInt( PROPERTY_NAME_2, property1_holder, PROPERTY_2_DEFAULT_VALUE );
}

int ::Gazebo2Robocomp::run(int argc, char* argv[])
{
#ifdef USE_QTGUI
	QApplication a(argc, argv);  // GUI application
#else
	QCoreApplication a(argc, argv);  // NON-GUI application
#endif


	sigset_t sigs;
	sigemptyset(&sigs);
	sigaddset(&sigs, SIGHUP);
	sigaddset(&sigs, SIGINT);
	sigaddset(&sigs, SIGTERM);
	sigprocmask(SIG_UNBLOCK, &sigs, 0);

	UnixSignalWatcher sigwatch;
	sigwatch.watchForSignal(SIGINT);
	sigwatch.watchForSignal(SIGTERM);
	QObject::connect(&sigwatch, SIGNAL(unixSignal(int)), &a, SLOT(quit()));

	int status=EXIT_SUCCESS;


	string proxy, tmp;
	initialize();

	IceStorm::TopicManagerPrxPtr topicManager;
	try
	{
		topicManager = topicManager = Ice::checkedCast<IceStorm::TopicManagerPrx>(communicator()->propertyToProxy("TopicManager.Proxy"));
		if (!topicManager)
		{
		    cout << "[" << PROGRAM_NAME << "]: TopicManager.Proxy not defined in config file."<<endl;
		    cout << "	 Config line example: TopicManager.Proxy=IceStorm/TopicManager:default -p 9999"<<endl;
	        return EXIT_FAILURE;
		}
	}
	catch (const Ice::Exception &ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: 'rcnode' not running: " << ex << endl;
		return EXIT_FAILURE;
	}

	tprx = std::tuple<>();
	SpecificWorker *worker = new SpecificWorker(tprx, startup_check_flag);
	//Monitor thread
	SpecificMonitor *monitor = new SpecificMonitor(worker,communicator());
	QObject::connect(monitor, SIGNAL(kill()), &a, SLOT(quit()));
	QObject::connect(worker, SIGNAL(kill()), &a, SLOT(quit()));
	monitor->start();

	if ( !monitor->isRunning() )
		return status;

	while (!monitor->ready)
	{
		usleep(10000);
	}

	try
	{
		try {
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "CommonBehavior.Endpoints", tmp, "")) {
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CommonBehavior\n";
			}
			Ice::ObjectAdapterPtr adapterCommonBehavior = communicator()->createObjectAdapterWithEndpoints("commonbehavior", tmp);
			auto commonbehaviorI = std::make_shared<CommonBehaviorI>(monitor);
			adapterCommonBehavior->add(commonbehaviorI, Ice::stringToIdentity("commonbehavior"));
			adapterCommonBehavior->activate();
		}
		catch(const Ice::Exception& ex)
		{
			status = EXIT_FAILURE;

			cout << "[" << PROGRAM_NAME << "]: Exception raised while creating CommonBehavior adapter: " << endl;
			cout << ex;

		}



		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "Camera360RGB.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy Camera360RGB";
			}
			Ice::ObjectAdapterPtr adapterCamera360RGB = communicator()->createObjectAdapterWithEndpoints("Camera360RGB", tmp);
			auto camera360rgb = std::make_shared<Camera360RGBI>(worker);
			adapterCamera360RGB->add(camera360rgb, Ice::stringToIdentity("camera360rgb"));
			adapterCamera360RGB->activate();
			cout << "[" << PROGRAM_NAME << "]: Camera360RGB adapter created in port " << tmp << endl;
		}
		catch (const IceStorm::TopicExists&){
			cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for Camera360RGB\n";
		}


		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "CameraRGBDSimple.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CameraRGBDSimple";
			}
			Ice::ObjectAdapterPtr adapterCameraRGBDSimple = communicator()->createObjectAdapterWithEndpoints("CameraRGBDSimple", tmp);
			auto camerargbdsimple = std::make_shared<CameraRGBDSimpleI>(worker);
			adapterCameraRGBDSimple->add(camerargbdsimple, Ice::stringToIdentity("camerargbdsimple"));
			adapterCameraRGBDSimple->activate();
			cout << "[" << PROGRAM_NAME << "]: CameraRGBDSimple adapter created in port " << tmp << endl;
		}
		catch (const IceStorm::TopicExists&){
			cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for CameraRGBDSimple\n";
		}


		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "DifferentialRobot.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy DifferentialRobot";
			}
			Ice::ObjectAdapterPtr adapterDifferentialRobot = communicator()->createObjectAdapterWithEndpoints("DifferentialRobot", tmp);
			auto differentialrobot = std::make_shared<DifferentialRobotI>(worker);
			adapterDifferentialRobot->add(differentialrobot, Ice::stringToIdentity("differentialrobot"));
			adapterDifferentialRobot->activate();
			cout << "[" << PROGRAM_NAME << "]: DifferentialRobot adapter created in port " << tmp << endl;
		}
		catch (const IceStorm::TopicExists&){
			cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for DifferentialRobot\n";
		}


		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "Gazebo2Robocomp.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy Gazebo2Robocomp";
			}
			Ice::ObjectAdapterPtr adapterGazebo2Robocomp = communicator()->createObjectAdapterWithEndpoints("Gazebo2Robocomp", tmp);
			auto gazebo2robocomp = std::make_shared<Gazebo2RobocompI>(worker);
			adapterGazebo2Robocomp->add(gazebo2robocomp, Ice::stringToIdentity("gazebo2robocomp"));
			adapterGazebo2Robocomp->activate();
			cout << "[" << PROGRAM_NAME << "]: Gazebo2Robocomp adapter created in port " << tmp << endl;
		}
		catch (const IceStorm::TopicExists&){
			cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for Gazebo2Robocomp\n";
		}


		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "IMU.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy IMU";
			}
			Ice::ObjectAdapterPtr adapterIMU = communicator()->createObjectAdapterWithEndpoints("IMU", tmp);
			auto imu = std::make_shared<IMUI>(worker);
			adapterIMU->add(imu, Ice::stringToIdentity("imu"));
			adapterIMU->activate();
			cout << "[" << PROGRAM_NAME << "]: IMU adapter created in port " << tmp << endl;
		}
		catch (const IceStorm::TopicExists&){
			cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for IMU\n";
		}


		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "JointMotorSimple.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy JointMotorSimple";
			}
			Ice::ObjectAdapterPtr adapterJointMotorSimple = communicator()->createObjectAdapterWithEndpoints("JointMotorSimple", tmp);
			auto jointmotorsimple = std::make_shared<JointMotorSimpleI>(worker);
			adapterJointMotorSimple->add(jointmotorsimple, Ice::stringToIdentity("jointmotorsimple"));
			adapterJointMotorSimple->activate();
			cout << "[" << PROGRAM_NAME << "]: JointMotorSimple adapter created in port " << tmp << endl;
		}
		catch (const IceStorm::TopicExists&){
			cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for JointMotorSimple\n";
		}


		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "Laser.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy Laser";
			}
			Ice::ObjectAdapterPtr adapterLaser = communicator()->createObjectAdapterWithEndpoints("Laser", tmp);
			auto laser = std::make_shared<LaserI>(worker);
			adapterLaser->add(laser, Ice::stringToIdentity("laser"));
			adapterLaser->activate();
			cout << "[" << PROGRAM_NAME << "]: Laser adapter created in port " << tmp << endl;
		}
		catch (const IceStorm::TopicExists&){
			cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for Laser\n";
		}


		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "Lidar3D.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy Lidar3D";
			}
			Ice::ObjectAdapterPtr adapterLidar3D = communicator()->createObjectAdapterWithEndpoints("Lidar3D", tmp);
			auto lidar3d = std::make_shared<Lidar3DI>(worker);
			adapterLidar3D->add(lidar3d, Ice::stringToIdentity("lidar3d"));
			adapterLidar3D->activate();
			cout << "[" << PROGRAM_NAME << "]: Lidar3D adapter created in port " << tmp << endl;
		}
		catch (const IceStorm::TopicExists&){
			cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for Lidar3D\n";
		}


		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "OmniRobot.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy OmniRobot";
			}
			Ice::ObjectAdapterPtr adapterOmniRobot = communicator()->createObjectAdapterWithEndpoints("OmniRobot", tmp);
			auto omnirobot = std::make_shared<OmniRobotI>(worker);
			adapterOmniRobot->add(omnirobot, Ice::stringToIdentity("omnirobot"));
			adapterOmniRobot->activate();
			cout << "[" << PROGRAM_NAME << "]: OmniRobot adapter created in port " << tmp << endl;
		}
		catch (const IceStorm::TopicExists&){
			cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for OmniRobot\n";
		}


		// Server adapter creation and publication
		std::shared_ptr<IceStorm::TopicPrx> joystickadapter_topic;
		Ice::ObjectPrxPtr joystickadapter;
		try
		{
			if (not GenericMonitor::configGetString(communicator(), prefix, "JoystickAdapterTopic.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy JoystickAdapterProxy";
			}
			Ice::ObjectAdapterPtr JoystickAdapter_adapter = communicator()->createObjectAdapterWithEndpoints("joystickadapter", tmp);
			RoboCompJoystickAdapter::JoystickAdapterPtr joystickadapterI_ =  std::make_shared <JoystickAdapterI>(worker);
			auto joystickadapter = JoystickAdapter_adapter->addWithUUID(joystickadapterI_)->ice_oneway();
			if(!joystickadapter_topic)
			{
				try {
					joystickadapter_topic = topicManager->create("JoystickAdapter");
				}
				catch (const IceStorm::TopicExists&) {
					//Another client created the topic
					try{
						cout << "[" << PROGRAM_NAME << "]: Probably other client already opened the topic. Trying to connect.\n";
						joystickadapter_topic = topicManager->retrieve("JoystickAdapter");
					}
					catch(const IceStorm::NoSuchTopic&)
					{
						cout << "[" << PROGRAM_NAME << "]: Topic doesn't exists and couldn't be created.\n";
						//Error. Topic does not exist
					}
				}
				catch(const IceUtil::NullHandleException&)
				{
					cout << "[" << PROGRAM_NAME << "]: ERROR TopicManager is Null. Check that your configuration file contains an entry like:\n"<<
					"\t\tTopicManager.Proxy=IceStorm/TopicManager:default -p <port>\n";
					return EXIT_FAILURE;
				}
				IceStorm::QoS qos;
				joystickadapter_topic->subscribeAndGetPublisher(qos, joystickadapter);
			}
			JoystickAdapter_adapter->activate();
		}
		catch(const IceStorm::NoSuchTopic&)
		{
			cout << "[" << PROGRAM_NAME << "]: Error creating JoystickAdapter topic.\n";
			//Error. Topic does not exist
		}


		// Server adapter creation and publication
		cout << SERVER_FULL_NAME " started" << endl;

		// User defined QtGui elements ( main window, dialogs, etc )

		#ifdef USE_QTGUI
			//ignoreInterrupt(); // Uncomment if you want the component to ignore console SIGINT signal (ctrl+c).
			a.setQuitOnLastWindowClosed( true );
		#endif
		// Run QT Application Event Loop
		a.exec();

		try
		{
			std::cout << "Unsubscribing topic: joystickadapter " <<std::endl;
			joystickadapter_topic->unsubscribe( joystickadapter );
		}
		catch(const Ice::Exception& ex)
		{
			std::cout << "ERROR Unsubscribing topic: joystickadapter " << ex.what()<<std::endl;
		}


		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;

		cout << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << endl;
		cout << ex;

	}
	#ifdef USE_QTGUI
		a.quit();
	#endif

	status = EXIT_SUCCESS;
	monitor->terminate();
	monitor->wait();
	delete worker;
	delete monitor;
	return status;
}

int main(int argc, char* argv[])
{
	string arg;

	// Set config file
	QString configFile("etc/config");
	bool startup_check_flag = false;
	QString prefix("");
	if (argc > 1)
	{

		// Search in argument list for arguments
		QString startup = QString("--startup-check");
		QString initIC = QString("--Ice.Config=");
		QString prfx = QString("--prefix=");
		for (int i = 0; i < argc; ++i)
		{
			arg = argv[i];
			if (arg.find(startup.toStdString(), 0) != std::string::npos)
			{
				startup_check_flag = true;
				cout << "Startup check = True"<< endl;
			}
			else if (arg.find(prfx.toStdString(), 0) != std::string::npos)
			{
				prefix = QString::fromStdString(arg).remove(0, prfx.size());
				if (prefix.size()>0)
					prefix += QString(".");
				printf("Configuration prefix: <%s>\n", prefix.toStdString().c_str());
			}
			else if (arg.find(initIC.toStdString(), 0) != std::string::npos)
			{
				configFile = QString::fromStdString(arg).remove(0, initIC.size());
				qDebug()<<__LINE__<<"Starting with config file:"<<configFile;
			}
			else if (i==1 and argc==2 and arg.find("--", 0) == std::string::npos)
			{
				configFile = QString::fromStdString(arg);
				qDebug()<<__LINE__<<QString::fromStdString(arg)<<argc<<arg.find("--", 0)<<"Starting with config file:"<<configFile;
			}
		}

	}
	::Gazebo2Robocomp app(prefix, startup_check_flag);

	return app.main(argc, argv, configFile.toLocal8Bit().data());
}
