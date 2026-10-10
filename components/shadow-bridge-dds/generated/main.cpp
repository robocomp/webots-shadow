/*
 *    Copyright (C) 2026 by YOUR NAME HERE
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


/** \mainpage RoboComp::Webots2Robocomp
 *
 * \section intro_sec Introduction
 *
 * The Webots2Robocomp component...
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
 * cd Webots2Robocomp
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
 * Just: "${PATH_TO_BINARY}/Webots2Robocomp --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */
#include <clocale>
#include <cstdlib>   // std::_Exit -- see the exit note at the end of main()
#include <cstdio>    // std::fflush
#include <signal.h>

// QT includes
#include <QtCore>
#include <QtWidgets>

// ICE includes
#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>
#include <Ice/Application.h>

#include <ConfigLoader/ConfigLoader.h>

#include <sigwatch/sigwatch.h>

#include "genericworker.h"
#include "../src/specificworker.h"

#include <camera360rgbI.h>
#include <camerargbdsimpleI.h>
#include <doorcontrolI.h>
#include <dronecontrolI.h>
#include <imuI.h>
#include <laserI.h>
#include <lidar3dI.h>
#include <omnirobotI.h>
#include <visualelementsI.h>
#include <webots2robocompI.h>
#include <joystickadapterI.h>

#include <Camera360RGB.h>
#include <CameraRGBDSimple.h>
#include <DoorControl.h>
#include <DroneControl.h>
#include <FullPoseEstimation.h>
#include <FullPoseEstimationPub.h>
#include <GenericBase.h>
#include <Gridder.h>
#include <IMU.h>
#include <JoystickAdapter.h>
#include <KinovaArm.h>
#include <Laser.h>
#include <Lidar3D.h>
#include <OmniRobot.h>
#include <Person.h>
#include <VisualElements.h>
#include <Webots2Robocomp.h>
#include "../src/media_descriptor_servant.h"

//#define USE_QTGUI

#define PROGRAM_NAME    "ShadowBridgeDDS"
#define SERVER_FULL_NAME   "RoboComp ShadowBridgeDDS::ShadowBridgeDDS"


template <typename InterfaceType>
void implement( const Ice::CommunicatorPtr& communicator,
                const std::string& endpointConfig,
                const std::string& adapterName,
                SpecificWorker* worker,
                int index)
{
    try
    {
        Ice::ObjectAdapterPtr adapter = communicator->createObjectAdapterWithEndpoints(adapterName, endpointConfig);
        auto servant = std::make_shared<InterfaceType>(worker, index);
        adapter->add(servant, Ice::stringToIdentity(adapterName));
        adapter->activate();
        std::cout << "[" << PROGRAM_NAME << "]: " << adapterName << " adapter created in port " << endpointConfig << std::endl;
    }
    catch (const IceStorm::TopicExists&)
    {
        std::cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for " << adapterName << std::endl;
    }
}

template <typename PubProxyType, typename PubProxyPointer>
void publish(const IceStorm::TopicManagerPrxPtr& topicManager,
             std::string name_topic,
             const std::string& topicBaseName,
             PubProxyPointer& pubProxy,
             const std::string& programName)
{
    if (!name_topic.empty()) name_topic += "/";
    name_topic += topicBaseName;

    std::cout << "[\033[1;36m" << programName << "\033[0m]: \033[32mINFO\033[0m Topic: " 
              << name_topic << " will be used for publication. \033[0m\n";

    std::shared_ptr<IceStorm::TopicPrx> topic;
    while (!topic)
    {
        try
        {
            topic = topicManager->retrieve(name_topic);
        }
        catch (const IceStorm::NoSuchTopic&)
        {
            std::cout << "\n\n[\033[1;36m" << programName << "\033[0m]: \033[1;33mWARNING\033[0m " 
                      << name_topic << " topic did not create. \033[32mCreating...\033[0m\n\n";
            try
            {
                topic = topicManager->create(name_topic);
            }
            catch (const IceStorm::TopicExists&)
            {
                std::cout << "[\033[31m" << programName << "\033[0m]: \033[1;33mWARNING\033[0m publishing the " 
                          << name_topic << " topic. It's possible that other component have created\n";
            }
        }
        catch(const IceUtil::NullHandleException&)
        {
            std::cout << "[\033[31m" << programName << "\033[0m]: \033[31mERROR\033[0m TopicManager is Null.\n";
            throw;
        }
    }
    auto publisher = topic->getPublisher()->ice_oneway();
    pubProxy = Ice::uncheckedCast<PubProxyType>(publisher);
}

template <typename SubInterfaceType>
void subscribe( const Ice::CommunicatorPtr& communicator,
                const IceStorm::TopicManagerPrxPtr& topicManager,
                const std::string& endpointConfig,
                std::string name_topic,
                const std::string& topicBaseName,
                SpecificWorker* worker,
                int index,
                std::shared_ptr<IceStorm::TopicPrx>& topic,
                Ice::ObjectPrxPtr& proxy, 
                const std::string& programName)
{
    try   
    {  
        if (!name_topic.empty()) name_topic += "/";
        name_topic += topicBaseName;

        Ice::ObjectAdapterPtr adapter = communicator->createObjectAdapterWithEndpoints(name_topic, endpointConfig);
        auto servant = std::make_shared<SubInterfaceType>(worker, index);
        auto proxy = adapter->addWithUUID(servant)->ice_oneway();

        std::cout << "[\033[1;36m" << programName << "\033[0m]: \033[32mINFO\033[0m Topic: " 
                  << name_topic << " will be used in subscription. \033[0m\n";

        std::shared_ptr<IceStorm::TopicPrx> topic;
        if(!topic)
        {
            try {
                topic = topicManager->create(name_topic);
                std::cout << "\n\n[\033[1;36m" << programName << "\033[0m]: \033[1;33mWARNING\033[0m " 
                          << name_topic << " topic did not create. \033[32mTopic created\033[0m\n\n";
            }
            catch (const IceStorm::TopicExists&) {
                try{
                    std::cout << "[\033[31m" << programName << "\033[0m]: \033[1;33mWARNING\033[0m Probably other client already opened the topic. \033[32mTrying to connect.\033[0m\n";
                    topic = topicManager->retrieve(name_topic);
                }
                catch(const IceStorm::NoSuchTopic&)
                {
                    std::cout << "[" << programName << "]: Topic doesn't exists and couldn't be created.\n";
                    return;
                }
            }
            catch(const IceUtil::NullHandleException&)
            {
                std::cout << "[\033[31m" << programName << "\033[0m]: \033[31mERROR\033[0m TopicManager is Null.\n";
                throw;
            }
            IceStorm::QoS qos;
            topic->subscribeAndGetPublisher(qos, proxy);
        }
        adapter->activate();
    }
    catch(const IceStorm::NoSuchTopic&)
    {
        std::cout << "[" << PROGRAM_NAME << "]: Error creating topic.\n";
    }
}


class Webots2Robocomp : public Ice::Application
{
public:
	Webots2Robocomp (QString configFile, QString prfx, bool startup_check) { 
		this->configFile = configFile.toStdString();
		this->prefix = prfx.toStdString();
		this->startup_check_flag=startup_check; 

		initialize();
		}

	Ice::InitializationData getInitializationDataIce();

private:
	void initialize();
	std::string prefix, configFile;
	ConfigLoader configLoader;
	TuplePrx tprx;
	bool startup_check_flag = false;

public:
	virtual int run(int, char*[]);
};

Ice::InitializationData Webots2Robocomp::getInitializationDataIce(){
        Ice::InitializationData initData;
        initData.properties = Ice::createProperties();

        // Forward EVERY Ice property the config declares, not a fixed list of four.
        //
        // Until 2026-09-11 this function copied exactly Ice.Warn.Connections, Ice.Trace.Network,
        // Ice.Trace.Protocol and Ice.MessageSizeMax, and nothing else. Every other Ice setting a
        // component config declared was therefore INERT: it was loaded, printed by printConfig(),
        // and never reached the communicator. A config block that is read and ignored looks exactly
        // like one that works, so this cost real time to find — in webots-bridge the config had
        // carried an [omnirobot.ThreadPool] block for months, with a comment explaining that it
        // gave base commands their own dispatch thread so they could not queue behind a camera.
        // They had always shared the default pool. It only became visible when some servants began
        // to block: four blocking calls took the single pool and the whole fleet serialised, an
        // untouched IMU stream falling from 113 Hz to 29 Hz while its data stayed 5 ms fresh.
        //
        // These properties are only read when the communicator is created, which is why this must
        // happen here and cannot be moved into SpecificWorker.
        //
        // "Ice.*" covers the run time's own settings; "*.ThreadPool.*" covers the per-object-adapter
        // pools, which are named after the adapter (see the implement<>() calls below) and so cannot
        // be listed in advance. Anything else in the config is the component's own business and is
        // deliberately not forwarded. A value that will not convert to a string is skipped rather
        // than aborting startup.
        for (const auto &key_view : this->configLoader.getKeys())
        {
            const std::string key{key_view};
            const bool is_ice_runtime  = key.rfind("Ice.", 0) == 0;
            const bool is_adapter_pool = key.find(".ThreadPool.") != std::string::npos;
            if (not (is_ice_runtime or is_adapter_pool))
                continue;
            try { initData.properties->setProperty(key, this->configLoader.get<std::string>(key)); }
            catch (...) { /* not string-convertible: leave Ice's default for this one */ }
        }
		return initData;
}

void Webots2Robocomp::initialize()
{
    this->configLoader.load(this->configFile);
	this->configLoader.printConfig();
	std::cout<<std::endl;
}

int Webots2Robocomp::run(int argc, char* argv[])
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

	RoboCompFullPoseEstimationPub::FullPoseEstimationPubPrxPtr fullposeestimationpub_proxy;
	std::shared_ptr<IceStorm::TopicPrx> joystickadapter_topic;
	Ice::ObjectPrxPtr joystickadapter;



	//Topic Manager code

	IceStorm::TopicManagerPrxPtr topicManager;
	try
	{
		topicManager = Ice::checkedCast<IceStorm::TopicManagerPrx>(communicator()->stringToProxy(configLoader.get<std::string>("Proxies.TopicManager")));
		if (!topicManager)
		{
		    std::cout << "[" << PROGRAM_NAME << "]: TopicManager.Proxy not defined in config file."<<std::endl;
		    std::cout << "	 Config line example: TopicManager.Proxy=IceStorm/TopicManager:default -p 9999"<<std::endl;
	        return EXIT_FAILURE;
		}
	}
	catch (const Ice::Exception &ex)
	{
		std::cout << "[" << PROGRAM_NAME << "]: Exception: 'rcnode' not running: " << ex << std::endl;
		return EXIT_FAILURE;
	}

	//Publish code
	publish<RoboCompFullPoseEstimationPub::FullPoseEstimationPubPrx, RoboCompFullPoseEstimationPub::FullPoseEstimationPubPrxPtr>(topicManager,
	                    configLoader.get<std::string>("Proxies.FullPoseEstimationPubPrefix"),
	                    "FullPoseEstimationPub", fullposeestimationpub_proxy, PROGRAM_NAME);

	tprx = std::make_tuple(fullposeestimationpub_proxy);
	SpecificWorker *worker = new SpecificWorker(this->configLoader, tprx, startup_check_flag);
	QObject::connect(worker, SIGNAL(kill()), &a, SLOT(quit()));

	try
	{

		//Implement code
		// Camera360RGB, CameraRGBDSimple and Lidar3D are NOT served: their only clients were the sim
		// drivers (ricoh_omni_dds, zed_camera, lidar3d_dds), which this bridge replaces by publishing
		// the media plane itself. See src/media_plane.h.
		implement<DoorControlI>(communicator(),
		                    configLoader.get<std::string>("Endpoints.DoorControl"), 
		                    "doorcontrol", worker,  0);
		implement<DroneControlI>(communicator(),
		                    configLoader.get<std::string>("Endpoints.DroneControl"), 
		                    "dronecontrol", worker,  0);
		implement<IMUI>(communicator(),
		                    configLoader.get<std::string>("Endpoints.IMU"), 
		                    "imu", worker,  0);
		implement<LaserI>(communicator(),
		                    configLoader.get<std::string>("Endpoints.Laser"), 
		                    "laser", worker,  0);
		implement<OmniRobotI>(communicator(),
		                    configLoader.get<std::string>("Endpoints.OmniRobot"), 
		                    "omnirobot", worker,  0);
		implement<VisualElementsI>(communicator(),
		                    configLoader.get<std::string>("Endpoints.VisualElements"), 
		                    "visualelements", worker,  0);
		implement<Webots2RobocompI>(communicator(),
		                    configLoader.get<std::string>("Endpoints.Webots2Robocomp"), 
		                    "webots2robocomp", worker,  0);

		// The five sensor drivers' MediaPlaneDDS endpoints, on THEIR ports, so robot_concept's
		// Proxies.MediaPlaneDDS..4 find the descriptors unchanged. One adapter per node.
		for (const auto& [stream, key] : {std::pair{"zed", "Endpoints.MediaPlaneZed"},
		                                  std::pair{"ricoh", "Endpoints.MediaPlaneRicoh"},
		                                  std::pair{"helios", "Endpoints.MediaPlaneHelios"},
		                                  std::pair{"bpearl", "Endpoints.MediaPlaneBpearl"},
		                                  std::pair{"imu", "Endpoints.MediaPlaneImu"}})
		{
			const std::string endpoint = configLoader.get<std::string>(key);
			auto adapter = communicator()->createObjectAdapterWithEndpoints(std::string("mediaplanedds_") + stream, endpoint);
			const std::string name = stream;
			adapter->add(std::make_shared<MediaDescriptorServant>([worker, name] { return worker->media_descriptor(name); }),
			             Ice::stringToIdentity("mediaplanedds"));
			adapter->activate();
			std::cout << "[" << PROGRAM_NAME << "]: MediaPlaneDDS '" << stream << "' descriptor served on " << endpoint << std::endl;
		}

		//Subscribe code
		subscribe<JoystickAdapterI>(communicator(),
		                    topicManager, configLoader.get<std::string>("Endpoints.JoystickAdapterTopic"),
						    configLoader.get<std::string>("Endpoints.JoystickAdapterPrefix"), "JoystickAdapter", worker,  0,
						    joystickadapter_topic, joystickadapter, PROGRAM_NAME);

		// Server adapter creation and publication
		std::cout << SERVER_FULL_NAME " started" << std::endl;

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
			joystickadapter_topic->unsubscribe(joystickadapter);

		}
		catch(const Ice::Exception& ex)
		{
			std::cout << "ERROR Unsubscribing" << ex.what()<<std::endl;
		}


		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;

		std::cerr << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << std::endl;
		std::cerr << ex;

	}
	#ifdef USE_QTGUI
		a.quit();
	#endif

	status = EXIT_SUCCESS;
	delete worker;
	return status;
}

int main(int argc, char* argv[])
{
	std::string arg;

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
				std::cout << "Startup check = True"<< std::endl;
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
	Webots2Robocomp app(configFile, prefix, startup_check_flag);

	const int status = app.main(argc, argv, app.getInitializationDataIce());

	// Leave the process HERE, before C++ static destruction and the dynamic linker's finalizers.
	//
	// libIce 3.7 destroys its own globals in the wrong order. At exit _dl_call_fini runs libIce's
	// finalizer, a static object inside it calls
	// IceInternal::FactoryTable::removeExceptionFactory(), and that locks a mutex an EARLIER
	// finalizer has already destroyed. pthread_mutex_lock returns EINVAL, IceUtil::Mutex::lock()
	// (IceUtil/Mutex.h:295) throws ThreadSyscallException out of a destructor, and the process
	// dies on EVERY otherwise-clean shutdown with
	//     terminate called after throwing an instance of 'IceUtil::ThreadSyscallException'
	//     what(): include/IceUtil/Mutex.h:295: syscall exception: Invalid argument
	// and SIGABRT (exit 134) plus a core dump. The whole fault is inside libIce's own finalizer
	// chain, so no link order or teardown ordering on the component's side can avoid it.
	//
	// Nothing the component owns is skipped: run() above ends with `delete worker`, so
	// SpecificWorker and everything it holds -- DSR graphs, their FastDDS DomainParticipants, the
	// owned-node cleanup in the destructor -- have already run, and Qt's application object is
	// gone too. Only third-party library static teardown is bypassed. Verified against a DSR
	// agent holding two DomainParticipants: no /dev/shm FastDDS segments leaked (99 before, 99
	// after), clean exit code 0 on both SIGTERM and SIGINT, no core dump.
	//
	// std::_Exit does not flush stdio, so flush first or the tail of the log is lost.
	std::cout.flush();
	std::cerr.flush();
	std::fflush(nullptr);
	std::_Exit(status);
}
