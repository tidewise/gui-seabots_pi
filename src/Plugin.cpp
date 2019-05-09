#include "Plugin.hpp"
#include "Images.hpp"
#include "OCPNInterfaceImpl.hpp"
#include <iostream>

#include <std/typekit/Plugin.hpp>
#include <std/transports/corba/TransportPlugin.hpp>
#include <std/transports/typelib/TransportPlugin.hpp>
#include <std/transports/mqueue/TransportPlugin.hpp>

#include <base/typekit/Plugin.hpp>
#include <base/transports/corba/TransportPlugin.hpp>
#include <base/transports/typelib/TransportPlugin.hpp>
#include <base/transports/mqueue/TransportPlugin.hpp>

#include <gps_base/typekit/Plugin.hpp>
#include <gps_base/transports/corba/TransportPlugin.hpp>
#include <gps_base/transports/typelib/TransportPlugin.hpp>
#include <gps_base/transports/mqueue/TransportPlugin.hpp>

#include <seabots_pi/Task.hpp>
#include <seabots_pi/typekit/Plugin.hpp>
#include <seabots_pi/transports/corba/TransportPlugin.hpp>
#include <seabots_pi/transports/typelib/TransportPlugin.hpp>
#include <seabots_pi/transports/mqueue/TransportPlugin.hpp>

#include <logger/Logger.hpp>
#include <logger/typekit/Plugin.hpp>
#include <logger/transports/corba/TransportPlugin.hpp>
#include <logger/transports/typelib/TransportPlugin.hpp>
#include <logger/transports/mqueue/TransportPlugin.hpp>

#include <rtt/Activity.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/base/ActivityInterface.hpp>
#include <rtt/extras/SlaveActivity.hpp>
#include <rtt/transports/corba/ApplicationServer.hpp>
#include <rtt/transports/corba/TaskContextServer.hpp>
#include <rtt/transports/corba/CorbaDispatcher.hpp>

#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/plugin/PluginLoader.hpp>

using namespace seabots_pi;

const char* Plugin::NAME = "Seabots";
const char* Plugin::DESCRIPTION_SHORT = "Seabots interface to OpenCPN";
const char* Plugin::DESCRIPTION_LONG = "OpenCPN/Seabots interface, to "
    "use OpenCPN for route planning in a Seabots system";


Plugin::orogenTaskTimer::orogenTaskTimer(Plugin& plugin)
    : plugin(plugin) {}
void Plugin::orogenTaskTimer::Notify()
{
    plugin.executeTasks();
}

Plugin::Plugin(void* pptr)
    : opencpn_plugin_116(pptr)
    , mTimer(*this)
{
    initialize_images();
}

int Plugin::Init() {
    static char const* argv[] = { "seabots_pi" };
    RTT::corba::ApplicationServer::InitOrb(1, const_cast<char**>(argv));
    RTT::corba::TaskContextServer::ThreadOrb(ORO_SCHED_OTHER, RTT::os::LowestPriority, 0);

    // Import typekits to allow RTT convert the types used by the components
    RTT::types::TypekitRepository::Import(new orogen_typekits::stdTypekitPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::stdCorbaTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::stdMQueueTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::stdTypelibTransportPlugin);

    RTT::types::TypekitRepository::Import(new orogen_typekits::baseTypekitPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::baseCorbaTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::baseMQueueTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::baseTypelibTransportPlugin);

    RTT::types::TypekitRepository::Import(new orogen_typekits::loggerTypekitPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::loggerCorbaTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::loggerMQueueTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::loggerTypelibTransportPlugin);

    RTT::types::TypekitRepository::Import(new orogen_typekits::gps_baseTypekitPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::gps_baseCorbaTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::gps_baseMQueueTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::gps_baseTypelibTransportPlugin);

    RTT::types::TypekitRepository::Import(new orogen_typekits::seabots_piTypekitPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::seabots_piCorbaTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::seabots_piMQueueTransportPlugin);
    RTT::types::TypekitRepository::Import(new orogen_typekits::seabots_piTypelibTransportPlugin);

    // Create the logger component and start the activity
    logger::Logger* logger_task = new logger::Logger();
    logger_task->provides()->setName("seabots_pi_Logger");
    // RTT::Activity runs the task in separate thread, unlike the default of a
    // slave activity
    RTT::Activity* logger_activity = new RTT::Activity( logger_task->engine() );
    setupTaskActivity(logger_task, logger_activity);

    Task* main_task = new Task("seabots_pi");
    mInterface = new OCPNInterfaceImpl();
    main_task->setOCPNInterface(mInterface);
    setupTaskActivity(main_task);

    mTimer.Start(UPDATE_PERIOD_MS, wxTIMER_CONTINUOUS); // start timer
    return 0;
}

void Plugin::setupTaskActivity(
    RTT::TaskContext* task, RTT::base::ActivityInterface* activity
)
{
    // Export the component interface on CORBA to Ruby access the component
    RTT::corba::TaskContextServer::Create( task );
    task->addConstant<int>("CorbaDispatcherScheduler", ORO_SCHED_OTHER);
    task->addConstant<int>("CorbaDispatcherPriority", RTT::os::LowestPriority);

    // Create and start sequential task activities
    if (!activity) {
        activity = new RTT::extras::SlaveActivity(task->engine());
    }
    activity->start();
    activities.push_back(activity);
    tasks.push_back(task);
}

void Plugin::executeTasks()
{
    for (auto& task : tasks) {
        task->getActivity()->execute();
    }
}

bool Plugin::DeInit() {
    mTimer.Stop();

    // Deregister the CORBA stuff
    RTT::corba::TaskContextServer::CleanupServers();
    RTT::corba::CorbaDispatcher::ReleaseAll();

    // Delete pointers to activity
    for(Activities::iterator activity_it = activities.begin();
            activity_it != activities.end(); ++activity_it)
    {
        delete *activity_it;
    }

    // Delete pointers to tasks
    for(Tasks::iterator task_it = tasks.begin();
            task_it != tasks.end(); ++task_it)
    {
        delete *task_it;
    }

    RTT::corba::TaskContextServer::ShutdownOrb();
    RTT::corba::TaskContextServer::DestroyOrb();
    return true;
}

wxBitmap* Plugin::GetPlugInBitmap() {
    return _img_seabots;
}

extern "C" DECL_EXP opencpn_plugin* create_pi(void *ppimgr)
{
    return new seabots_pi::Plugin(ppimgr);
}

extern "C" DECL_EXP void destroy_pi(opencpn_plugin* p)
{
    delete p;
}
