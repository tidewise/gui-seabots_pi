#include "Plugin.hpp"
#include "OCPNInterfaceImpl.hpp"
#include "Paths.hpp"
#include <iostream>

#include <wx/filename.h>
#include <wx/file.h>
#include <GL/gl.h>
#include <GL/glext.h>

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
using namespace std;

#define GL_CHECK_ERRORS() glCheckErrors(__FILE__, __LINE__, true);

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
}

int Plugin::Init() {
    loadSVGs();

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
    mInterface = new OCPNInterfaceImpl(*main_task);
    main_task->setOCPNInterface(mInterface);
    setupTaskActivity(main_task);

    setupToolbar();

    // Start timer
    mTimer.Start(UPDATE_PERIOD_MS, wxTIMER_CONTINUOUS);
    return (
        WANTS_TOOLBAR_CALLBACK |
        WANTS_DYNAMIC_OPENGL_OVERLAY_CALLBACK | WANTS_OPENGL_OVERLAY_CALLBACK | WANTS_OVERLAY_CALLBACK |
        INSTALLS_TOOLBAR_TOOL
    );
}

void Plugin::setupToolbar()
{
    mPlanRouteTool = InsertPlugInToolSVG(
        _T( "Plan Route" ),
        mPlanRouteSVG, mPlanRouteSVG, mPlanRouteSVGToggled, wxITEM_NORMAL,
        _("Plan Route"), _T( "" ), NULL, TOOL_PLAN_ROOT_POSITION, 0, this);
    mExecuteRouteTool = InsertPlugInToolSVG(
        _T( "Execute Route" ),
        mExecuteRouteSVG, mExecuteRouteSVG, mExecuteRouteSVGToggled, wxITEM_CHECK,
        _("Execute Route"), _T( "" ), NULL, TOOL_EXECUTE_ROOT_POSITION, 0, this);
}

void Plugin::glAllocateTrajectoryArrays(int neededSize) {
    int currentSize = mPlannedTrajectoryVBOs.size();
    if (currentSize >= neededSize)
        return;

    mPlannedTrajectoryVAOs.resize(neededSize, 0);
    mPlannedTrajectoryVBOs.resize(neededSize, 0);
    glGenVertexArrays(neededSize - currentSize, &mPlannedTrajectoryVAOs[currentSize]);
    glGenBuffers(neededSize - currentSize, &mPlannedTrajectoryVBOs[currentSize]);

    for (int i = currentSize; i < neededSize; ++i) {
        glBindVertexArray(mPlannedTrajectoryVAOs[i]);
        glBindBuffer(GL_ARRAY_BUFFER, mPlannedTrajectoryVBOs[i]);
        glVertexAttribPointer(mTrajectoryPointPositionAttribute, 2, GL_FLOAT, GL_FALSE, 0, 0);
        glEnableVertexAttribArray(mTrajectoryPointPositionAttribute);
    }
}

void Plugin::glUploadSampledTrajectory(
    int index, OCPNInterfaceImpl::SampledTrajectory const& trajectory, PlugIn_ViewPort* vp)
{
    std::vector<float> coords;
    coords.reserve(trajectory.points.size() * 2);
    for (auto ll : trajectory.points) {
        wxPoint pp;
        GetCanvasPixLL(vp, &pp, ll.latitude_deg, ll.longitude_deg);
        coords.push_back(pp.x);
        coords.push_back(pp.y);
    }

    glNamedBufferData(mPlannedTrajectoryVBOs[index],
        sizeof(coords[0]) * coords.size(),
        coords.data(), GL_STATIC_DRAW);
}

struct GLStatePush
{
    GLint currentProgram;
    GLint arrayBuffer;

    GLStatePush() {
        glGetIntegerv(GL_CURRENT_PROGRAM, &currentProgram);
        glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &arrayBuffer);
    }

    ~GLStatePush() {
        glUseProgram(currentProgram);
        glBindBuffer(GL_ARRAY_BUFFER, arrayBuffer);
        glBindVertexArray(0);
    }
};

bool Plugin::RenderGLOverlayMultiCanvas(wxGLContext *pcontext, PlugIn_ViewPort *vp, int canvasIndex)
{
    glCheckErrors(__FILE__, __LINE__, false);
    glLoadPrograms();

    GLStatePush state;
    float viewTransform[16] = {
        2.0f / vp->pix_width, 0, 0, -1,
        0, -2.0f / vp->pix_height, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1
    };

    auto const& current = mInterface->getCurrentPlanningResult();
    auto const& trajectories = current.sampled;
    if (!trajectories.empty()) {
        glAllocateTrajectoryArrays(trajectories.size());
        for (unsigned int i = 0; i < trajectories.size(); ++i)
        {
            glUploadSampledTrajectory(i, trajectories[i], vp);
            GL_CHECK_ERRORS();
        }

        glUseProgram(mTrajectoryGLProgramID);
        glUniformMatrix4fv(mTrajectoryViewTransformUniform, 1, true, viewTransform);

        for (unsigned int i = 0; i < trajectories.size(); ++i) {
            glBindVertexArray(mPlannedTrajectoryVAOs[i]);
            glDrawArrays(GL_LINE_STRIP, 0, trajectories[i].points.size());
            GL_CHECK_ERRORS();
        }
    }

    return true;
}

wxString Plugin::readDataFile(wxString const& name)
{
    wxFileName fn;
    fn.SetPath(paths::DATA_PATH);
    fn.SetFullName(name);
    wxFile file;
    file.Open(fn.GetFullPath());

    wxString contents;
    if (!file.ReadAll(&contents)) {
        throw std::runtime_error("failed to read file " + name);
    }
    return contents;
}

void Plugin::glLoadPrograms()
{
    if (!mTrajectoryGLProgramID) {
        mTrajectoryGLProgramID = glLoadProgram("trajectory");
        mTrajectoryPointPositionAttribute =
            glGetAttribLocation(mTrajectoryGLProgramID, "position");
        mTrajectoryViewTransformUniform =
            glGetUniformLocation(mTrajectoryGLProgramID, "viewTransform");
    }
}

GLuint Plugin::glLoadProgram(wxString name)
{
    auto vertex = glLoadShader(name + ".vert", GL_VERTEX_SHADER);
    auto frag = glLoadShader(name + ".frag", GL_FRAGMENT_SHADER);

    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertex);
    glAttachShader(shaderProgram, frag);

    // Flag the shaders for deletion
    glDeleteShader(vertex);
    glDeleteShader(frag);

    // Link and use the program
    glLinkProgram(shaderProgram);

    GL_CHECK_ERRORS();
    return shaderProgram;
}

GLuint Plugin::glLoadShader(wxString name, GLenum shaderType)
{
    wxString content = readDataFile(name);
    const char* code = content.GetData();
    GLuint shader = glCreateShader(shaderType);
    glShaderSource(shader, 1, &code, NULL);
    glCompileShader(shader);

    // Check the result of the compilation
    GLint test;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &test);
    if(!test) {
        std::cerr << "Shader compilation failed with this message:" << std::endl;
        std::vector<char> compilation_log(512);
        glGetShaderInfoLog(shader, compilation_log.size(), NULL, &compilation_log[0]);
        std::cerr << &compilation_log[0] << std::endl;
        throw std::runtime_error("failed to load shader " + name);
    }
    return shader;
}

void Plugin::glCheckErrors(const char *file, int line, bool throwOnError) {
    bool hasErrors = false;
    for(GLenum err = glGetError(); err != GL_NO_ERROR; err = glGetError()) {
        string error;

        switch(err) {
            case GL_INVALID_OPERATION:      error="INVALID_OPERATION";      break;
            case GL_INVALID_ENUM:           error="INVALID_ENUM";           break;
            case GL_INVALID_VALUE:          error="INVALID_VALUE";          break;
            case GL_OUT_OF_MEMORY:          error="OUT_OF_MEMORY";          break;
            case GL_INVALID_FRAMEBUFFER_OPERATION:  error="INVALID_FRAMEBUFFER_OPERATION";  break;
        }

        cerr << "GL_" << error.c_str() << " - " << file << ":" << line<<endl;
        hasErrors = true;
    }
    if (hasErrors && throwOnError) {
        throw std::runtime_error("OpenGL errors detected");
    }
}

void Plugin::loadSVGs()
{
    wxFileName fn;
    fn.SetPath(paths::DATA_PATH);

    fn.SetFullName("plan_route.svg");
    mPlanRouteSVG = fn.GetFullPath();
    fn.SetFullName("plan_route_toggled.svg");
    mPlanRouteSVGToggled = fn.GetFullPath();
    fn.SetFullName("execute_route.svg");
    mExecuteRouteSVG = fn.GetFullPath();
    fn.SetFullName("execute_route_toggled.svg");
    mExecuteRouteSVGToggled = fn.GetFullPath();
    fn.SetFullName("seabots.svg");
    mSeabotsSVG = fn.GetFullPath();
    mSeabotsBitmap = GetBitmapFromSVGFile(mSeabotsSVG, 128, 128);
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

int Plugin::GetToolbarToolCount(void)
{
      return 1;
}

void Plugin::OnToolbarToolCallback(int id)
{
    if (id == mPlanRouteTool) {
        bool result = planCurrentRoute();
        SetToolbarItemState(mExecuteRouteTool, result);
    }
    else if (id == mExecuteRouteTool) {
        bool result = executeCurrentTrajectories();
        SetToolbarItemState(mExecuteRouteTool, result);
    }
}

bool Plugin::planCurrentRoute()
{
    wxString routeID = GetActiveRouteGUID_Plugin();
    if (routeID.IsEmpty()) {
        wxMessageBox("No route selected");
        return false;
    }

    auto route = GetRoute_Plugin(routeID);
    mInterface->pushRoute(*route);
    return true;
}

bool Plugin::executeCurrentTrajectories()
{
    wxString routeID = GetActiveRouteGUID_Plugin();
    if (!mInterface->hasValidPlanningResultForRoute(routeID.ToStdString())) {
        wxMessageBox("No planned trajectories for the current route");
        return false;
    }
    mInterface->executeCurrentTrajectories(routeID.ToStdString());
    return true;
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
    activities.clear();

    // Delete pointers to tasks
    for(Tasks::iterator task_it = tasks.begin();
            task_it != tasks.end(); ++task_it)
    {
        delete *task_it;
    }
    tasks.clear();

    RTT::corba::TaskContextServer::ShutdownOrb();
    RTT::corba::TaskContextServer::DestroyOrb();
    delete mInterface;
    mInterface = nullptr;
    return true;
}

wxBitmap* Plugin::GetPlugInBitmap() {
    return &mSeabotsBitmap;
}

extern "C" DECL_EXP opencpn_plugin* create_pi(void *ppimgr)
{
    return new seabots_pi::Plugin(ppimgr);
}

extern "C" DECL_EXP void destroy_pi(opencpn_plugin* p)
{
    delete p;
}
