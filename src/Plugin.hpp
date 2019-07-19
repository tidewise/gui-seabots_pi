#ifndef SEABOTS_PI_PLUGIN_HPP
#define SEABOTS_PI_PLUGIN_HPP

#include <wx/wx.h>
#include "OCPNInterfaceImpl.hpp"
#include "ocpn_plugin.h"

#include <GL/gl.h>

namespace RTT {
    class TaskContext;
    namespace base {
        class ActivityInterface;
    }
}

namespace seabots_pi {
    class OCPNInterfaceImpl;

    class Plugin : public opencpn_plugin_116 {
        friend class orogenTaskTimer;

        struct orogenTaskTimer : public wxTimer
        {
            Plugin& plugin;
            orogenTaskTimer(Plugin& plugin);
            void Notify();
        };

        static const int REQUIRED_API_VERSION_MAJOR = 1;
        static const int REQUIRED_API_VERSION_MINOR = 16;
        static const int PLUGIN_VERSION_MAJOR = 0;
        static const int PLUGIN_VERSION_MINOR = 1;

        static const int UPDATE_PERIOD_MS = 100;

        static const char* NAME;
        static const char* DESCRIPTION_SHORT;
        static const char* DESCRIPTION_LONG;

        static const int TOOL_EXECUTE_ROOT_POSITION = -1;

        typedef std::vector<RTT::TaskContext*> Tasks;
        Tasks tasks;
        typedef std::vector<RTT::base::ActivityInterface*> Activities;
        Activities activities;

        void setupTaskActivity(
            RTT::TaskContext* task,
            RTT::base::ActivityInterface* activity = nullptr
        );

        OCPNInterfaceImpl* mInterface;

        orogenTaskTimer mTimer;
        void executeTasks();

        void loadSVGs();
        wxString readDataFile(wxString const& name);
        void setupToolbar();

        wxString mSeabotsSVG;
        wxBitmap mSeabotsBitmap;

        wxString mExecuteRouteSVG;
        wxString mExecuteRouteSVGToggled;
        int mExecuteRoutePosition = -1;
        int mExecuteRouteTool = -1;
        int GetToolbarToolCount(void);
        void OnToolbarToolCallback(int id);

        GLuint mTrajectoryGLProgramID = 0;
        GLint mTrajectoryPointPositionAttribute = 0;
        GLint mTrajectoryViewTransformUniform = 0;

        uint mPlannedTrajectoryID = 0;
        uint mPlannedTrajectoryCount = 0;
        std::vector<uint> mPlannedTrajectoryVAOs;
        std::vector<uint> mPlannedTrajectoryVBOs;

        void glCheckErrors(const char *file, int line, bool throwOnError);
        void glLoadPrograms();
        GLuint glLoadProgram(wxString name);
        GLuint glLoadShader(wxString name, GLenum shaderType);
        void glAllocateTrajectoryArrays(int neededSize);
        void glUploadSampledTrajectory(
            int i, OCPNInterfaceImpl::SampledTrajectory const&,
            PlugIn_ViewPort* vp
        );

    public:
        Plugin(void* pptr);

        int Init();
        bool DeInit();

        int GetAPIVersionMajor() { return REQUIRED_API_VERSION_MAJOR; }
        int GetAPIVersionMinor() { return REQUIRED_API_VERSION_MINOR; }
        int GetPlugInVersionMajor() { return PLUGIN_VERSION_MAJOR; }
        int GetPlugInVersionMinor() { return PLUGIN_VERSION_MINOR; }
        wxBitmap *GetPlugInBitmap();

        virtual bool RenderGLOverlayMultiCanvas(wxGLContext *pcontext, PlugIn_ViewPort *vp, int index);

        bool executeCurrentRoute();

        wxString GetCommonName() { return NAME; }
        wxString GetShortDescription() { return DESCRIPTION_SHORT; }
        wxString GetLongDescription() { return DESCRIPTION_LONG; }
    };
}

#endif